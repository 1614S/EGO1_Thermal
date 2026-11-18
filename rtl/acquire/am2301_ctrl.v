`timescale 1ns/1ps
//======================================================================
// am2301_ctrl.v —— AM2302 / DHT22 单总线温湿度传感器驱动（AM2301/DHT21 协议兼容，可直接替换）
//
// 时序（100 MHz = 10 ns/cycle）：
//   主机拉低 1.2 ms -> 释放 -> 传感器 80us 低 + 80us 高
//   -> 40 bit：每 bit 50us 低 + 高电平 26~28us(=0) / 70us(=1)
//   数据：湿度[15:0] 温度[15:0] 校验和[7:0]
//   温度：bit15 为符号，数值 = T*10
//
// 采样周期 >= 2 s（器件限制），由 PERIOD_MS 控制。
// 输出：t_q8_8 / rh_q8_8（Q8.8），valid / err 单拍。
//======================================================================
`include "ego1_thermal_pkg.vh"

module am2301_ctrl #(
    parameter integer CLK_HZ    = 100_000_000,
    parameter integer PERIOD_US = 2_000_000
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_us,      // 真实 1 µs（不参与仿真加速）
    inout  wire               dq,           // 单总线，外部 4.7k~10k 上拉
    output reg  signed [15:0] t_q8_8,
    output reg         [15:0] rh_q8_8,
    output reg                valid,        // 单拍
    output reg                err,          // 单拍
    output reg         [15:0] err_cnt
);

    //------------------------------------------------------------------
    // 时间常数（单位：clk 周期）
    //------------------------------------------------------------------
    localparam integer C_START_LOW = CLK_HZ / 1000 * 12 / 10;   // 1.2 ms
    localparam integer C_TIMEOUT   = CLK_HZ / 1000 * 3 / 10;    // 300 us
    localparam integer C_BIT_TH    = CLK_HZ / 1000000 * 48;     // 48 us
    localparam integer C_TOTAL_MAX = CLK_HZ / 1000 * 20;        // 20 ms

    //------------------------------------------------------------------
    // 状态编码
    //------------------------------------------------------------------
    localparam [3:0] S_IDLE    = 4'd0;
    localparam [3:0] S_START_L = 4'd1;
    localparam [3:0] S_RELEASE = 4'd2;
    localparam [3:0] S_RESP_L  = 4'd3;
    localparam [3:0] S_RESP_H  = 4'd4;
    localparam [3:0] S_BIT_L   = 4'd5;
    localparam [3:0] S_BIT_H   = 4'd6;
    localparam [3:0] S_CHECK   = 4'd7;
    localparam [3:0] S_ERROR   = 4'd8;

    reg [3:0]  st;
    reg [21:0] cnt;
    reg [21:0] tot;
    reg [5:0]  biti;
    reg [39:0] sh;
    reg        bit_v;
    reg        drive_low;
    reg [20:0] us_cnt;
    reg        start_pulse;

    //------------------------------------------------------------------
    // 三态总线 + 同步
    //------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg dq_m;
    (* ASYNC_REG = "TRUE" *) reg dq_s;

    assign dq = drive_low ? 1'b0 : 1'bz;

    // dq_m 是较新的一拍，dq_s 是较旧的一拍（两级同步）
    wire rise = dq_m & ~dq_s;
    wire fall = ~dq_m & dq_s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dq_m <= 1'b1;
            dq_s <= 1'b1;
        end else begin
            dq_m <= dq;
            dq_s <= dq_m;
        end
    end

    //------------------------------------------------------------------
    // 采样周期定时
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            us_cnt      <= 21'd0;
            start_pulse <= 1'b0;
        end else begin
            start_pulse <= 1'b0;
            if (st == S_IDLE) begin
                if (us_cnt >= PERIOD_US-1) begin
                    us_cnt      <= 21'd0;
                    start_pulse <= 1'b1;
                end else if (tick_us) begin
                    us_cnt <= us_cnt + 21'd1;
                end
            end
        end
    end

    //------------------------------------------------------------------
    // 数据字段
    //------------------------------------------------------------------
    wire [15:0] rh_raw = sh[39:24];
    wire [15:0] t_raw  = sh[23:8];
    wire [7:0]  chk    = sh[7:0];
    wire [7:0]  sum8   = rh_raw[15:8] + rh_raw[7:0] + t_raw[15:8] + t_raw[7:0];

    // 注意：乘法必须显式扩到 32 bit，否则会按 16 bit 上下文截断
    //   t*10 -> Q8.8 的系数是 256/10 = 25.6 = 6554/256
    wire [31:0] t_mag  = ({1'b0, t_raw[14:0]} * 32'd6554) >> 8;
    wire [31:0] rh_mag = (rh_raw * 32'd6554) >> 8;

    //------------------------------------------------------------------
    // 主状态机
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            cnt       <= 22'd0;
            tot       <= 22'd0;
            biti      <= 6'd0;
            sh        <= 40'd0;
            drive_low <= 1'b0;
            t_q8_8    <= 16'sd0;
            rh_q8_8   <= 16'd0;
            valid     <= 1'b0;
            err       <= 1'b0;
            err_cnt   <= 16'd0;
        end else begin
            valid <= 1'b0;
            err   <= 1'b0;

            if (st != S_IDLE) tot <= tot + 22'd1;
            if (tot > C_TOTAL_MAX) st <= S_ERROR;

            case (st)
                S_IDLE: begin
                    drive_low <= 1'b0;
                    tot       <= 22'd0;
                    cnt       <= 22'd0;
                    if (start_pulse) begin
                        drive_low <= 1'b1;
                        st        <= S_START_L;
                    end
                end

                S_START_L: begin
                    cnt <= cnt + 22'd1;
                    if (cnt >= C_START_LOW) begin
                        drive_low <= 1'b0;
                        cnt       <= 22'd0;
                        st        <= S_RELEASE;
                    end
                end

                S_RELEASE: begin
                    cnt <= cnt + 22'd1;
                    if (fall) begin
                        cnt <= 22'd0;
                        st  <= S_RESP_L;
                    end else if (cnt > C_TIMEOUT) begin
                        st <= S_ERROR;
                    end
                end

                S_RESP_L: begin
                    cnt <= cnt + 22'd1;
                    if (rise) begin
                        cnt <= 22'd0;
                        st  <= S_RESP_H;
                    end else if (cnt > C_TIMEOUT) begin
                        st <= S_ERROR;
                    end
                end

                S_RESP_H: begin
                    cnt <= cnt + 22'd1;
                    if (fall) begin
                        cnt  <= 22'd0;
                        biti <= 6'd0;
                        st   <= S_BIT_L;
                    end else if (cnt > C_TIMEOUT) begin
                        st <= S_ERROR;
                    end
                end

                S_BIT_L: begin
                    if (rise) begin
                        cnt <= 22'd0;
                        st  <= S_BIT_H;
                    end else begin
                        cnt <= cnt + 22'd1;
                        if (cnt > C_TIMEOUT) st <= S_ERROR;
                    end
                end

                S_BIT_H: begin
                    if (fall) begin
                        bit_v = (cnt > C_BIT_TH);
                        sh    <= {sh[38:0], bit_v};
                        cnt   <= 22'd0;
                        if (biti == 6'd39) begin
                            st <= S_CHECK;
                        end else begin
                            biti <= biti + 6'd1;
                            st   <= S_BIT_L;
                        end
                    end else begin
                        cnt <= cnt + 22'd1;
                        if (cnt > C_TIMEOUT) st <= S_ERROR;
                    end
                end

                S_CHECK: begin
                    if (sum8 == chk) begin
                        if (t_raw[15])
                            t_q8_8 <= -$signed(t_mag[15:0]);
                        else
                            t_q8_8 <=  $signed(t_mag[15:0]);
                        rh_q8_8 <= rh_mag[15:0];
                        valid   <= 1'b1;
                    end else begin
                        err     <= 1'b1;
                        err_cnt <= err_cnt + 16'd1;
                    end
                    st <= S_IDLE;
                end

                S_ERROR: begin
                    drive_low <= 1'b0;
                    err       <= 1'b1;
                    err_cnt   <= err_cnt + 16'd1;
                    st        <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
