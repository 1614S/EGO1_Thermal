`timescale 1ns/1ps
//======================================================================
// am2301_vplant.v —— 虚拟 AM2301（单总线 1-Wire 从机）
//
// 目的：让"现有的 am2301_ctrl 驱动一行不改"就能读到虚拟环境温度。
// 它按真实器件的电气时序应答，而不是直接给数值：
//   主机拉低 ≥1 ms → 释放 → 从机等 25 µs → 拉低 80 µs → 释放 80 µs
//   → 40 bit：每 bit 拉低 48 µs（主机判据 48 µs 分界），
//     高电平 26 µs = 0 / 70 µs = 1
//   数据：RH[15:0]  T[15:0]（bit15 为符号，数值 = ℃×10）  CHK[7:0]
//
// 误差注入：偏置 +0.60 ℃（`VP_AM2301_OFF_UK`）；每 N 次转换故意写错校验和，
//          用来验证驱动里的 err_cnt / 重试路径真的有效。
//======================================================================
`include "vplant_pkg.vh"

module am2301_vplant #(
    parameter integer ERR_PER = 64        // 每 N 次转换注入一次校验错
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_us,      // 真实 1 µs（不参与仿真加速）
    input  wire               en,           // 1 = 虚拟器件接管总线
    input  wire signed [31:0] ta_uk,        // 虚拟环境温度（µK）
    input  wire        [15:0] rh_pm,        // 相对湿度千分比（550 = 55.0%）
    inout  wire               dq,
    // 监测
    output reg  [39:0] last_frame,
    output reg         frame_ok,
    output reg  [15:0] conv_cnt,
    output reg  [15:0] err_inj_cnt
);
    localparam [2:0] S_IDLE = 3'd0, S_HOSTLOW = 3'd1, S_DLY = 3'd2,
                     S_RL = 3'd3, S_RH = 3'd4, S_BL = 3'd5, S_BH = 3'd6,
                     S_END = 3'd7;

    reg [2:0]  st;
    reg [15:0] usec;
    reg [5:0]  biti;
    reg        drive_low;
    reg [39:0] sh;
    reg [15:0] lfsr;

    // 总线
    assign dq = (drive_low && en) ? 1'b0 : 1'bz;

    // 同步（主机也在驱动同一条线）
    (* ASYNC_REG = "TRUE" *) reg dq_m, dq_s;
    always @(posedge clk) begin
        if (!rst_n) begin dq_m <= 1'b1; dq_s <= 1'b1; end
        else begin dq_m <= dq; dq_s <= dq_m; end
    end

    //------------------------------------------------------------------
    // 待发送的 40 bit：在事务开始前算好（含误差与校验和）
    //------------------------------------------------------------------
    // ℃×10 = µK / 100000。常数除法会被综合成"乘高位 + 修正"的长进位链
    // （实测 21 级 CARRY4、单周期 −12.8 ns），改成 Q32 倒数乘法：1/1e5 ≈ 42950/2^32
    wire signed [31:0] ta_err = ta_uk + $signed({1'b0, `VP_AM2301_OFF_UK});
    wire signed [47:0] t10_w  = ($signed(ta_err) * 48'sd42950) >>> 32;

    // 两级流水：℃×10 打一拍，帧内容再打一拍（ta_uk 变化极慢，滞后 1 µs 无影响）
    reg  signed [31:0] t10_r;
    always @(posedge clk) begin
        if (!rst_n)       t10_r <= 32'sd0;
        else if (tick_us) t10_r <= t10_w[31:0];
    end
    wire signed [31:0] t10 = t10_r;
    wire        [15:0] t_abs  = (t10 < 0) ? -t10[15:0] : t10[15:0];
    wire        [15:0] t_field= { (t10 < 0), t_abs[14:0] };
    wire        [15:0] rh_field = rh_pm;
    wire         [7:0] chk    = rh_field[15:8] + rh_field[7:0]
                              + t_field[15:8] + t_field[7:0];
    // 每 VP_AM2301_ERR_PER 次转换注入一次校验错
    wire         inject = (conv_cnt != 16'd0) &&
                          ((conv_cnt % ERR_PER) == 0);
    wire   [7:0] chk_tx = inject ? (chk ^ 8'h01) : chk;
    wire [39:0] frame  = {rh_field, t_field, chk_tx};

    // 帧内容寄存器化：把"乘法 + 求和 + 拼接"从组合路径上拿下来
    reg [39:0] frame_r;
    always @(posedge clk) begin
        if (!rst_n) frame_r <= 40'd0;
        else if (tick_us) frame_r <= frame;
    end

    wire bitv = sh[39 - biti];
    wire [15:0] high_us = bitv ? 16'd70 : 16'd26;

    //------------------------------------------------------------------
    // 1-Wire 时序状态机
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            usec        <= 16'd0;
            biti        <= 6'd0;
            drive_low   <= 1'b0;
            sh          <= 40'd0;
            lfsr        <= 16'h6B21;
            last_frame  <= 40'd0;
            frame_ok    <= 1'b0;
            conv_cnt    <= 16'd0;
            err_inj_cnt <= 16'd0;
        end else if (tick_us) begin
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[12] ^ lfsr[5] ^ lfsr[0]};
            if (!en) begin
                st        <= S_IDLE;
                drive_low <= 1'b0;
                usec      <= 16'd0;
            end else begin
                case (st)
                    S_IDLE: begin
                        drive_low <= 1'b0;
                        if (!dq_s) begin                 // 主机开始拉低
                            usec <= 16'd0;
                            st   <= S_HOSTLOW;
                        end
                    end
                    S_HOSTLOW: begin
                        if (dq_s) begin                  // 主机释放
                            usec <= 16'd0;
                            st   <= S_DLY;
                        end else if (usec < 16'hFFFF) usec <= usec + 16'd1;
                    end
                    S_DLY: begin
                        drive_low <= 1'b0;
                        if (usec >= 16'd25) begin usec <= 16'd0; st <= S_RL; end
                        else usec <= usec + 16'd1;
                    end
                    S_RL: begin                      // 应答低 80 µs
                        drive_low <= 1'b1;
                        if (usec >= 16'd80) begin usec <= 16'd0; st <= S_RH; end
                        else usec <= usec + 16'd1;
                    end
                    S_RH: begin                      // 应答高 80 µs
                        drive_low <= 1'b0;
                        if (usec >= 16'd80) begin
                            usec <= 16'd0; biti <= 6'd0;
                            sh   <= frame_r;         // 锁存本帧（已寄存器化）
                            conv_cnt <= conv_cnt + 16'd1;
                            if (inject) err_inj_cnt <= err_inj_cnt + 16'd1;
                            st   <= S_BL;
                        end else usec <= usec + 16'd1;
                    end
                    S_BL: begin                      // 每位先拉低 48 µs
                        drive_low <= 1'b1;
                        if (usec >= 16'd48) begin usec <= 16'd0; st <= S_BH; end
                        else usec <= usec + 16'd1;
                    end
                    S_BH: begin                      // 再释放 26/70 µs
                        drive_low <= 1'b0;
                        if (usec >= high_us) begin
                            usec <= 16'd0;
                            if (biti == 6'd39) begin
                                drive_low <= 1'b1;   // 第 40 位后必须有一个收尾低脉冲
                                st        <= S_END;  // 真实 DHT22 就是这样，少了它会卡死主机
                            end else begin
                                biti <= biti + 6'd1;
                                st   <= S_BL;
                            end
                        end else usec <= usec + 16'd1;
                    end
                    S_END: begin                     // 收尾低脉冲 50 µs 后释放总线
                        drive_low <= 1'b1;
                        if (usec >= 16'd50) begin
                            drive_low  <= 1'b0;
                            last_frame <= sh;
                            frame_ok   <= 1'b1;
                            usec       <= 16'd0;
                            st         <= S_IDLE;
                        end else usec <= usec + 16'd1;
                    end
                    default: st <= S_IDLE;
                endcase
            end
        end
    end

endmodule



