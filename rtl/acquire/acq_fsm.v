`timescale 1ns/1ps
//======================================================================
// acq_fsm.v —— 统一采集调度：每 100 ms 锁存一次双源数据 + 时间戳
// 外部传感器（AM2302）更新慢，用 ext_age 记录数据龄期，单位 100 ms，饱和 255。
//======================================================================
`include "ego1_thermal_pkg.vh"

module acq_fsm (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_ms,
    input  wire signed [15:0] tj_in,
    input  wire signed [15:0] text_in,
    input  wire        [15:0] rh_in,
    input  wire               text_new,      // 传感器新数据（单拍）
    output reg  signed [15:0] tj_out,
    output reg  signed [15:0] text_out,
    output reg         [15:0] rh_out,
    output reg         [15:0] ext_age,       // 0..255（100 ms 单位）
    output reg         [31:0] ts_ms,
    output reg                sample_valid
);

    reg [6:0] ph;
    reg       text_seen;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ph           <= 7'd0;
            ts_ms        <= 32'd0;
            tj_out       <= 16'sd0;
            text_out     <= 16'sd0;
            rh_out       <= 16'd0;
            ext_age      <= 16'd0;
            sample_valid <= 1'b0;
            text_seen    <= 1'b0;
        end else begin
            sample_valid <= 1'b0;

            // 传感器数据到达与采样节拍异步，必须在节拍外捕获
            if (text_new) text_seen <= 1'b1;

            if (tick_ms) begin
                ts_ms <= ts_ms + 32'd1;

                if (ph >= `SAMPLE_MS-1) begin
                    ph       <= 7'd0;
                    tj_out   <= tj_in;
                    text_out <= text_in;
                    rh_out   <= rh_in;
                    if (text_seen) begin
                        ext_age   <= 16'd0;
                        text_seen <= 1'b0;
                    end else if (ext_age < 16'd255) begin
                        ext_age <= ext_age + 16'd1;   // 100 ms 为单位（AM2302 的 2 s → 20）
                    end
                    sample_valid <= 1'b1;
                end else begin
                    ph <= ph + 7'd1;
                end
            end
        end
    end

endmodule

