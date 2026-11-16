`timescale 1ns/1ps
//======================================================================
// freq_monitor.v —— 频率监测（1 ms 闸门）
//
// 被测时钟可能高于采样时钟（150 MHz vs 200 MHz），直接数边沿会混叠。
// 做法：先在 clk_test 域做 32 分频（一个翻转位），再在参考域数翻转次数，
//       f_khz = 翻转次数 × 32（1 ms 窗口）。
//======================================================================
module freq_monitor #(
    parameter integer CLK_HZ = 200_000_000,
    parameter integer PRE    = 32
) (
    input  wire        clk,          // 参考时钟（200 MHz）
    input  wire        rst_n,
    input  wire        clk_test,     // 被测时钟（clk_core）
    output reg  [17:0] f_khz,
    output reg         valid
);

    localparam integer GATE = CLK_HZ / 1000;      // 1 ms

    //------------------------------------------------------------------
    // 被测域：PRE 分频，输出一个翻转位
    //------------------------------------------------------------------
    reg [4:0] pre;
    reg       tog;

    always @(posedge clk_test or negedge rst_n) begin
        if (!rst_n) begin
            pre <= 5'd0;
            tog <= 1'b0;
        end else if (pre >= PRE-1) begin
            pre <= 5'd0;
            tog <= ~tog;
        end else begin
            pre <= pre + 5'd1;
        end
    end

    //------------------------------------------------------------------
    // 参考域：同步 + 数翻转
    //------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg t1, t2, t3;
    reg [15:0] cnt;
    reg [17:0] gate;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t1 <= 1'b0;
            t2 <= 1'b0;
            t3 <= 1'b0;
            cnt  <= 16'd0;
            gate <= 18'd0;
            f_khz <= 18'd0;
            valid <= 1'b0;
        end else begin
            t1 <= tog;
            t2 <= t1;
            t3 <= t2;
            valid <= 1'b0;

            if (t2 ^ t3) cnt <= cnt + 16'd1;

            if (gate >= GATE-1) begin
                gate  <= 18'd0;
                f_khz <= {cnt, 5'b0};        // ×32
                cnt   <= 16'd0;
                valid <= 1'b1;
            end else begin
                gate <= gate + 18'd1;
            end
        end
    end

endmodule
