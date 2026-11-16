`timescale 1ns/1ps
//======================================================================
// power_model.v —— 芯片功耗模型 P(f, level)
//   P[mW] = P_STATIC + (f_khz × level × P_COEF) >> 14
// 标定：level=15 @150 MHz ≈ 1.57 W（含静态 60 mW）
// 说明：这是"按 XPE 思路标定的代表性负载"，不是实测功率；
//       报告里 u(P)/P 取 25%（见 sw/analyze_thermal.py）。
//======================================================================
`include "vplant_pkg.vh"

module power_model (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_ms,
    input  wire [17:0] f_khz,        // 实测频率（来自 freq_monitor）
    input  wire [3:0]  level,        // 自热负载级数 0…15
    output reg  [31:0] p_mw,
    output reg  [15:0] p_dither      // ±抖动，给"功率估计误差"用（监测）
);
    reg [15:0] lfsr;

    // 先 18×4 再 ×11，全部落在 26 位内，避免 32×32 乘法
    wire [21:0] f_times_lv = f_khz * {18'd0, level};
    wire [25:0] p_dyn26    = (f_times_lv * 5'd11) >> 14;
    wire [31:0] p_sum      = `VP_P_STATIC_MW + {6'd0, p_dyn26};

    always @(posedge clk) begin
        if (!rst_n) begin
            lfsr     <= 16'h1D2B;
            p_mw     <= `VP_P_STATIC_MW;
            p_dither <= 16'd0;
        end else if (tick_ms) begin
            lfsr     <= {lfsr[14:0], lfsr[15] ^ lfsr[14] ^ lfsr[12] ^ lfsr[3]};
            p_dither <= {10'd0, lfsr[5:0]};                       // 0…63 mW
            p_mw     <= p_sum + {26'd0, lfsr[5:0]};               // 加 0…63 mW 抖动
        end
    end

endmodule


