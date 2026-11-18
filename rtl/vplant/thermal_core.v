`timescale 1ns/1ps
//======================================================================
// thermal_core.v —— 2R2C 双节点热网络（虚拟发热芯片的物理内核）
//
//   C1·dTj/dt = P - (Tj - Tc)/R1
//   C2·dTc/dt = (Tj - Tc)/R1 - (Tc - Ta)/R2
//
// 一、定点设计（关键是"每次乘法都必须落在 32 位内"，否则会静默溢出）
//   温度 µK（int32，25000000 = 25.000 ℃）
//   热流 q[µW]  = ΔT[µK] × (1/R)[Q12] >>> 12   —— ΔT≤2e5、1/R≤1365 → 乘积 ≤2.7e8 ✓
//   能量 E[nJ]  = P[mW]×1000 − q[µW]           —— ≤2e6 ✓
//   增量 dT[µK] = E[nJ] × (1/C)[Q17] >>> 17     —— E≤2e6、1/C≤874 → ≤1.8e9 ✓
//
// 二、时序：三级流水（每级一个乘法 + 一个加法器）
//   s1 乘 1/R → 热流   s2 能量差   s3 乘 1/C → 累加
//   不做流水时这条路径 32 级逻辑、WNS −18.5 ns（实测），必须拆。
//======================================================================
`include "vplant_pkg.vh"

module thermal_core (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_ms,
    input  wire signed [31:0] p_mw,
    input  wire signed [31:0] ta_uk,
    input  wire        [15:0] inv_r2_q12,     // 1/R2（Q12，最大 512）
    input  wire signed [31:0] t_init_uk,
    output reg  signed [31:0] tj_uk,
    output reg  signed [31:0] tc_uk,
    output wire signed [31:0] q12_uw,
    output wire signed [31:0] q2a_uw
);
    localparam signed [11:0] INV_R1_Q12 = 12'sd1365;   // 4096/3
    localparam signed [15:0] RC1_Q17    = 16'sd874;    // 131072/150
    localparam signed [15:0] RC2_Q17    = 16'sd109;    // 131072/1200

    wire signed [31:0] dt12 = tj_uk - tc_uk;
    wire signed [31:0] dt2a = tc_uk - ta_uk;

    reg tick_d1, tick_d2;

    reg signed [31:0] q12_r, q2a_r;
    reg signed [31:0] e1_r, e2_r;

    assign q12_uw = q12_r;
    assign q2a_uw = q2a_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            tick_d1 <= 1'b0;
            tick_d2 <= 1'b0;
            q12_r   <= 32'sd0;
            q2a_r   <= 32'sd0;
            e1_r    <= 32'sd0;
            e2_r    <= 32'sd0;
            tj_uk   <= t_init_uk;
            tc_uk   <= t_init_uk;
        end else begin
            tick_d1 <= tick_ms;
            tick_d2 <= tick_d1;

            // s1：q = ΔT × (1/R) >> 12
            if (tick_ms) begin
                q12_r <= ((dt12 >>> 8) * INV_R1_Q12)     >>> 4;   // 先右移再乘，避免 32 位溢出
                q2a_r <= ((dt2a >>> 8) * $signed({1'b0, inv_r2_q12})) >>> 4;
            end

            // s2：E = P·1ms − q
            if (tick_d1) begin
                e1_r <= p_mw * 32'sd1000 - q12_r;
                e2_r <= q12_r - q2a_r;
            end

            // s3：ΔT = E × (1/C) >> 17，并累加
            if (tick_d2) begin
                tj_uk <= tj_uk + (((e1_r >>> 4) * RC1_Q17) >>> 13);
                tc_uk <= tc_uk + (((e2_r >>> 4) * RC2_Q17) >>> 13);
            end
        end
    end

endmodule




