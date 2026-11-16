`timescale 1ns/1ps
//======================================================================
// xadc_vplant.v —— 虚拟 XADC（DRP 从机）
//
// 目的：让"现有的 xadc_frontend 一行不改"就能读到虚拟结温。
// 它在 DRP 总线上扮演 XADC，把 thermal_core 的结温按真实 XADC 的码制返回，
// 并叠加按 UG480 手册量级设定的传感器误差。
//
// 换算：T(℃) = code × 503.975 / 4096 − 273.15
//   内部用 1.024 mK 为单位（= µK >> 10），把乘法压到 22×12 / 22×14 位（各 1 个 DSP），
//   并把"增益"和"转码"拆到 DRP 状态机的两级，避免长组合路径。
//
// 误差注入（= 未标定时的手册上限，两点标定后应被消掉）：
//   增益 0.99805（XADC 基准误差按 Kelvin 1:1 放大）、偏置 +1.80 ℃、
//   噪声 ±0.143 ℃、0.123 ℃/LSB 量化（12 bit 天然产生）
//======================================================================
`include "vplant_pkg.vh"

module xadc_vplant (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,            // 1 = 虚拟器件接管 DRP
    input  wire signed [31:0] tj_uk,         // 虚拟结温（µK）
    input  wire        [31:0] p_mw,          // 功耗（用于 VCCINT 压降）
    input  wire               den,
    input  wire               dwe,
    input  wire        [6:0]  daddr,
    input  wire        [15:0] di,
    output reg  [15:0] dout,
    output reg                drdy,
    output reg  [15:0] code_mon,
    output reg  [15:0] err_cnt
);
    // 0 ℃ 在 "1.024 mK" 单位下的值 = 273150000 >> 10
    localparam signed [21:0] T0_U   = 22'sd266748;
    localparam signed [15:0] OFF_U  = 16'sd1758;      // +1.80 ℃
    localparam        [13:0] K_CODE = 14'd8726;       // 每单位 → code（Q20）

    // VCCINT：标称 1.000 V → 1365 码，随功耗压降；VCCAUX ≈ 2.0 V
    wire [11:0] code_vccint = 12'd1365 - {6'd0, p_mw[9:4]};
    wire [11:0] code_vccaux = 12'd2731;

    reg [1:0]  st;
    reg [6:0]  addr_r;
    reg [15:0] do_r;
    reg [15:0] lfsr;

    // 温度（1.024 mK 单位，20 bit 有效）
    wire signed [21:0] t_k = ($signed(tj_uk) >>> 10) + T0_U;

    // 噪声：±143 单位（±0.143 ℃），乘加代替除法
    wire signed [9:0]  nz      = {1'b0, lfsr[8:0]} - 10'sd256;
    wire signed [15:0] noise_u = (nz * 10'sd36) >>> 6;

    reg signed [31:0] t_kelv;
    reg signed [31:0] t_err;
    // 增益：22 bit × 12 bit → 34 bit（必须显式放宽，否则在 22 位上下文里被截断）
    wire signed [33:0] t_g = ($signed(t_k) * $signed({1'b0, `VP_XADC_GAIN_Q12})) >>> 12;

    // 转码：只取 36 位乘积里有效的一段，比较也在 16 位上做
    // （原来在 48 位里比较 → 16 级 CARRY4、单周期 −1.2 ns）
    function [15:0] code_of(input signed [31:0] v);
        reg signed [47:0] prod;
        reg        [15:0] cs;
        begin
            prod = $signed(v) * $signed({1'b0, K_CODE});
            cs   = prod[35:20];
            if (v <= 0)             code_of = 16'd0;
            else if (cs > 16'd4095) code_of = 16'd4095;
            else                    code_of = {4'd0, cs[11:0]};
        end
    endfunction

    always @(posedge clk) begin
        if (!rst_n) begin
            st       <= 2'd0;
            addr_r   <= 7'd0;
            do_r     <= 16'd0;
            dout     <= 16'd0;
            drdy     <= 1'b0;
            lfsr     <= 16'h3C5A;
            code_mon <= 16'd0;
            err_cnt  <= 16'd0;
            t_kelv   <= 22'sd0;
            t_err    <= 22'sd0;
        end else begin
            drdy <= 1'b0;
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[9] ^ lfsr[2]};

            case (st)
                2'd0: if (den && !dwe) begin          // 一级：增益（1 个 DSP）
                    addr_r  <= daddr;
                    t_kelv  <= t_g[31:0];
                    st      <= 2'd1;
                    err_cnt <= err_cnt + 16'd1;
                end
                2'd1: begin                            // 二级：补回偏置/噪声
                    t_err <= t_kelv - T0_U + OFF_U + noise_u;
                    st    <= 2'd2;
                end
                2'd2: begin                            // 三级：转码（1 个 DSP）
                    do_r <= (addr_r == 7'h00) ? code_of(t_err + T0_U) :
                            (addr_r == 7'h01) ? {4'd0, code_vccint} :
                            (addr_r == 7'h02) ? {4'd0, code_vccaux} : 16'h0000;
                    st   <= 2'd3;
                end
                default: begin                         // 四级：DRDY + DO
                    dout     <= do_r;
                    drdy     <= 1'b1;
                    code_mon <= do_r;
                    st       <= 2'd0;
                end
            endcase

            if (!en) begin
                st   <= 2'd0;
                drdy <= 1'b0;
            end
        end
    end

endmodule



