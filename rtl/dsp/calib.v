`timescale 1ns/1ps
//======================================================================
// calib.v —— 两点标定：y = ((x - b) * a) >>> 12
//   a: 增益 Q4.12（默认 1.0 = 0x1000）
//   b: 偏置 Q8.8
// 结果饱和到 16 bit 有符号。
//======================================================================
module calib (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid,
    input  wire signed [15:0] din,
    input  wire        [15:0] gain_q4_12,
    input  wire signed [15:0] offset_q8_8,
    output reg  signed [15:0] dout,
    output reg                dout_valid
);

    reg signed [16:0] d;
    reg signed [36:0] p;
    reg signed [24:0] q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d          <= 17'sd0;
            p          <= 37'sd0;
            q          <= 25'sd0;
            dout       <= 16'sd0;
            dout_valid <= 1'b0;
        end else begin
            dout_valid <= 1'b0;
            if (valid) begin
                d = $signed({{1{din[15]}}, din}) - $signed({{1{offset_q8_8[15]}}, offset_q8_8});
                // 显式把增益扩到 32 bit，避免乘法被 16/17 bit 上下文截断
                p = d * $signed({16'd0, gain_q4_12});
                q = p >>> 12;
                if (q > 25'sd32000)       dout <= 16'sd32000;
                else if (q < -25'sd10240) dout <= -16'sd10240;
                else                      dout <= q[15:0];
                dout_valid <= 1'b1;
            end
        end
    end

endmodule
