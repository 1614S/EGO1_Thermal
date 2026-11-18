`timescale 1ns/1ps
//======================================================================
// seg7_scan.v —— 4 位动态扫描 + BCD 译码
// DIG_CLK: 每位点亮时间（100 MHz / 100000 = 1 kHz/位）
// 段码与位选均为低有效（共阳数码管）
//======================================================================
module seg7_scan #(
    parameter integer DIV = 100000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] digits,      // 4 组 BCD，digits[3:0] 为最低位
    input  wire [3:0]  dp,          // 小数点，1 点亮
    output reg  [7:0]  seg,         // {dp,g,f,e,d,c,b,a} 低有效
    output reg  [3:0]  an           // 位选，低有效
);

    localparam integer W = (DIV > 1) ? $clog2(DIV) : 1;

    reg [W-1:0] cnt;
    reg [1:0]   dig;
    reg [3:0]   nib;
    reg [7:0]   seg_r;
    reg         dp_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= {W{1'b0}};
            dig <= 2'd0;
        end else if (cnt >= DIV-1) begin
            cnt <= {W{1'b0}};
            dig <= dig + 2'd1;
        end else begin
            cnt <= cnt + 1'b1;
        end
    end

    always @(*) begin
        case (dig)
            2'd0: begin nib = digits[3:0];   an = 4'b1110; dp_r = dp[0]; end
            2'd1: begin nib = digits[7:4];   an = 4'b1101; dp_r = dp[1]; end
            2'd2: begin nib = digits[11:8];  an = 4'b1011; dp_r = dp[2]; end
            default: begin nib = digits[15:12]; an = 4'b0111; dp_r = dp[3]; end
        endcase

        case (nib)
            4'h0: seg_r = 8'hC0;
            4'h1: seg_r = 8'hF9;
            4'h2: seg_r = 8'hA4;
            4'h3: seg_r = 8'hB0;
            4'h4: seg_r = 8'h99;
            4'h5: seg_r = 8'h92;
            4'h6: seg_r = 8'h82;
            4'h7: seg_r = 8'hF8;
            4'h8: seg_r = 8'h80;
            4'h9: seg_r = 8'h90;
            4'hA: seg_r = 8'h88;
            4'hB: seg_r = 8'h83;
            4'hC: seg_r = 8'hC6;
            4'hD: seg_r = 8'hA1;
            4'hE: seg_r = 8'h86;
            default: seg_r = 8'h8E;      // 'F'
        endcase

        if (dp_r) seg = seg_r & 8'h7F;
        else      seg = seg_r | 8'h80;
    end

endmodule
