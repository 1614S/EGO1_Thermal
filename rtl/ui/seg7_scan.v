`timescale 1ns/1ps
//======================================================================
// seg7_scan.v —— 4 位动态扫描 + BCD 译码
//
// DIG_CLK: 每位点亮时间（100 MHz / 100000 = 1 kHz/位）
//
// 极性：EGO1 板载数码管为**共阴极，段选与位选都是高有效**
//       （docs/09 §2 实测记录）。因此默认 SEG_ACT_LOW=0、AN_ACT_LOW=0。
//       若换到共阳板子，把这两个参数改成 1 即可，逻辑不用动。
//
// an[7:0]：只扫描低 4 位，高 4 位恒灭（板上另一组数码管不点亮）。
// 4'hF 是“空白”码，整位熄灭。
//======================================================================
module seg7_scan #(
    parameter integer DIV         = 100000,
    parameter integer SEG_ACT_LOW = 0,      // 0 = 段选高有效（EGO1 共阴）
    parameter integer AN_ACT_LOW  = 0       // 0 = 位选高有效（EGO1 共阴）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] digits,      // 4 组 BCD，digits[3:0] 为最低位
    input  wire [3:0]  dp,          // 小数点，1 点亮
    output wire [7:0]  seg,         // {dp,g,f,e,d,c,b,a}
    output wire [3:0]  an4          // 本位组 4 个位选（bit0 = 最左），由顶层拼成 an[7:0]
);

    localparam integer W = (DIV > 1) ? $clog2(DIV) : 1;

    reg [W-1:0] cnt;
    reg [1:0]   dig;
    reg [3:0]   nib;
    reg [7:0]   seg_l;              // 内部统一按“低有效”表示，输出前再转换
    reg [3:0]   an_l;
    reg         dp_r;

    //------------------------------------------------------------------
    // 扫描计数
    //------------------------------------------------------------------
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

    //------------------------------------------------------------------
    // 译码 + 位选（内部低有效）
    //------------------------------------------------------------------
    always @(*) begin
        an_l = 4'b1111;             // 低有效：1 = 灭

        case (dig)
            2'd0: begin nib = digits[3:0];   an_l = 4'b1110; dp_r = dp[0]; end
            2'd1: begin nib = digits[7:4];   an_l = 4'b1101; dp_r = dp[1]; end
            2'd2: begin nib = digits[11:8];  an_l = 4'b1011; dp_r = dp[2]; end
            default: begin nib = digits[15:12]; an_l = 4'b0111; dp_r = dp[3]; end
        endcase

        case (nib)
            4'h0: seg_l = 8'hC0;
            4'h1: seg_l = 8'hF9;
            4'h2: seg_l = 8'hA4;
            4'h3: seg_l = 8'hB0;
            4'h4: seg_l = 8'h99;
            4'h5: seg_l = 8'h92;
            4'h6: seg_l = 8'h82;
            4'h7: seg_l = 8'hF8;
            4'h8: seg_l = 8'h80;
            4'h9: seg_l = 8'h90;
            4'hA: seg_l = 8'h88;
            4'hB: seg_l = 8'h83;
            4'hC: seg_l = 8'hC6;
            4'hD: seg_l = 8'hA1;
            4'hE: seg_l = 8'h86;
            4'hF: seg_l = 8'hFF;        // 空白
            default: seg_l = 8'hFF;
        endcase

        if (nib != 4'hF) begin          // 空白位不加小数点
            if (dp_r) seg_l = seg_l & 8'h7F;
            else      seg_l = seg_l | 8'h80;
        end
    end

    //------------------------------------------------------------------
    // 极性转换
    //------------------------------------------------------------------
    assign seg = SEG_ACT_LOW ? seg_l : ~seg_l;
    assign an4 = AN_ACT_LOW  ? an_l  : ~an_l;

endmodule
