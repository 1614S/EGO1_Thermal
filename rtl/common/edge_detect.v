`timescale 1ns/1ps
//======================================================================
// edge_detect.v —— 上升沿 / 下降沿单拍脉冲
//======================================================================
module edge_detect (
    input  wire clk,
    input  wire rst_n,
    input  wire sig,
    output wire rise,
    output wire fall
);

    reg sig_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sig_d <= 1'b0;
        else        sig_d <= sig;
    end

    assign rise = sig & ~sig_d;
    assign fall = ~sig & sig_d;

endmodule
