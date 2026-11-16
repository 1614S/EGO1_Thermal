`timescale 1ns/1ps
//======================================================================
// moving_avg.v —— N 点滑动平均（Q8.8 有符号）
// N 由参数给出，默认 8；只用加法与移位，无除法。
//
// 预热（primed）：第一个样本把整个窗口填满，输出直接等于该样本。
//   否则上电后窗口里全是 0，输出会从 0 慢慢爬上来 —— 这会让 PI 在
//   上电头几秒看到假的"低温"，把设定值斜坡起点锁到 0（实测踩过）。
//======================================================================
module moving_avg #(
    parameter integer N = 8,
    parameter integer SHIFT = 3        // N = 2^SHIFT
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid,   // 单拍
    input  wire signed [15:0] din,
    output reg  signed [15:0] dout,
    output reg                dout_valid,
    output reg                primed   // 窗口已填满（监测用）
);

    reg signed [15:0] hist [0:N-1];
    reg signed [19:0] sum;
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < N; i = i + 1)
                hist[i] <= 16'sd0;
            sum        <= 20'sd0;
            dout       <= 16'sd0;
            dout_valid <= 1'b0;
            primed     <= 1'b0;
        end else begin
            dout_valid <= 1'b0;
            if (valid) begin
                if (!primed) begin
                    for (i = 0; i < N; i = i + 1)
                        hist[i] <= din;
                    dout   <= din;
                    primed <= 1'b1;
                end else begin
                    sum = 20'sd0;
                    for (i = 0; i < N-1; i = i + 1)
                        sum = sum + hist[i];
                    sum = sum + din;
                    for (i = N-1; i > 0; i = i - 1)
                        hist[i] <= hist[i-1];
                    hist[0] <= din;
                    dout <= sum >>> SHIFT;
                end
                dout_valid <= 1'b1;
            end
        end
    end

endmodule
