`timescale 1ns/1ps
//======================================================================
// iir_lpf.v —— 一阶 IIR：y += (x - y) >>> K
// K=2 时 τ ≈ 4 个采样点（100 ms 采样 -> 0.4 s）
//
// 预热：第一个样本直接装载 y = x，避免上电从 0 爬升。
//======================================================================
module iir_lpf #(
    parameter integer K = 2
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid,
    input  wire signed [15:0] din,
    output reg  signed [15:0] dout,
    output reg                dout_valid,
    output reg                primed
);

    reg signed [17:0] y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y          <= 18'sd0;
            dout       <= 16'sd0;
            dout_valid <= 1'b0;
            primed     <= 1'b0;
        end else begin
            dout_valid <= 1'b0;
            if (valid) begin
                if (!primed) begin
                    y      <= {{2{din[15]}}, din};
                    dout   <= din;
                    primed <= 1'b1;
                end else begin
                    y    <= y + (($signed({{2{din[15]}}, din}) - y) >>> K);
                    dout <= y[15:0];
                end
                dout_valid <= 1'b1;
            end
        end
    end

endmodule
