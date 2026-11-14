//======================================================================
// moving_avg.v —— N 点滑动平均（Q8.8 有符号）
// N 由参数给出，默认 8；只用加法与移位，无除法。
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
    output reg                dout_valid
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
        end else begin
            dout_valid <= 1'b0;
            if (valid) begin
                sum = 20'sd0;
                for (i = 0; i < N-1; i = i + 1)
                    sum = sum + hist[i];
                sum = sum + din;
                for (i = N-1; i > 0; i = i - 1)
                    hist[i] <= hist[i-1];
                hist[0] <= din;
                dout <= sum >>> SHIFT;
                dout_valid <= 1'b1;
            end
        end
    end

endmodule
