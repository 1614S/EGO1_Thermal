`timescale 1ns/1ps
//======================================================================
// cdc_bit_sync.v —— 单比特 / 慢变总线跨时钟域同步（2FF）
// 只用于“慢变、可容忍单周期抖动”的信号（按键、状态位、格雷码每位）。
//======================================================================
module cdc_bit_sync #(
    parameter integer WIDTH = 1,
    parameter integer STAGES = 2
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire [WIDTH-1:0] din,
    output wire [WIDTH-1:0] dout
);

    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] sync [0:STAGES-1];
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < STAGES; i = i + 1)
                sync[i] <= {WIDTH{1'b0}};
        end else begin
            sync[0] <= din;
            for (i = 1; i < STAGES; i = i + 1)
                sync[i] <= sync[i-1];
        end
    end

    assign dout = sync[STAGES-1];

endmodule
