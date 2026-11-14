//======================================================================
// uart_tx.v —— 8N1 发送
// DIV = clk / baud。100 MHz / 115200 = 868（误差 0.006 %）。
// 仿真时 DIV 由 pkg 改为 10（10 Mbps），行为一致。
//======================================================================
`include "ego1_thermal_pkg.vh"

module uart_tx #(
    parameter integer DIV = `UART_DIV
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,      // 单拍启动
    input  wire [7:0] data,
    output reg        busy,
    output reg        tx
);

    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_START = 2'd1;
    localparam [1:0] S_DATA  = 2'd2;
    localparam [1:0] S_STOP  = 2'd3;

    reg [1:0]  st;
    reg [15:0] cnt;
    reg [3:0]  biti;
    reg [7:0]  sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st   <= S_IDLE;
            cnt  <= 16'd0;
            biti <= 4'd0;
            sh   <= 8'd0;
            tx   <= 1'b1;
            busy <= 1'b0;
        end else begin
            case (st)
                S_IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        sh   <= data;
                        busy <= 1'b1;
                        cnt  <= 16'd0;
                        st   <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0;
                    if (cnt == DIV-1) begin
                        cnt  <= 16'd0;
                        biti <= 4'd0;
                        st   <= S_DATA;
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    if (cnt == 16'd0) tx <= sh[0];
                    if (cnt == DIV-1) begin
                        cnt <= 16'd0;
                        sh  <= {1'b0, sh[7:1]};
                        if (biti == 4'd7) st <= S_STOP;
                        else              biti <= biti + 4'd1;
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                S_STOP: begin
                    tx <= 1'b1;
                    if (cnt == DIV-1) begin
                        cnt  <= 16'd0;
                        busy <= 1'b0;
                        st   <= S_IDLE;
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
