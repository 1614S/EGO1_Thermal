`timescale 1ns/1ps
//======================================================================
// uart_rx.v —— 8N1 接收，起始位中点对齐
// 先等半个位宽确认起始位，再每 DIV 个周期采样一位。
//======================================================================
`include "ego1_thermal_pkg.vh"

module uart_rx #(
    parameter integer DIV = `UART_DIV
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid,      // 单拍
    output reg        frame_err   // 停止位为 0 时置位（单拍）
);

    localparam integer HALF = (DIV > 2) ? (DIV/2) : 1;

    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_START = 2'd1;
    localparam [1:0] S_DATA  = 2'd2;
    localparam [1:0] S_STOP  = 2'd3;

    (* ASYNC_REG = "TRUE" *) reg rx_meta;
    (* ASYNC_REG = "TRUE" *) reg rx_sync;

    reg [1:0]  st;
    reg [15:0] cnt;
    reg [2:0]  biti;
    reg [7:0]  sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            cnt       <= 16'd0;
            biti      <= 3'd0;
            sh        <= 8'd0;
            data      <= 8'd0;
            valid     <= 1'b0;
            frame_err <= 1'b0;
        end else begin
            valid     <= 1'b0;
            frame_err <= 1'b0;

            case (st)
                S_IDLE: begin
                    if (!rx_sync) begin
                        cnt <= HALF;
                        st  <= S_START;
                    end
                end

                S_START: begin
                    if (cnt == 16'd0) begin
                        if (!rx_sync) begin
                            cnt  <= DIV-1;
                            biti <= 3'd0;
                            st   <= S_DATA;
                        end else begin
                            st <= S_IDLE;      // 毛刺
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                S_DATA: begin
                    if (cnt == 16'd0) begin
                        sh[biti] <= rx_sync;
                        cnt      <= DIV-1;
                        if (biti == 3'd7) st <= S_STOP;
                        else              biti <= biti + 3'd1;
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                S_STOP: begin
                    if (cnt == 16'd0) begin
                        data <= sh;
                        if (rx_sync) valid <= 1'b1;
                        else         frame_err <= 1'b1;
                        st <= S_IDLE;
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
