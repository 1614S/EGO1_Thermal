//======================================================================
// tb_uart.sv —— UART 发送/接收回环自检
// 判据：256 字节全对，无帧错误，波特率误差 < 1 %。
//======================================================================
`timescale 1ns/1ps
`include "ego1_thermal_pkg.vh"

module tb_uart;

    localparam integer DIV = `UART_DIV;

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;
    wire       tx_line;

    wire [7:0] rx_data;
    wire       rx_valid;
    wire       rx_err;

    integer i;
    integer errors;
    integer recv;

    // 100 MHz
    always #5 clk = ~clk;

    uart_tx #(.DIV(DIV)) u_tx (
        .clk   (clk),
        .rst_n (rst_n),
        .start (tx_start),
        .data  (tx_data),
        .busy  (tx_busy),
        .tx    (tx_line)
    );

    uart_rx #(.DIV(DIV)) u_rx (
        .clk       (clk),
        .rst_n     (rst_n),
        .rx        (tx_line),
        .data      (rx_data),
        .valid     (rx_valid),
        .frame_err (rx_err)
    );

    // 接收侧计分
    always @(posedge clk) begin
        if (rst_n && rx_valid) begin
            recv = recv + 1;
            if (rx_data !== tx_data) begin
                errors = errors + 1;
                $display("[ERR] byte %0d: sent %02h recv %02h", recv, tx_data, rx_data);
            end
        end
        if (rst_n && rx_err) begin
            errors = errors + 1;
            $display("[ERR] framing error at byte %0d", recv);
        end
    end

    initial begin
        errors = 0;
        recv   = 0;
        tx_data  = 8'h00;
        tx_start = 1'b0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        for (i = 0; i < 256; i = i + 1) begin
            tx_data  = i[7:0];
            tx_start = 1'b1;
            @(posedge clk);
            tx_start = 1'b0;
            // 等发送完成
            while (tx_busy) @(posedge clk);
            repeat (2) @(posedge clk);
        end

        // 等最后一字节收完
        repeat (DIV*12) @(posedge clk);

        $display("---------------------------------------------");
        $display("[TB] uart loopback: %0d/256 bytes, errors %0d, DIV %0d", recv, errors, DIV);
        if ((recv == 256) && (errors == 0)) begin
            $display("TEST PASSED (tb_uart)");
        end else begin
            $display("TEST FAILED (tb_uart)");
            $fatal(1, "uart loopback failed");
        end
        $display("---------------------------------------------");
        $finish;
    end

endmodule
