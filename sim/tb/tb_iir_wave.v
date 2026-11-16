// ============================================================================
//  文件名 : tb_iir_wave.v
//  用途   : 为报告 §8 证据 E-02 生成 IIR 滤波器的阶跃响应波形（不参与功能回归）
//
//  为什么单独写一个而不是复用 tb_top_smoke：
//    系统级 TB 里 IIR 被包在远处，信号又被 100 ms 节拍稀释，波形看不清。
//    这里只驱动 iir_lpf 一个模块，把采样间隔压成 1 µs（样本域行为不变），
//    图上能直接数出 τ≈4 个样本的指数收敛。
//
//  运行：iverilog -g2005 -s tb_iir_wave -o iir.vvp tb_iir_wave.v ../../rtl/dsp/iir_lpf.v
//        vvp iir.vvp
//  产物：iir_wave.vcd（再用 sim/tools/vcd2svg.py 转 SVG）
// ============================================================================
`timescale 1ns / 1ps

module tb_iir_wave;

    localparam integer CLK_HALF = 5;      // 100 MHz
    localparam integer SAMPLE_DIV = 100;  // 每 100 个时钟来一个样本 = 1 µs

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #CLK_HALF clk = ~clk;

    reg               valid = 1'b0;
    reg signed [15:0] din   = 16'sd0;     // Q8.8
    wire signed [15:0] dout;
    wire               dout_valid;

    iir_lpf #(.K(2)) u_iir (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid      (valid),
        .din        (din),
        .dout       (dout),
        .dout_valid (dout_valid)
    );

    integer n;
    integer cnt = 0;
    localparam integer N_SAMPLES = 120;

    // 20.0 ℃ 起步，第 10 个样本起跳到 50.0 ℃（Q8.8 = ×256）
    function signed [15:0] step_val(input integer i);
        begin
            step_val = (i < 10) ? 16'sd5120 : 16'sd12800;
        end
    endfunction

    initial begin
        $dumpfile("iir_wave.vcd");
        $dumpvars(0, tb_iir_wave);

        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        for (n = 0; n < N_SAMPLES; n = n + 1) begin
            // 等一个采样点
            repeat (SAMPLE_DIV) @(posedge clk);
            din   <= step_val(n);
            valid <= 1'b1;
            @(posedge clk);
            valid <= 1'b0;
        end

        repeat (200) @(posedge clk);
        $display("[INFO] tb_iir_wave: %0d samples, final dout=%0d (Q8.8, expect ~12800)",
                 N_SAMPLES, dout);
        $finish;
    end

endmodule
