`timescale 1ns/1ps
//======================================================================
// tb_units_clk.sv —— 时基与测频的模块级自检（两个独立 top）
//   tb_tick : tick_gen 的 1 ms / 1 µs 节拍周期是否精确（所有时间指标的地基）
//   tb_fmon : freq_monitor 在 25/100/150 MHz 下的读数是否准
//======================================================================
`include "tb_common.svh"

//----------------------------------------------------------------------
// tb_tick
//----------------------------------------------------------------------
module tb_tick;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    wire t_ms_a, t_us_a, t_ms_b;
    tick_gen #(.CLK_HZ(100_000_000), .ACCEL(1))    u_a (.clk(clk), .rst_n(rst_n), .tick_ms(t_ms_a), .tick_us(t_us_a));
    tick_gen #(.CLK_HZ(100_000_000), .ACCEL(1000)) u_b (.clk(clk), .rst_n(rst_n), .tick_ms(t_ms_b), .tick_us());

    integer cyc = 0, pa = -1, pb = -1, pu = -1;
    integer per_a = -1, per_b = -1, per_u = -1;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (t_ms_a) begin if (pa >= 0 && per_a < 0) per_a = cyc - pa; pa = cyc; end
        if (t_ms_b) begin if (pb >= 0 && per_b < 0) per_b = cyc - pb; pb = cyc; end
        if (t_us_a) begin if (pu >= 0 && per_u < 0) per_u = cyc - pu; pu = cyc; end
    end

    initial begin
        `INIT_TB
        $display("---- tb_tick ----");
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (420000) @(posedge clk);   // 让两种 ACCEL 都至少出 2 个节拍
        `CHK(per_a == 100000, "tick_ms @ACCEL=1 应为 100000 拍")
        `CHK(per_b == 100,    "tick_ms @ACCEL=1000 应为 100 拍")
        `CHK(per_u   == 100,    "tick_us 应为 100 拍（1 µs @100 MHz）")
        $display("  tick_ms(ACCEL=1)=%0d  tick_ms(ACCEL=1000)=%0d  tick_us=%0d 拍", per_a, per_b, per_u);
        `DONE("tb_tick")
    end
endmodule

//----------------------------------------------------------------------
// tb_fmon：三路已知频率同时测
//----------------------------------------------------------------------
module tb_fmon;
    reg clk = 1'b0;                 // 200 MHz 参考
    reg rst_n = 1'b0;
    reg c25 = 1'b0, c100 = 1'b0, c150 = 1'b0;

    always #2.5   clk  = ~clk;
    always #20.0  c25  = ~c25;      // 25 MHz
    always #5.0   c100 = ~c100;     // 100 MHz
    always #3.333 c150 = ~c150;     // 150 MHz

    wire [17:0] f25, f100, f150;
    wire v25, v100, v150;

    freq_monitor #(.CLK_HZ(200_000_000)) u25  (.clk(clk), .rst_n(rst_n), .clk_test(c25),  .f_khz(f25),  .valid(v25));
    freq_monitor #(.CLK_HZ(200_000_000)) u100 (.clk(clk), .rst_n(rst_n), .clk_test(c100), .f_khz(f100), .valid(v100));
    freq_monitor #(.CLK_HZ(200_000_000)) u150 (.clk(clk), .rst_n(rst_n), .clk_test(c150), .f_khz(f150), .valid(v150));

    initial begin
        `INIT_TB
        $display("---- tb_fmon ----");
        #50 rst_n = 1'b1;                 // 两个时钟域都先吃到 rst_n=0 的边沿
        repeat (2000000) @(posedge clk);   // 10 ms，够 10 次闸门
        $display("  25 MHz → %0d kHz | 100 MHz → %0d kHz | 150 MHz → %0d kHz", f25, f100, f150);
        `CHK_NEAR(f25,  25000.0, 200.0, "25 MHz 测频")
        `CHK_NEAR(f100, 100000.0, 200.0, "100 MHz 测频")
        `CHK_NEAR(f150, 150000.0, 200.0, "150 MHz 测频")
        `DONE("tb_fmon")
    end
endmodule


