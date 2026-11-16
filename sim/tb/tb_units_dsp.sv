`timescale 1ns/1ps
//======================================================================
// tb_units_dsp.sv —— DSP/PWM 模块级自检（四个独立 top）
//   tb_ma    : 滑动平均（重点是上电预热：第一拍就该等于输入，而不是从 0 爬）
//   tb_iir   : 一阶 IIR（预热 + 每样本收敛 1/4）
//   tb_calib : 两点标定（增益/偏置/饱和）
//   tb_pwm   : PWM 载频 24.414 kHz、12 bit、占空比映射
//======================================================================
`include "tb_common.svh"

//----------------------------------------------------------------------
module tb_ma;
    reg clk = 0, rst_n = 0;
    reg valid = 0;
    reg signed [15:0] din = 0;
    wire signed [15:0] dout;
    wire dv, primed;
    always #5 clk = ~clk;

    moving_avg #(.N(8), .SHIFT(3)) u (.clk(clk), .rst_n(rst_n), .valid(valid), .din(din),
                                     .dout(dout), .dout_valid(dv), .primed(primed));

    task feed(input signed [15:0] d);
        begin din = d; valid = 1'b1; @(posedge clk); valid = 1'b0; @(posedge clk); end
    endtask

    initial begin
        `INIT_TB
        $display("---- tb_ma ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        feed(16'sd1000);
        `CHK_NEAR(dout, 1000.0, 1.0, "预热：第一拍就应等于输入（不是 125）")
        `CHK(primed == 1'b1, "预热标志")

        repeat (4) feed(16'sd1000);
        `CHK_NEAR(dout, 1000.0, 1.0, "稳态保持")

        feed(16'sd2000);                       // 7×1000 + 2000 = 9000/8 = 1125
        `CHK_NEAR(dout, 1125.0, 2.0, "单点阶跃后应为 1125")

        repeat (8) feed(16'sd2000);
        `CHK_NEAR(dout, 2000.0, 2.0, "窗口滑满后应为 2000")
        `DONE("tb_ma")
    end
endmodule

//----------------------------------------------------------------------
module tb_iir;
    reg clk = 0, rst_n = 0, valid = 0;
    reg signed [15:0] din = 0;
    wire signed [15:0] dout;
    wire dv, primed;
    always #5 clk = ~clk;

    iir_lpf #(.K(2)) u (.clk(clk), .rst_n(rst_n), .valid(valid), .din(din),
                        .dout(dout), .dout_valid(dv), .primed(primed));
    task feed(input signed [15:0] d);
        begin din = d; valid = 1'b1; @(posedge clk); valid = 1'b0; @(posedge clk); end
    endtask

    initial begin
        `INIT_TB
        $display("---- tb_iir ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        feed(16'sd1000);
        `CHK_NEAR(dout, 1000.0, 1.0, "预热：第一拍等于输入")

        feed(16'sd2000);                       // 内部 y 更新，dout 打一拍
        feed(16'sd2000);                       // 这一拍 dout = 1000 + (2000-1000)/4 = 1250
        `CHK_NEAR(dout, 1250.0, 2.0, "一个样本后应为 1250（dout 有一拍延迟）")

        repeat (20) feed(16'sd2000);
        `CHK_NEAR(dout, 2000.0, 8.0, "约 5τ 后应逼近 2000")
        `DONE("tb_iir")
    end
endmodule

//----------------------------------------------------------------------
module tb_calib;
    reg clk = 0, rst_n = 0, valid = 0;
    reg signed [15:0] din = 0;
    reg [15:0] gain = 16'h1000;
    reg signed [15:0] off = 16'sd0;
    wire signed [15:0] dout;
    wire dv;
    always #5 clk = ~clk;

    calib u (.clk(clk), .rst_n(rst_n), .valid(valid), .din(din),
             .gain_q4_12(gain), .offset_q8_8(off), .dout(dout), .dout_valid(dv));
    task apply(input signed [15:0] d, input [15:0] g, input signed [15:0] o);
        begin din = d; gain = g; off = o; valid = 1'b1; @(posedge clk); valid = 1'b0; @(posedge clk); end
    endtask

    initial begin
        `INIT_TB
        $display("---- tb_calib ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        apply(16'sd12800, 16'h1000, 16'sd0);      // 50 ℃ × 1.0 + 0
        `CHK_NEAR(dout, 12800.0, 2.0, "增益1.0 应直通")

        apply(16'sd12800, 16'h0800, 16'sd0);      // ×0.5
        `CHK_NEAR(dout, 6400.0, 4.0, "增益0.5")

        apply(16'sd12800, 16'h1000, -16'sd2560);  // b=-10℃ → y=(x−b)=x+10℃
        `CHK_NEAR(dout, 15360.0, 4.0, "偏置 b=-10 ℃ 应等价于 +10 ℃")

        apply(16'sd32000, 16'h2000, 16'sd0);      // ×2 → 饱和
        `CHK_NEAR(dout, 32000.0, 2.0, "上限饱和")
        `DONE("tb_calib")
    end
endmodule

//----------------------------------------------------------------------
module tb_pwm;
    reg clk = 0, rst_n = 0;
    reg [10:0] duty = 0;
    wire pwm;
    always #5 clk = ~clk;

    pwm_gen #(.BITS(12)) u (.clk(clk), .rst_n(rst_n), .duty(duty), .pwm(pwm));

    integer hi = 0, per = 0, last_per = 0, nrise = 0;
    reg pwm_d = 0;
    always @(posedge clk) begin
        if (pwm_d) hi = hi + 1;
        per = per + 1;
        pwm_d = pwm;
        if (pwm && !pwm_d) begin                 // 占位，实际用下一拍判断
        end
    end
    // 用下降沿统计一个周期内的占空比更稳
    integer hi_n = 0, lo_n = 0;
    reg pwm_q = 0;
    always @(posedge clk) begin
        pwm_q = pwm;
        if (pwm) hi_n = hi_n + 1; else lo_n = lo_n + 1;
    end

    task measure(input [10:0] d);
        begin
            duty = d;
            repeat (8200) @(posedge clk);        // 跨过两个周期
            hi_n = 0; lo_n = 0;
            repeat (4096) @(posedge clk);        // 整一个周期
            $display("  duty=%4d → 高 %4d / 4096 拍 (%.2f%%)", d, hi_n, hi_n*100.0/4096.0);
        end
    endtask

    initial begin
        `INIT_TB
        $display("---- tb_pwm ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        measure(11'd0);
        `CHK(hi_n <= 2, "duty=0 应该几乎没有高电平")
        measure(11'd500);
        `CHK_NEAR(hi_n, 2048.0, 8.0, "duty=500 应为 2048/4096")
        measure(11'd1000);
        `CHK_NEAR(hi_n, 4095.0, 3.0, "duty=1000 应接近满占空")
        $display("  载频 = 100 MHz / 4096 = %.3f kHz", 100000.0/4096.0);
        `DONE("tb_pwm")
    end
endmodule
