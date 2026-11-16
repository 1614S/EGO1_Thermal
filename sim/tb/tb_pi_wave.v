// ============================================================================
//  文件名 : tb_pi_wave.v
//  用途   : 为报告 §8 证据 E-03 生成 PI 闭环波形（不参与功能回归）
//
//  为什么单独写：
//    系统级 TB（tb_top_smoke）里 PI 的输入来自采集链，波形被帧格式稀释，
//    看不出"误差 -> 占空比 -> 温度回落"这条因果链。
//    这里用一阶热对象把 pi_ctrl 包成闭环，只保留三个信号：
//        tj_filt（结温） / setpoint（设定值） / duty（占空比）
//    图上能直接看到超调、调节时间和稳态误差。
//
//  被控对象（刻意简单，只为跑出可看的波形）：
//       T_eq(duty) = 25 + 60 * (1 - duty/1000)      // 占空比越大越凉
//       T[n+1]     = T[n] + (T_eq - T[n]) / K_PLANT // 一阶惯性
//    这是"演示用"对象，不是真实热模型；真实参数要上板辨识。
//
//  运行：iverilog -g2005 -s tb_pi_wave -o pi.vvp tb_pi_wave.v ../../rtl/ctrl/pi_ctrl.v
//        vvp pi.vvp
// ============================================================================
`timescale 1ns / 1ps

module tb_pi_wave;

    localparam integer CLK_HALF   = 5;      // 100 MHz
    localparam integer SAMPLE_DIV = 200;    // 每 200 拍一个控制周期 = 500 kHz（仿真加速）
    localparam integer N_SAMPLE   = 4000;   // 共 4000 个控制周期
    localparam integer K_PLANT    = 200;    // 对象时间常数（样本数）
    localparam integer Q8_8       = 256;

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #CLK_HALF clk = ~clk;

    reg               update   = 1'b0;
    reg               enable   = 1'b1;
    reg signed [15:0] tj_filt  = 16'sd0;
    reg signed [15:0] setpoint = 16'sd12800;   // 50.00 ℃
    reg        [15:0] kp_q8_8  = 16'd10240;    // 40.00（与 tb_ctrl 一致）
    reg        [15:0] ki_q8_8  = 16'd38;       // 0.148

    wire [10:0] duty;
    wire        saturated;

    pi_ctrl u_pi (
        .clk       (clk),
        .rst_n     (rst_n),
        .update    (update),
        .enable    (enable),
        .tj_filt   (tj_filt),
        .setpoint  (setpoint),
        .kp_q8_8   (kp_q8_8),
        .ki_q8_8   (ki_q8_8),
        .duty      (duty),
        .saturated (saturated)
    );

    // ---------------- 一阶热对象 ----------------
    integer t_i;                 // Q8.8
    integer t_eq;
    integer n;

    task plant_step(input [10:0] d);
        begin
            t_eq = (25 * Q8_8) + (60 * Q8_8 * (1000 - d)) / 1000;
            t_i  = t_i + (t_eq - t_i) / K_PLANT;
        end
    endtask

    initial begin
        $dumpfile("pi_wave.vcd");
        $dumpvars(0, tb_pi_wave);

        t_i  = 40 * Q8_8;        // 起始 40.00 ℃
        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        // 前 500 个周期保持开环，让对象先稳一下（同时作为"阶跃前"基线）
        for (n = 0; n < N_SAMPLE; n = n + 1) begin
            repeat (SAMPLE_DIV) @(posedge clk);

            tj_filt <= t_i[15:0];
            update  <= 1'b1;
            @(posedge clk);
            update  <= 1'b0;

            plant_step(duty);    // 用上一拍 PI 的输出驱动对象
        end

        repeat (200) @(posedge clk);
        $display("[INFO] tb_pi_wave: final T=%0d.%02d degC  duty=%0d  set=%0d.%02d",
                 t_i / Q8_8, (t_i % Q8_8) * 100 / Q8_8,
                 duty, setpoint / Q8_8, (setpoint % Q8_8) * 100 / Q8_8);
        $finish;
    end

endmodule
