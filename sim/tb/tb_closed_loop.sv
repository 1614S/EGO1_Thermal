`timescale 1ns/1ps
//======================================================================
// tb_closed_loop.sv —— 赛题指标 4 的自动判分（40/50/60 ℃ 三档）
//
// 被测对象：完整顶层 + 虚拟热台（sw6=1），所以量的是"整条链"，
// 不是某个模块的孤立行为。
//
// 三个指标（赛题原文）：
//   稳态误差 |Tj−Tset| ≤ ±1 ℃（最后 20 s 平均）
//   超调量  (Tmax−Tset)/(Tset−T0) ≤ 10 %
//   调节时间  首次进入 ±1 ℃ 并保持 10 s 的时刻 ≤ 60 s
//
// 整定入口：+KP=<Q8.8> +KI=<Q8.8> 直接改 PI 寄存器（不开命令通道也能调参）
// 运行：iverilog ... -DSIM_ACCEL=10000，然后 vvp -KP=10240 -KI=38
//======================================================================
`include "ego1_thermal_pkg.vh"
`include "tb_common.svh"

module tb_closed_loop;
    reg clk = 1'b0;
    reg rst_btn = 1'b0;
    reg [4:0] btn = 5'b1_1111;
    reg [7:0] sw  = 8'b0100_1100;      // sw6=1 虚拟热台, level=12

    wire [15:0] led;
    wire [7:0]  seg;
    wire [3:0]  an;
    wire        uart_tx, fan_pwm, dht_dq;

    assign (weak1, weak0) dht_dq = 1'b1;
    always #5 clk = ~clk;

    ego1_thermal_top u_dut (
        .clk(clk), .rst_btn(rst_btn), .btn(btn), .sw(sw),
        .led(led), .seg(seg), .an(an), .uart_tx(uart_tx), .uart_rx(1'b1),
        .fan_pwm(fan_pwm), .fan_tach(1'b1), .dht_dq(dht_dq)
    );

    integer kp_arg, ki_arg;
    real    t_start;
    integer fd;                      // 轨迹 CSV（给 analyze_thermal.py 出图用）

    // 单档量测
    task run_phase(input signed [15:0] sp, input integer secs,
                   output real ovs, output real settle, output real ess);
        integer tt, hold;
        real tj, spc, mx, sum, d;
        begin
            u_dut.u_regs.r_setpoint = sp;      // 测试后门：正常由按键/命令通道写
            spc   = $itor(sp) / 256.0;
            tt    = 0; hold = 0; mx = -200.0; settle = -1.0; sum = 0.0;
            t_start = $itor(u_dut.tj_filt) / 256.0;
            repeat (secs * 1000) begin
                @(posedge u_dut.tick_ms);
                tt = tt + 1;
                if ((tt % 10) == 0) begin
                    tj = $itor(u_dut.tj_filt) / 256.0;
                    if (tj > mx) mx = tj;
                    d = (tj > spc) ? (tj - spc) : (spc - tj);
                    if (d <= 1.0) hold = hold + 1; else hold = 0;
                    if (settle < 0 && hold >= 10) settle = tt * 0.001;
                    if (tt > (secs - 20) * 1000) sum = sum + d;
                    if ((tt % 1000) == 0)
                        $fdisplay(fd, "%0.2f,%0.2f,%0.2f,%0d,%0d,%0d",
                                  tt * 0.001, tj, spc, u_dut.duty,
                                  u_dut.gear, u_dut.vp_p_mw);
                    if ((tt % 20000) == 0)
                        $display("    t=%4.0f s  Tj=%6.2f  duty=%4d  rpm=%4d  state=%0d",
                                 tt * 0.001, tj, u_dut.duty, u_dut.vp_rpm, u_dut.state);
                end
            end
            ess = sum / ((20 * 1000) / 10);
            ovs = (mx - spc) / ((spc - t_start) > 0.5 ? (spc - t_start) : 0.5) * 100.0;
            $display("  Tset=%5.1f  T0=%5.2f  Tmax=%6.2f  超调=%5.1f%%  t_settle=%5.1f s  ess=%.3f C",
                     spc, t_start, mx, ovs, settle, ess);
        end
    endtask

    real o1, s1, e1, o2, s2, e2, o3, s3, e3;

    initial begin
        `INIT_TB
        $display("==== 闭环指标判分（虚拟热台，level=12，150 MHz） ====");
        fd = $fopen(".work/closed_loop_trace.csv", "w");
        $fdisplay(fd, "t_s,tj_c,tset_c,duty,gear,p_mw");
        if (!$value$plusargs("KP=%d", kp_arg)) kp_arg = `KP_DEF_Q8_8;
        if (!$value$plusargs("KI=%d", ki_arg)) ki_arg = `KI_DEF_Q8_8;
        $display("  Kp=%.2f duty/℃  Ki=%.4f duty/(℃·样本)", kp_arg/256.0, ki_arg/256.0);

        repeat (20) @(posedge clk);
        rst_btn = 1'b1;
        repeat (10) @(posedge clk);
        u_dut.u_regs.r_gear = 3'd4;            // 150 MHz 档，让 40…60 ℃ 都可控
        u_dut.u_regs.r_kp   = kp_arg[15:0];
        u_dut.u_regs.r_ki   = ki_arg[15:0];
        // 等滤波链出第一个有效样本（MA/IIR 预热后的值），否则 T0 会被算成 0 ℃
        while (u_dut.tj_filt == 16'sd0) @(posedge u_dut.tick_ms);

        run_phase(16'sd10240, 150, o1, s1, e1);   // 40.0 ℃
        run_phase(16'sd12800, 150, o2, s2, e2);   // 50.0 ℃
        run_phase(16'sd15360, 150, o3, s3, e3);   // 60.0 ℃

        `CHK(e1 <= 1.0 && e2 <= 1.0 && e3 <= 1.0, "稳态误差 ≤±1 ℃（三档）")
        `CHK(o1 <= 10.0 && o2 <= 10.0 && o3 <= 10.0, "超调 ≤10 %（三档）")
        `CHK(s1 > 0.0 && s1 <= 60.0, "40 ℃ 调节时间 ≤60 s")
        `CHK(s2 > 0.0 && s2 <= 60.0, "50 ℃ 调节时间 ≤60 s")
        `CHK(s3 > 0.0 && s3 <= 60.0, "60 ℃ 调节时间 ≤60 s")
        $fclose(fd);
        `DONE("tb_closed_loop")
    end
endmodule
