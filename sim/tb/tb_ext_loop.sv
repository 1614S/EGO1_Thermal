`timescale 1ns/1ps
//======================================================================
// tb_ext_loop.sv —— 环境温度（AM2302）闭环仿真：前馈 + PI 控加热电阻
//
// 被控对象（一阶热模型，按 5 V / 10 Ω / 5 W 加热电阻 + 装置 θ≈100 K/W）：
//   P_heat = duty/1000 × V²/R = duty/1000 × 2.5 W
//   C dT/dt = P_heat − (T − Tamb)/Rth
//   Rth = 100 K/W、C = 1 J/K → τ = 100 s（与实物的几十秒量级一致）
//
// 时间压缩：TB 每 100 个时钟算一次 update（等价硬件 100 ms），
//           每个 update 推进被控对象 0.1 s 的物理时间 → 与硬件动态一致。
//
// 判据：40 / 50 / 60 ℃ 三档稳定误差 ≤ ±0.5 ℃，超调 ≤ 10 %
//======================================================================
module tb_ext_loop;

    reg clk = 0, rst_n = 0;
    reg  update = 0;
    reg signed [15:0] sp = 16'sd10240;      // 40.0 ℃
    wire [10:0] duty;
    wire        sat;

    // ---------------- DUT ----------------
    reg  signed [15:0] t_amb_q;             // 环境温度（Q8.8），由被控对象给出
    ext_loop u_dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .update   (update),
        .t_ext    (t_amb_q),
        .t_room   (16'sd6400),    // 室温参考 25.0 C
        .sp_ext   (sp),
        .kp_q8_8  (16'd512),      // Kp = 2 duty/C
        .ki_q8_8  (16'd3),        // Ki = 0.01 duty/(C*sample)
        .heater_en(1'b1),
        .duty     (duty),
        .saturated(sat)
    );

    always #5 clk = ~clk;                   // 100 MHz

    // ---------------- 一阶热模型 ----------------
    real t_amb   = 25.0;                    // ℃
    real p_max   = 2.5;                     // W（5 V / 10 Ω）
    real rth     = 100.0;                   // K/W
    real cth     = 1.0;                     // J/K
    real dt      = 0.1;                     // 每个 update 推进 0.1 s

    integer cycle = 0;
    always @(posedge clk) begin
        cycle <= cycle + 1;
        if (cycle % 100 == 99) begin
            update <= 1'b1;
            // 用**上一个** duty 推进被控对象（模拟控制周期）
            t_amb = t_amb + ((duty / 1000.0) * p_max - (t_amb - 25.0) / rth) * dt / cth;
            t_amb_q <= $rtoi(t_amb * 256.0);
        end else begin
            update <= 1'b0;
        end
    end

    // ---------------- 测试序列 ----------------
    integer i, k;
    integer fails = 0;
    real y0, ymax, yss, os;
    task step(input real sp_c, input integer n_upd);
        begin
            sp = $rtoi(sp_c * 256.0);
            // 记录起点
            y0 = t_amb;
            ymax = t_amb;
            for (i = 0; i < n_upd; i = i + 1) begin
                @(posedge clk);
                while (update !== 1'b1) @(posedge clk);
                if (t_amb > ymax) ymax = t_amb;
            end
            yss = t_amb;
            os = (ymax - sp_c) / (sp_c - y0) * 100.0;
            if (os < 0) os = 0;
            $display("  Tset=%5.1f C  T0=%6.2f  Tmax=%6.2f  Tss=%6.2f  overshoot=%5.1f%%  err=%+5.2f C  duty=%0d",
                     sp_c, y0, ymax, yss, os, yss - sp_c, duty);
            // 判据：稳态误差 ≤±0.5 C、超调 ≤10 %
            if ((yss - sp_c) > 0.5 || (yss - sp_c) < -0.5) begin
                fails = fails + 1;
                $display("  [FAIL] 稳态误差 %.2f C 超差（要求 ±0.5）", yss - sp_c);
            end
            if (os > 10.0) begin
                fails = fails + 1;
                $display("  [FAIL] 超调 %.1f %% 超差（要求 ≤10）", os);
            end
        end
    endtask

    initial begin
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(20) @(posedge clk);
        $display("=== 环境温度闭环（前馈 + PI + 积分分离，对象 τ=100 s, θ=100 K/W, Pmax=2.5 W）===");
        step(40.0, 3000);      // 300 s 物理时间
        step(50.0, 3000);
        step(60.0, 3000);
        if (fails == 0) $display("==== tb_ext_loop : PASS (fails=0) ====");
        else            $display("==== tb_ext_loop : FAIL (fails=%0d) ====", fails);
        $display("=== 完成 ===");
        $finish;
    end

endmodule
