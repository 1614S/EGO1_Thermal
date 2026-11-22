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
    wire        bang;      // 1 = 加热环处于饱和区（0/100 %）

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
        .bang     (bang),
        .saturated(sat)
    );

    always #5 clk = ~clk;                   // 100 MHz

    // ---------------- 一阶热模型 ----------------
    real t_amb   = 25.0;                    // ℃
    real p_max   = 2.5;                     // W（5 V / 10 Ω）
    // 2026-11-22 实测标定（results/heat_no_mos.csv，5 V 常通 2.5 W）：
    //   AM2302 23.0 → 48.2 ℃（10 min），指数外推稳态 89.9 ℃ → θ_eff = 26.7 K/W
    //   热容取 4 J/K，使 τ = 107 s（真实装置 τ≈1150 s，仿真里等比压缩）
    real rth     = 26.7;                    // K/W（实测）
    real cth     = 4.0;                     // J/K → τ = 107 s
    real dt      = 0.1;                     // 每个 update 推进 0.1 s

    reg seen_band = 1'b0;
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
    integer max_duty;
    integer min_duty;
    integer n_bang;
    integer n_relapse;      // 进入 PI 区后又掉回饱和区的次数（应为 0）
    real y0, ymax, yss, os;
    task step(input real sp_c, input integer n_upd);
        begin
            seen_band = 1'b0;
            sp = $rtoi(sp_c * 256.0);
            // 记录起点
            y0 = t_amb;
            ymax = t_amb;
            max_duty = 0;
            min_duty = 1000;
            n_bang   = 0;
            n_relapse = 0;
            for (i = 0; i < n_upd; i = i + 1) begin
                @(posedge clk);
                while (update !== 1'b1) @(posedge clk);
                if (t_amb > ymax) ymax = t_amb;
                if (duty > max_duty) max_duty = duty;
                if (duty < min_duty) min_duty = duty;
                if (bang) n_bang = n_bang + 1;
                // 误差首次进入 ±3 K 之后，若又回到饱和区 → 迟滞失效（真正要防的现象）
                if (!seen_band && (sp_c - t_amb) <= 3.0 && (t_amb - sp_c) <= 3.0)
                    seen_band = 1'b1;
                if (seen_band && bang) n_relapse = n_relapse + 1;
            end
            yss = t_amb;
            os = (ymax - sp_c) / (sp_c - y0) * 100.0;
            if (os < 0) os = 0;
            $display("  Tset=%5.1f C  T0=%6.2f  Tmax=%6.2f  Tss=%6.2f  overshoot=%5.1f%%  err=%+5.2f C  duty=%0d",
                     sp_c, y0, ymax, yss, os, yss - sp_c, duty);
            $display("            饱和区样本=%0d/%0d  duty 范围=%0d~%0d  回饱和次数=%0d",
                     n_bang, n_upd, min_duty, max_duty, n_relapse);
            if (n_relapse > 5) begin
                fails = fails + 1;
                $display("  [FAIL] 进入 PI 区后又反复回到饱和区 %0d 次（迟滞阈值写反？）", n_relapse);
            end
            // 判据：稳态误差 ≤±0.5 C、超调 ≤10 %
            if ((yss - sp_c) > 0.5 || (yss - sp_c) < -0.5) begin
                fails = fails + 1;
                $display("  [FAIL] 稳态误差 %.2f C 超差（要求 ±0.5）", yss - sp_c);
            end
            if (os > 10.0) begin
                fails = fails + 1;
                $display("  [FAIL] 超调 %.1f %% 超差（要求 ≤10）", os);
            end
            // 阶跃幅度 >4 K 时，必须先进饱和区（0/100 %），逼近后才交给 PI
            if ((sp_c - y0) < -4.0) begin
                if (n_bang == 0) begin
                    fails = fails + 1;
                    $display("  [FAIL] 反向大误差没进饱和区（应输出 0 %% 降温）");
                end
                if (min_duty > 0) begin
                    fails = fails + 1;
                    $display("  [FAIL] 反向大误差时占空比没降到 0（最小 %0d）", min_duty);
                end
            end
            if ((sp_c - y0) > 4.0) begin
                if (n_bang == 0) begin
                    fails = fails + 1;
                    $display("  [FAIL] 大误差阶段没有进入饱和区（应输出 100%%）");
                end
                if (max_duty < 1000) begin
                    fails = fails + 1;
                    $display("  [FAIL] 大误差阶段占空比没到 1000（实测 %0d）", max_duty);
                end
            end
        end
    endtask

    initial begin
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(20) @(posedge clk);
        $display("=== 环境温度闭环（前馈 + PI + 积分分离；对象 τ=107 s, θ=26.7 K/W 实测标定, Pmax=2.5 W）===");
        // 每档给 600 s ≈ 5.6τ，保证真的收敛到稳态再判误差
        step(40.0, 6000);
        step(50.0, 6000);
        step(60.0, 6000);
        step(42.0, 6000);      // 反向大阶跃：应进饱和区输出 0 %，降温后再交给 PI
        if (fails == 0) $display("==== tb_ext_loop : PASS (fails=0) ====");
        else            $display("==== tb_ext_loop : FAIL (fails=%0d) ====", fails);
        $display("=== 完成 ===");
        $finish;
    end

endmodule
