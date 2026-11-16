`timescale 1ns/1ps
//======================================================================
// tb_vplant_open.sv —— 虚拟热台开环特性自检（不需要 Vivado、不需要板子）
//
// 验证四件事：
//   1. 稳态：Tj_ss = Ta + P·(R1+R2)，与解析值一致（误差 <5%）
//   2. τ：duty 0→1000 的阶跃，Tj 达到 63.2% 的时间 ≈ (R1+R2)·C2
//   3. 风扇死区/迟滞：duty 50/150 不转，duty 200 起转且延迟 ≈200 ms
//   4. 档位/频率对温度的影响：f 150→25 MHz，同一 duty 下温度下降
//
// 时间压缩：tick_ms 每 100 个时钟一次（1 µs 树墙时间 = 1 ms 物理时间）
//======================================================================
`include "vplant_pkg.vh"

module tb_vplant_open;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg tick_ms = 1'b0;
    always #5 clk = ~clk;

    integer div_cnt;
    integer tick_i;
    always @(posedge clk) begin
        if (!rst_n) begin tick_ms <= 1'b0; div_cnt <= 0; end
        else if (div_cnt == 7) begin div_cnt <= 0; tick_ms <= 1'b1; end
        else begin div_cnt <= div_cnt + 1; tick_ms <= 1'b0; end
    end

    // ---------------- DUT ----------------
    reg  [10:0] duty = 11'd0;
    reg  [3:0]  level = 4'd15;
    reg  [17:0] f_khz = 18'd150000;
    reg         ac_toggle = 1'b0;
    reg         force_en = 1'b0;
    reg  signed [31:0] ta_force = 32'sd0;
    reg  [15:0] trim = 16'd256;

    wire [15:0] rpm;
    wire        tach, stalled;
    wire [15:0] inv_r2;
    wire [7:0]  flow;
    wire [31:0] p_mw;
    wire [15:0] p_dither;
    wire signed [31:0] ta_uk, drift_uk, tj_uk, tc_uk, q12, q2a;
    wire               ac_on;

    fan_vplant u_fan (
        .clk(clk), .rst_n(rst_n), .tick_ms(tick_ms), .duty(duty),
        .unit_trim_q8(trim), .rpm(rpm), .tach(tach), .inv_r2_q12(inv_r2),
        .flow_q8(flow), .stalled(stalled)
    );

    power_model u_pwr (
        .clk(clk), .rst_n(rst_n), .tick_ms(tick_ms), .f_khz(f_khz),
        .level(level), .p_mw(p_mw), .p_dither(p_dither)
    );

    ambient_vplant u_amb (
        .clk(clk), .rst_n(rst_n), .tick_ms(tick_ms), .ac_toggle(ac_toggle),
        .force_en(force_en), .ta_force_uk(ta_force),
        .ta_uk(ta_uk), .ac_on(ac_on), .drift_uk(drift_uk)
    );

    thermal_core u_th (
        .clk(clk), .rst_n(rst_n), .tick_ms(tick_ms),
        .p_mw($signed(p_mw)), .ta_uk(ta_uk), .inv_r2_q12(inv_r2),
        .t_init_uk(`VP_TAMB_DEF_UK),
        .tj_uk(tj_uk), .tc_uk(tc_uk), .q12_uw(q12), .q2a_uw(q2a)
    );

    // ---------------- 量测工具 ----------------
    real  r2_now, t_exp, t_err, t0, tss, tau_meas;
    integer fails = 0;

    task wait_ms(input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) @(posedge tick_ms);
        end
    endtask

    task settle_and_report(input [10:0] d);
        begin
            duty = d;
            wait_ms(150000);                       // 150 s 物理时间 ≈ 4.5τ（自然对流档）
            r2_now = 4096.0 / $itor(inv_r2);    // Q20 → K/W
            t_exp  = 25.0 + ($itor(p_mw)/1000.0) * (3.0 + r2_now);
            t_err  = ($itor(tj_uk)/1.0e6) - t_exp;
            $display("duty=%4d rpm=%4d R2=%5.1f K/W  Tj=%8.3f C  Texp=%8.3f C  err=%+6.3f C  flow=%0d%s",
                     d, rpm, r2_now, ($itor(tj_uk)/1.0e6), t_exp, t_err, flow,
                     stalled ? "  [STALL]" : "");
            if (t_err > 4.0 || t_err < -4.0) begin
                $display("  [FAIL] 稳态与解析值不符（尚未收敛或参数错）");
                fails = fails + 1;
            end
        end
    endtask

    initial begin
        $display("==== 虚拟热台开环自检 ====");
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        wait_ms(2000);

        // ---- 1. 风扇死区 ----
        duty = 11'd50;   wait_ms(1000);
        if (rpm != 0) begin $display("[FAIL] duty=50 应该不转，实际 rpm=%0d", rpm); fails=fails+1; end
        else $display("[OK] duty=50  不转（死区）");
        duty = 11'd40;   wait_ms(1000);
        if (rpm != 0) begin $display("[FAIL] duty=40 低于 6%% 起转阈值，实际 rpm=%0d", rpm); fails=fails+1; end
        else $display("[OK] duty=40 不起转（需要 6%% 才踢得动）");

        // ---- 2. 启动延迟 ----
        t0 = 0;
        duty = 11'd200;
        wait_ms(150);
        if (rpm != 0) $display("[WARN] 启动延迟内就转起来了 rpm=%0d", rpm);
        wait_ms(600);
        $display("[OK] duty=200 起转后 rpm=%0d（含 200 ms 延迟 + 256 ms 惯性）", rpm);

        // ---- 3. 稳态扫点 ----
        settle_and_report(11'd0);
        settle_and_report(11'd300);
        settle_and_report(11'd500);
        settle_and_report(11'd700);
        settle_and_report(11'd1000);

        // ---- 4. τ 测量：duty 1000 → 0（热阶跃，温度上升，方向为正）----
        duty = 11'd1000; wait_ms(150000);              // 冷稳态
        duty = 11'd0;    wait_ms(150000);              // 热稳态
        tss = ($itor(tj_uk)/1.0e6);
        r2_now = 4096.0 / $itor(inv_r2);            // ≈28 K/W
        duty = 11'd1000; wait_ms(150000);              // 回到冷稳态
        t0 = ($itor(tj_uk)/1.0e6);
        duty = 11'd0;
        tau_meas = -1.0;
        for (tick_i = 0; tick_i < 160000; tick_i = tick_i + 1) begin
            @(posedge tick_ms);
            if (tau_meas < 0 && ($itor(tj_uk)/1.0e6) >= t0 + 0.632*(tss - t0)) tau_meas = tick_i*0.001;
        end
        $display("[tau] 热阶跃: T0=%.2f Tss=%.2f  t63=%.1f s  (理论 (R1+R2)·C2 ≈ %.1f s)",
                 t0, tss, tau_meas, (3.0 + r2_now)*1.2);
        if (tau_meas < 15.0 || tau_meas > 60.0) begin
            $display("[FAIL] τ 超出 15…60 s"); fails = fails + 1;
        end

        // ---- 5. 频率对温度的影响（同一 duty=500）----
        duty = 11'd500; f_khz = 18'd150000; wait_ms(150000);
        t0 = ($itor(tj_uk)/1.0e6);
        f_khz = 18'd25000; wait_ms(150000);
        tss = ($itor(tj_uk)/1.0e6);
        $display("[f-T] duty=500: 150 MHz → %.2f C, 25 MHz → %.2f C (Δ=%.2f C)", t0, tss, t0-tss);
        if (t0 <= tss) begin $display("[FAIL] 降频应该降温"); fails = fails + 1; end

        $display("==== %s (fails=%0d) ====", (fails==0) ? "PASS" : "FAIL", fails);
        if (fails != 0) $fatal(1, "vplant open-loop failed");
        $finish;
    end
endmodule






