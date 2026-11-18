`timescale 1ns/1ps
//======================================================================
// tb_vplant_sys.sv —— 全系统闭环（虚拟热台模式）
//
// 用一个 TB 回答一个问题：**整条链闭起来能不能稳到设定值**。
//   sw[6]=1 → 虚拟热台接管 XADC/AM2302；sw[3:0]=15 → 满负载
//   时间压缩：-DSIM_ACCEL=10000（1 ms 物理时间 = 10 个时钟）
//
// 判据（赛题指标 4）：稳态误差 ≤±1 ℃、超调 ≤10 %、调节时间 ≤60 s
// 本 TB 先只打印轨迹，指标判分在 tb_closed_loop 里做（下一步）。
//======================================================================
`include "ego1_thermal_pkg.vh"
`include "tb_common.svh"

module tb_vplant_sys;
    reg clk = 1'b0;
    reg rst_btn = 1'b0;
    reg [4:0] btn = 5'b1_1111;
    reg [7:0] sw  = 8'b0100_1111;   // sw6=1 虚拟热台, level=15

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

    integer t_s;
    real tj, sp, tmax;
    initial begin
        `INIT_TB
        $display("==== 虚拟热台闭环（样机） ====");
        repeat (20) @(posedge clk);
        rst_btn = 1'b1;
        tmax = -100.0;
        repeat (5) @(posedge clk);

        for (t_s = 0; t_s <= 180; t_s = t_s + 10) begin
            if (t_s != 0) #1000us;                  // 1000 µs 仿真 = 10 s 物理（ACCEL=10000）
            tj = $itor(u_dut.tj_filt) / 256.0;
            sp = $itor(u_dut.setpoint) / 256.0;
            if (tj > tmax) tmax = tj;
            $display("t=%4d s  Tj=%7.3f C  set=%5.1f  duty=%4d  rpm=%4d  state=%0d  P=%4d mW  Ta=%.2f C",
                     t_s, tj, sp, u_dut.duty, u_dut.vp_rpm, u_dut.state,
                     u_dut.vp_p_mw, $itor(u_dut.vp_ta_uk)/1.0e6);
        end

        $display("---- 超调：Tmax=%.2f C，设定 %.1f C ----", tmax, sp);
        `CHK(tmax <= 52.0, "超调应 ≤2 ℃（相对 50 ℃ 设定）")
        `CHK(tj > 49.0 && tj < 51.0, "最终应稳定在 50±1 ℃")
        `DONE("tb_vplant_sys")
    end
endmodule
