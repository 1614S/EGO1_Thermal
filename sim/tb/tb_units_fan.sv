`timescale 1ns/1ps
//======================================================================
// tb_units_fan.sv —— 风扇测速与堵转判定
//   1. 140 Hz tach（= 4200 rpm）→ 读数应 ≈4200
//   2. 有占空比但没脉冲 → 1 s 后 fan_fail 置位
//   3. 脉冲恢复 → fan_fail 清除
//   4. 占空比 0 且无脉冲 → 不算故障（主动停机）
//======================================================================
`include "tb_common.svh"

module tb_fan;
    reg clk = 0, rst_n = 0, tick = 0;
    reg tach = 0;
    reg [10:0] duty = 11'd500;
    wire [15:0] rpm, fail_cnt;
    wire        fan_fail;

    always #5 clk = ~clk;
    integer k = 0;
    always @(posedge clk) begin
        if (k == 99) begin k = 0; tick = 1; end else begin k = k + 1; tick = 0; end   // tick = 1 µs, 闸门 1000 拍 = 1 s
    end

    fan_tach #(.GATE_MS(1000), .DUTY_MIN(11'd200)) u (
        .clk(clk), .rst_n(rst_n), .tick_ms(tick), .tach(tach),
        .duty(duty), .rpm(rpm), .fan_fail(fan_fail), .fail_cnt(fail_cnt)
    );

    // 4200 rpm = 70 转/s × 2 脉冲 = 140 脉冲/s → 半周期 100000/280 = 357 拍
    task tach_run(input integer n_pulse);
        integer i;
        begin
            for (i = 0; i < n_pulse * 2; i = i + 1) begin
                tach = ~tach;
                repeat (357) @(posedge clk);   // 半周期 357 拍 → 140 脉冲/s → 4200 rpm
            end
        end
    endtask

    task wait_ms(input integer n);
        integer i;
        begin for (i = 0; i < n; i = i + 1) @(posedge tick); end
    endtask

    initial begin
        `INIT_TB
        $display("---- tb_fan ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        // ---- 1. 有脉冲：读数 ≈ 4200 ----
        tach_run(150);                       // 约 1.07 s 的脉冲
        `CHK_NEAR(rpm, 4200.0, 60.0, "140 Hz tach 应读≈4200 rpm")
        `CHK(fan_fail == 1'b0, "有脉冲不应报故障")

        // ---- 2. 停脉冲但仍给占空比 → 堵转 ----
        duty = 11'd800;
        wait_ms(2500);   // 跨过完整的 1 s 闸门
        `CHK(fan_fail == 1'b1, "有占空比但 1 s 无脉冲应报堵转")
        `CHK_NEAR(rpm, 0.0, 1.0, "无脉冲时 rpm 应为 0")

        // ---- 3. 脉冲恢复 → 故障清除 ----
        tach_run(150);
        `CHK(fan_fail == 1'b0, "脉冲恢复后故障应清除")

        // ---- 4. 主动停机不算故障 ----
        duty = 11'd0;
        wait_ms(2500);   // 跨过完整的 1 s 闸门
        `CHK(fan_fail == 1'b0, "占空比 0 且无脉冲不应报故障")
        $display("  最终 rpm=%0d 堵转累计=%0d", rpm, fail_cnt);
        `DONE("tb_fan")
    end
endmodule



