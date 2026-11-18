`timescale 1ns/1ps
//======================================================================
// tb_units_ctrl.sv —— 控制层模块级自检
//   tb_sup  : 四级告警（NORMAL/WARN/HOT/CRIT）+ 迟滞 + 去抖 + 锁存
//   tb_gear : 自动 DVFS 外环（降档/升档/驻留/保护优先）
//======================================================================
`include "tb_common.svh"

module tb_sup;
    reg clk = 0, rst_n = 0;
    reg eval = 0;
    reg signed [15:0] tj = 16'sd0;
    reg clr = 0;
    wire [1:0] state;
    wire fan_max, gear_low, load_en, alarm;

    always #5 clk = ~clk;

    supervisor u (
        .clk(clk), .rst_n(rst_n), .eval(eval), .tj(tj),
        .th_warn_q8_8(16'd16128), .th_hot_q8_8(16'd17664), .th_crit_q8_8(16'd19456),
        .clr_alarm(clr), .state(state), .fan_max(fan_max),
        .gear_low(gear_low), .load_en(load_en), .alarm(alarm)
    );

    task ev(input signed [15:0] t, input integer n);
        integer i;
        begin
            tj = t;
            for (i = 0; i < n; i = i + 1) begin
                @(posedge clk); eval = 1'b1; @(posedge clk); eval = 1'b0;
            end
        end
    endtask

    initial begin
        `INIT_TB
        $display("---- tb_sup ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        `CHK(state == 2'd0 && load_en == 1'b1, "复位后应 NORMAL 且负载使能")
        `CHK(fan_max == 1'b0 && gear_low == 1'b0, "复位后不强制风扇/降频")

        ev(16'sd14848, 20);
        `CHK(state == 2'd0, "58 ℃ 应保持 NORMAL")

        ev(16'sd16400, 3);
        `CHK(state == 2'd0, "64 ℃ 前 3 拍仍在 NORMAL（去抖）")
        ev(16'sd16400, 2);
        `CHK(state == 2'd1, "64 ℃ 第 4 拍应进 WARN")
        `CHK(fan_max == 1'b0, "WARN 不强制风扇")

        ev(16'sd18000, 6);
        `CHK(state == 2'd2, "70 ℃ 应进 HOT")
        `CHK(fan_max == 1'b1, "HOT 必须强制风扇 100%")
        `CHK(gear_low == 1'b1, "HOT 必须降频（赛题过温降频）")

        ev(16'sd19800, 6);
        `CHK(state == 2'd3, "77 ℃ 应进 CRIT")
        `CHK(load_en == 1'b0, "CRIT 必须断开自热负载")
        `CHK(alarm == 1'b1, "CRIT 必须置报警")

        ev(16'sd15360, 60);   // 60 ℃：低于 WARN，解除后应稳定在 NORMAL
        `CHK(state == 2'd3, "未清报警时 CRIT 应锁存")
        clr = 1'b1;
        ev(16'sd15360, 60);
        clr = 1'b0;
        `CHK(state == 2'd0, "人工复位 + 回落后应回 NORMAL")
        `CHK(alarm == 1'b0, "报警应被清除")
        `DONE("tb_sup")
    end
endmodule

//----------------------------------------------------------------------
module tb_gear;
    reg clk = 0, rst_n = 0, tick = 0;
    reg auto_en = 1, prot = 0;
    reg [10:0] duty = 11'd0;
    reg signed [15:0] tj = 16'sd12800, sp = 16'sd12800;
    reg [2:0] gear_man = 3'd2;
    wire [2:0] gear, gear_auto;
    wire [15:0] n_down, n_up;

    always #5 clk = ~clk;
    integer k = 0;
    always @(posedge clk) begin
        if (k == 9) begin k = 0; tick = 1; end else begin k = k + 1; tick = 0; end
    end

    gear_ctrl u (
        .clk(clk), .rst_n(rst_n), .tick_ms(tick), .auto_en(auto_en), .prot(prot),
        .duty(duty), .tj(tj), .setpoint(sp), .gear_man(gear_man),
        .gear(gear), .gear_auto(gear_auto), .n_down(n_down), .n_up(n_up)
    );

    task wait_ms(input integer n);
        integer i;
        begin for (i = 0; i < n; i = i + 1) @(posedge tick); end
    endtask

    initial begin
        `INIT_TB
        $display("---- tb_gear ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        `CHK(gear_auto == 3'd3, "复位默认 100 MHz（档 3）")
        auto_en = 0;
        #1;
        `CHK(gear == 3'd2, "关闭自动时应跟随手动档位")

        auto_en = 1;
        duty = 11'd1000; tj = sp + 16'sd512;    // 超设定 2 ℃，风扇满
        wait_ms(11000);                          // 驻留 5 s + 判据 5 s
        `CHK(gear_auto == 3'd2, "高温+风扇饱和持续 5 s 应降一档")
        wait_ms(11000);
        `CHK(gear_auto < 3'd2, "继续高温应继续降档")

        prot = 1;
        #1 `CHK(gear == 3'd0, "保护动作时档位强制 0")
        prot = 0;
        #1

        duty = 11'd0; tj = sp - 16'sd512;
        wait_ms(16000);
        `CHK(n_up > 16'd0, "低温+风扇空闲 10 s 应升档")
        $display("  降档 %0d 次，升档 %0d 次，最终档 %0d", n_down, n_up, gear_auto);
        `DONE("tb_gear")
    end
endmodule

