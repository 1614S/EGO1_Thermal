`timescale 1ns/1ps
//======================================================================
// fan_tach.v —— 4 线风扇测速与堵转判定
//
// 4 线风扇约定：每转 2 个 tach 脉冲（开漏，需上拉）。
//   转速 rpm = 每秒边沿数 × 15（每个脉冲帧两个边沿，每转 2 个脉冲）
//
// 测量方法：1 s 闸门数脉冲。选 1 s 而不是测周期，是因为低速时
// 周期很长（300 rpm 时两个脉冲间隔 100 ms），测周期需要的计时器更大，
// 而 1 s 闸门的量化误差在 300 rpm 时是 30 rpm（10%），够判堵转。
//
// 堵转判定（去抖 1 s）：
//   占空比 > 20% 且这一秒一个脉冲都没有 → fan_fail 置位（迟滞：转速恢复即清）
//   占空比 = 0（主动停风扇）不算故障
//======================================================================
module fan_tach #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer GATE_MS = 1000,      // 测速闸门
    parameter [10:0]  DUTY_MIN = 11'd200   // 低于此占空比不判堵转
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_ms,
    input  wire        tach,               // 异步输入
    input  wire [10:0] duty,               // 当前占空比（判断"应该有转速"）
    output reg  [15:0] rpm,
    output reg         fan_fail,
    output reg  [15:0] fail_cnt            // 累计堵转次数（监测）
);
    //------------------------------------------------------------------
    // 两级同步 + 边沿检测
    //------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg t1, t2, t3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin t1 <= 1'b0; t2 <= 1'b0; t3 <= 1'b0; end
        else begin
            t1 <= tach;
            t2 <= t1;
            t3 <= t2;
        end
    end
    wire tach_edge = t2 ^ t3;

    //------------------------------------------------------------------
    // 1 s 闸门计数
    //------------------------------------------------------------------
    reg [9:0]  cnt;
    reg [9:0]  gate_ms;
    reg [15:0] pulse_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt       <= 10'd0;
            gate_ms   <= 10'd0;
            pulse_cnt <= 16'd0;
            rpm       <= 16'd0;
            fan_fail  <= 1'b0;
            fail_cnt  <= 16'd0;
        end else begin
            if (tach_edge && pulse_cnt != 16'hFFFF) pulse_cnt <= pulse_cnt + 16'd1;

            if (tick_ms) begin
                if (gate_ms >= GATE_MS - 1) begin
                    gate_ms <= 10'd0;
                    // 数的是边沿不是脉冲：每个脉冲 2 个边沿、每转 2 个脉冲 → rpm = 边沿数 × 15
                    rpm <= (pulse_cnt > 16'd4368) ? 16'hFFFF : (pulse_cnt * 16'd15);
                    if (pulse_cnt == 16'd0 && duty >= DUTY_MIN) begin
                        if (!fan_fail) fail_cnt <= fail_cnt + 16'd1;
                        fan_fail <= 1'b1;
                    end else if (pulse_cnt != 16'd0) begin
                        fan_fail <= 1'b0;
                    end
                    pulse_cnt <= 16'd0;
                end else begin
                    gate_ms <= gate_ms + 10'd1;
                end
            end
        end
    end

endmodule


