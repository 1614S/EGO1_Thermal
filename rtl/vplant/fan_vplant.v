`timescale 1ns/1ps
//======================================================================
// fan_vplant.v —— 虚拟风扇（带真实非理想性，不是"理想线性"）
//
// 建模要点（每一项都对应真实 4 线风扇的一个毛病）：
//   1. 死区/堵转：占空比 <6% 完全不转；6–2.5% 之间要靠"已经转着"才能维持
//      —— 迟滞。真实 4 线风扇手册多写 "PWM 5%~100%"。
//   2. 调速非线性：占空比→转速是 16 点表，低段斜率小（S 曲线）。
//   3. 启动延迟：从 0 起转要先等 200 ms。
//   4. 一阶惯性：转速建立 τ≈256 ms。
//   5. 转速抖动：±32 rpm 随机抖动。
//   6. 个体差异：unit_trim 参数（±8%）。
//   7. 测速输出：tach = 2 脉冲/转，频率 = rpm/30 Hz。
//
// 冷却效果（关键改动）：
//   不再用"rpm → R2 查表"（低转速段一档就是 7 K/W ≈ 8 ℃，PI 压不住 ±1 ℃），
//   改成 **1/R2 对流量线性**：h ∝ v（强制对流的常用工程近似），
//     1/R2 = 1/R_max + (1/R_min − 1/R_max)·flow/256
//   R: 34 K/W（无风，裸板自然对流）→ 8 K/W（满速强冷）。连续、单调、可压。
//======================================================================
`include "vplant_pkg.vh"

module fan_vplant (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_ms,
    input  wire [10:0] duty,            // 0…1000（0.1%）
    input  wire [15:0] unit_trim_q8,    // 个体差异，256 = 1.00 倍
    output reg  [15:0] rpm,
    output reg         tach,
    output wire [15:0] inv_r2_q12,      // 1/R2（Q12），连续量
    output wire [7:0]  flow_q8,         // 0…255，冷却流量
    output wire        stalled
);
    localparam [10:0] D_START = 11'd60;    // 6% 才起转
    localparam [10:0] D_STOP  = 11'd25;    // 2.5% 以下停转
    localparam integer KICK_MS = 200;
    localparam [31:0] TACH_TH = 32'd15000;

    // 1/R 的两端（Q12）：R_max=34 → 120，R_min=8 → 512
    localparam [15:0] INV_R_LO   = 16'd120;
    localparam [15:0] INV_R_SPAN = 16'd392;

    reg [15:0] lfsr;
    reg [9:0]  kick_cnt;
    reg [31:0] tach_acc;

    //------------------------------------------------------------------
    // 1) 占空比 → 目标转速（16 点表 + 死区迟滞）
    //------------------------------------------------------------------
    reg [15:0] tab_rpm;
    always @(*) begin
        case (duty[10:6])
            5'd0:    tab_rpm = 16'd0;
            5'd1:    tab_rpm = 16'd350;
            5'd2:    tab_rpm = 16'd550;
            5'd3:    tab_rpm = 16'd800;
            5'd4:    tab_rpm = 16'd1050;
            5'd5:    tab_rpm = 16'd1300;
            5'd6:    tab_rpm = 16'd1550;
            5'd7:    tab_rpm = 16'd1800;
            5'd8:    tab_rpm = 16'd2050;
            5'd9:    tab_rpm = 16'd2300;
            5'd10:   tab_rpm = 16'd2550;
            5'd11:   tab_rpm = 16'd2800;
            5'd12:   tab_rpm = 16'd3100;
            5'd13:   tab_rpm = 16'd3400;
            5'd14:   tab_rpm = 16'd3800;
            default: tab_rpm = 16'd4200;
        endcase
    end

    wire want_spin = (duty >= D_START) || ((duty >= D_STOP) && (rpm != 16'd0));

    //------------------------------------------------------------------
    // 2) 转速动态：启动延迟 + 一阶惯性 + 抖动
    //------------------------------------------------------------------
    wire [15:0] jit = {10'd0, lfsr[5:0]} - 16'd32;
    wire [15:0] tgt_trim  = want_spin ? tab_rpm : 16'd0;
    wire [31:0] tgt_scaled = tgt_trim * unit_trim_q8;          // 16×16，1 个 DSP
    wire [15:0] tgt = ((tgt_scaled >> 8) > 32'd4600) ? 16'd4600 : tgt_scaled[23:8];
    wire [15:0] t   = want_spin ? tgt : 16'd0;

    // 目标转速打一拍：占空比→查表→个体差异乘法 也是一条长链
    reg [15:0] tgt_r;
    always @(posedge clk) begin
        if (!rst_n)    tgt_r <= 16'd0;
        else if (tick_ms) tgt_r <= t;
    end

    wire signed [16:0] rpm_s = $signed({1'b0, rpm});
    wire signed [16:0] tgt_s = $signed({1'b0, tgt_r});
    wire signed [16:0] step  = (tgt_s - rpm_s + $signed(jit)) >>> 8;   // τ≈256 ms

    assign stalled = (duty >= D_START) && (rpm == 16'd0) && (kick_cnt >= KICK_MS);

    always @(posedge clk) begin
        if (!rst_n) begin
            lfsr     <= 16'hACE1;
            rpm      <= 16'd0;
            kick_cnt <= 10'd0;
            tach     <= 1'b0;
            tach_acc <= 32'd0;
        end else if (tick_ms) begin
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

            // 启动延迟：从静止起转，要先空转 KICK_MS 才出转速
            if (tgt_r != 16'd0 && kick_cnt < KICK_MS)
                kick_cnt <= kick_cnt + 10'd1;
            else if (tgt_r == 16'd0)
                kick_cnt <= 10'd0;

            if ((tgt_r == 16'd0) || (kick_cnt >= KICK_MS))
                rpm <= (tgt_r == 16'd0 && rpm < 16'd32) ? 16'd0
                                                       : (rpm_s[15:0] + step[15:0]);

            // tach：2 脉冲/转 → 每秒翻转 rpm/15 次
            tach_acc <= tach_acc + {16'd0, rpm};
            if (tach_acc >= TACH_TH) begin
                tach_acc <= tach_acc - TACH_TH;
                tach     <= ~tach;
            end


        end
    end

    // 流量与 1/R2 打一拍：rpm → 流量 → 1/R 这条链会一路连到 thermal_core 的
    // 乘法器输入端，组合过去是 8 级逻辑（实测 −0.09 ns 就差这一点）。
    // 打一拍只引入 10 ns 延迟，相对 1 ms 的热积分步长可忽略。
    wire [15:0] flow_raw   = (rpm >= 16'd4080) ? 16'd255 : {12'd0, rpm[15:4]};
    wire [17:0] flow_scale = 18'd392 * flow_raw;               // 10×8，纯 LUT
    wire [16:0] inv_sum    = {1'b0, INV_R_LO} + flow_scale[16:8];

    reg [15:0] inv_r2_r;
    reg [7:0]  flow_r;
    always @(posedge clk) begin
        inv_r2_r <= inv_sum[15:0];
        flow_r   <= flow_raw[7:0];
    end
    assign inv_r2_q12 = inv_r2_r;
    assign flow_q8    = flow_r;

endmodule



