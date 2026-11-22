`timescale 1ns/1ps
//======================================================================
// ext_loop.v —— 环境温度（AM2302）闭环：**饱和逼近 + PI 精细调控**
//
// 结构（2026-11-22 改进：大误差直接 0/100 %，接近设定值再交给 PI）：
//
//   |e| > HYST(4.0 K)  ┌─ 饱和区：e>0 → 100 %；e<0 → 0 %
//   |e| ≤ BAND(3.0 K)  └─ PI 区 ：前馈(KFF×ΔT) + PI(Kp,Ki) 精细调节
//   两者之间 → 保持原模式（迟滞，避免在边界反复切换）
//
// 无扰切换：输出经过斜率限制（SLEW duty/100 ms），从 1000 降到 PI 目标
//   约 2 s，不会出现"瞬间断电"造成的温度跌落。
//
// 前馈标定（2026-11-22 实测 results/heat_no_mos.csv，5 V/10 Ω 常通 = 2.5 W）：
//   AM2302 从 23.0 ℃ 升到 48.2 ℃（+25.2 K/10 min），指数拟合稳态外推 89.9 ℃
//   → θ_eff = (89.9−23.0)/2.5 W = 26.7 K/W
//   → k_ff = 1000/(θ_eff × Pmax) = 1000/(26.7×2.5) = 15 duty/K
//
// 安全：heater_en=0 时输出**立即**清零（不走斜率限制）。
//======================================================================
module ext_loop #(
    // 前馈增益：duty_ff = (Tset − T_room)[Q8.8] × KFF_NUM / 256
    parameter integer KFF_NUM  = 15,      // 15 duty/K（实测 θ_eff≈26.7 K/W、Pmax=2.5 W）
    parameter integer BAND_Q88 = 768,     // 3.0 K：进入 PI 区
    parameter integer HYST_Q88 = 1024,    // 4.0 K：退回饱和区
    parameter integer SLEW     = 40       // 每 100 ms 最多变化 40 duty（0→1000 约 2.5 s）
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               update,        // 100 ms 单拍
    input  wire signed [15:0] t_ext,         // AM2302 环境温度 Q8.8（被加热对象）
    input  wire signed [15:0] t_room,        // 室温参考（未经加热的位置，可给常数）
    input  wire signed [15:0] sp_ext,        // 目标环境温度 Q8.8
    input  wire        [15:0] kp_q8_8,
    input  wire        [15:0] ki_q8_8,
    input  wire               heater_en,     // 0 = 强制关断（告警/超温/数据失效）
    output wire        [10:0] duty,
    output reg                bang,          // 1 = 处于饱和区（便于观测/调试）
    output wire               saturated
);

    //------------------------------------------------------------------
    // 前馈：稳态占空比 = (Tset − T_room) × KFF
    //   必须用**室温**做参考：被加热的传感器到达设定值后 (Tset−T)=0，
    //   用它做前馈会得到 0，稳态只能靠积分硬撑（超调大、易饱和）。
    //------------------------------------------------------------------
    wire signed [16:0] dt    = $signed({1'b0, sp_ext}) - $signed({1'b0, t_room});
    wire signed [20:0] ff_m  = dt * KFF_NUM;
    wire signed [16:0] ff_s  = (dt > 17'sd0) ? (ff_m >>> 8) : 17'sd0;
    wire        [10:0] ff_d  = (ff_s > 17'sd1000) ? 11'd1000 : ff_s[10:0];

    // 打一拍：减法 + 限幅 + PI 的 32 位加法原来挤在同一拍里（19 级 / 15 个 CARRY4，
    // WNS −1.19 ns）。PI 每 100 ms 才更新一次，多一拍完全无影响。
    reg [10:0] ff_d_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ff_d_r <= 11'd0;
        else        ff_d_r <= ff_d;
    end

    //------------------------------------------------------------------
    // PI（复用结温环的模块；积分分离阈值 2 K，避免饱和区积分饱和）
    //------------------------------------------------------------------
    wire [10:0] duty_pi;
    pi_ctrl #(.INVERT_ERROR(1), .SP_RAMP(64), .E_ILIM(512)) u_pi_ext (
        .clk      (clk),
        .rst_n    (rst_n),
        .update   (update),
        .enable   (heater_en),
        .tj_filt  (t_ext),
        .setpoint (sp_ext),
        .kp_q8_8  (kp_q8_8),
        .ki_q8_8  (ki_q8_8),
        .ff_duty  (ff_d_r),
        .duty     (duty_pi),
        .saturated(saturated)
    );

    //------------------------------------------------------------------
    // 误差与模式判定（带迟滞）
    //------------------------------------------------------------------
    wire signed [16:0] err  = $signed({1'b0, sp_ext}) - $signed({1'b0, t_ext});  // 正 = 需要加热
    wire signed [16:0] aerr = err[16] ? (17'sd0 - err) : err;                    // |err|

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      bang <= 1'b1;                 // 上电先饱和逼近
        else if (update) begin
            // 迟滞的正确方向：误差**缩小**到 BAND(3 K) 才退出饱和；
            // 误差**涨过** HYST(4 K) 才回到饱和。两者都在 [3,4] 之间时保持原模式。
            //   （旧代码把两个阈值写反了：退出阈值 4 K > 回退阈值 3 K，
            //     误差停在 4 K 附近就会 饱和↔PI 反复抖动，上板实测 H 在 960↔1000 跳）
            if (bang) bang <= (aerr > BAND_Q88);       // 饱和区：误差进入 3 K 以内 → 交给 PI
            else      bang <= (aerr > HYST_Q88);       // PI 区：误差涨过 4 K → 回饱和
        end
    end

    // 目标占空比：饱和区 = 0/1000；PI 区 = PI 输出
    wire [10:0] target = (!heater_en) ? 11'd0 :
                         bang ? (err[16] ? 11'd0 : 11'd1000) : duty_pi;

    //------------------------------------------------------------------
    // 斜率限制：保证饱和区↔PI 区切换时没有阶跃（无扰切换）
    //   安全门控（heater_en=0）时立即清零，不做斜率限制。
    //------------------------------------------------------------------
    reg [10:0] duty_r;
    wire [11:0] tgt12 = {1'b0, target};
    wire [11:0] up12  = {1'b0, duty_r} + SLEW;
    wire [11:0] dn12  = ({1'b0, duty_r} > SLEW) ? ({1'b0, duty_r} - SLEW) : 12'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)              duty_r <= 11'd0;
        else if (!heater_en)     duty_r <= 11'd0;                  // 安全：立即关断
        else if (update) begin
            if      (tgt12 > up12) duty_r <= up12[10:0];
            else if (tgt12 < dn12) duty_r <= dn12[10:0];
            else                   duty_r <= target;
        end
    end

    assign duty = duty_r;

endmodule
