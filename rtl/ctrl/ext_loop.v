`timescale 1ns/1ps
//======================================================================
// ext_loop.v —— 环境温度（AM2302）闭环：前馈 + PI + 安全门控
//
// 与结温环（pi_ctrl）同款结构，但对象是"加热电阻"：
//   P = V²/R × duty，温度对占空比近似一阶线性 → 稳态 duty ≈ k_ff × (Tset − Tamb)
//   前馈取 k_ff = 4 duty/K（按 10 Ω/5 W、5 V 供电、装置 θ≈100 K/W 估：
//   ΔT=35 K 需 0.35 W ≈ 14 % 占空比），PI 只修残差。
//
// 安全：heater_en=0 时输出强制 0（告警/超温/传感器失效/时钟未锁定），
//       绝不能像风扇那样"故障反而拉满"。
//======================================================================
module ext_loop (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               update,        // 100 ms 单拍
    input  wire signed [15:0] t_ext,         // AM2302 环境温度 Q8.8（被加热对象）
    input  wire signed [15:0] t_room,        // 室温参考（未经加热的位置，可给常数）
    input  wire signed [15:0] sp_ext,        // 目标环境温度 Q8.8
    input  wire        [15:0] kp_q8_8,
    input  wire        [15:0] ki_q8_8,
    input  wire               heater_en,     // 0 = 强制关断
    output wire        [10:0] duty,
    output wire               saturated
);

    //------------------------------------------------------------------
    // 前馈：稳态占空比 = (Tset − T_room)/θ / Pmax
    //   以 10 Ω/5 W、5 V（Pmax=2.5 W）、装置 θ≈100 K/W 标定 → 4 duty/K
    //   注意必须用**室温**做参考：被加热的传感器温度到达设定值后 (Tset−T)=0，
    //   用它做前馈会得到 0，稳态只能靠积分硬撑（超调大、易饱和）。
    //   Q8.8 下 1 K = 256 LSB，×4/256 = >>6
    //------------------------------------------------------------------
    wire signed [16:0] dt    = $signed({1'b0, sp_ext}) - $signed({1'b0, t_room});
    wire signed [16:0] ff_s  = (dt > 17'sd0) ? (dt >>> 6) : 17'sd0;
    wire        [10:0] ff_d  = (ff_s > 17'sd1000) ? 11'd1000 : ff_s[10:0];

    // 打一拍：减法 + 限幅 + PI 的 32 位加法原来挤在同一拍里（19 级 / 15 个 CARRY4，
    // WNS −1.19 ns）。PI 每 100 ms 才更新一次，多一拍完全无影响。
    reg [10:0] ff_d_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ff_d_r <= 11'd0;
        else        ff_d_r <= ff_d;
    end

    //------------------------------------------------------------------
    // PI（复用结温环的模块，参数由外部给）
    //------------------------------------------------------------------
    wire [10:0] duty_pi;
    pi_ctrl #(.INVERT_ERROR(1), .SP_RAMP(64), .E_ILIM(256)) u_pi_ext (
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

    assign duty = heater_en ? duty_pi : 11'd0;

endmodule
