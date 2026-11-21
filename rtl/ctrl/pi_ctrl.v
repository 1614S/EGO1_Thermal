`timescale 1ns/1ps
//======================================================================
// pi_ctrl.v —— 位置式 PI + 设定值斜坡 + 前馈 + 抗饱和 + 死区
//
//   e      = Tj_filt - sp_eff（偏热为正 → 加大风扇）
//   sp_eff : 以 0.25 ℃/样本 逼近 setpoint（上电从当前温度起步，避免大误差）
//   p      = (e × Kp) >>> 16                  Kp: Q8.8, duty/℃
//   i_acc  : Q16.16 积分器，i += e×Ki          Ki: Q8.8, duty/(℃·样本)
//   duty   = ff_duty + p + (i_acc >>> 16)，限幅 0…1000
//
// 为什么是四级流水：
//   不分流水时"误差 → 两个乘法 → 两次限幅比较"是一条 23 级逻辑的链
//   （实测 WNS −6.6 ns）。PI 每 100 ms 才更新一次，4 个时钟的延迟
//   （40 ns）对控制环完全没有影响。
//     级1 误差 + 设定值斜坡   级2 两个乘法
//     级3 积分累加 + 抗饱和 + 输出限幅   级4 死区
//
// 积分允许取负：有前馈时，前馈的偏置误差只能靠负积分修掉。
//======================================================================
module pi_ctrl #(
    parameter integer INVERT_ERROR = 0,     // 0 = 制冷（风扇：T>sp 加 duty）；1 = 加热（T<sp 加 duty）
    parameter integer SP_RAMP = 64,         // 设定值斜坡，Q8.8 K/样本；64 = 0.25 K
    parameter integer E_ILIM   = 32767      // 积分分离阈值（Q8.8）；32767 等于不分离
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               update,
    input  wire               enable,
    input  wire signed [15:0] tj_filt,
    input  wire        [15:0] setpoint,
    input  wire        [15:0] kp_q8_8,
    input  wire        [15:0] ki_q8_8,
    input  wire        [10:0] ff_duty,      // 前馈占空比（0…1000），PI 只修残差
    output wire        [10:0] duty,
    output reg                saturated
);

    // 积分用 Q8.8（1 LSB = 1/256 占空比，远细于 0.1% 的显示精度），
    // 这样"加法 + 限幅比较"只有 20 位宽，不再是 40 位长的进位链
    localparam signed [20:0] I8_MAX =  21'sd256000;   // 1000 × 256
    localparam signed [20:0] I8_MIN = -21'sd256000;

    reg signed [15:0] sp_eff;
    reg signed [20:0] i_acc;        // Q8.8 占空比
    reg               first;

    reg signed [16:0] e_c;          // 级1 输出
    reg signed [31:0] p_r;          // 级2：比例项（duty）
    reg signed [20:0] i_r;          // 级2：积分增量（Q8.8 duty，已饱和）
    reg        [10:0] duty_r;
    reg               sat_r;
    reg        [4:0]  upd_d;        // update 的流水标记（5 级）

    assign duty = duty_r;

    //------------------------------------------------------------------
    // 级1：设定值斜坡 + 误差
    //------------------------------------------------------------------
    reg signed [15:0] sp_c;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sp_eff <= 16'sd0;
            first  <= 1'b1;
            e_c    <= 17'sd0;
            sp_c   <= 16'sd0;
            upd_d  <= 5'd0;
        end else begin
            upd_d <= {upd_d[3:0], (update && enable)};

            if (update && enable) begin
                if (first) begin
                    sp_eff <= tj_filt;
                    first  <= 1'b0;
                    sp_c   <= tj_filt;
                end else if (sp_eff + SP_RAMP < setpoint) begin
                    sp_eff <= sp_eff + SP_RAMP;
                    sp_c   <= sp_eff;
                end else if (sp_eff - SP_RAMP > setpoint) begin
                    sp_eff <= sp_eff - SP_RAMP;
                    sp_c   <= sp_eff;
                end else begin
                    sp_eff <= setpoint;
                    sp_c   <= setpoint;
                end
                e_c <= INVERT_ERROR ? ($signed(sp_c) - $signed(tj_filt))
                                     : ($signed(tj_filt) - $signed(sp_c));
            end
        end
    end

    //------------------------------------------------------------------
    // 级2：乘法（两个 DSP）
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_r <= 40'sd0;
            i_r <= 40'sd0;
        end else if (upd_d[0]) begin
            p_r <= ($signed(e_c) * $signed({1'b0, kp_q8_8})) >>> 16;
            // 积分分离：|e| > E_ILIM 时冻结积分（过渡过程只靠 P + 前馈），
            // 避免慢对象（加热器 τ 几十秒）把积分积到饱和造成大超调
            if ((e_c > E_ILIM) || (e_c < -E_ILIM)) begin
                i_r <= 40'sd0;
            end else begin
                // e[Q8.8] × Ki[Q8.8] = Q16.16 duty → 右移 8 位得 Q8.8，并按积分范围饱和
                i_r <= (($signed(e_c) * $signed({4'd0, ki_q8_8[11:0]})) >>> 8) > I8_MAX ? I8_MAX :
                   (($signed(e_c) * $signed({4'd0, ki_q8_8[11:0]})) >>> 8) < I8_MIN ? I8_MIN :
                    (($signed(e_c) * $signed({4'd0, ki_q8_8[11:0]})) >>> 8);
            end
        end
    end

    //------------------------------------------------------------------
    // 级3：积分累加 + 抗饱和 + 输出限幅
    //------------------------------------------------------------------
    reg signed [21:0] i_sum;        // 级3：积分累加（只做加法）
    reg signed [20:0] i_c;          // 级4：限幅后的积分值
    reg signed [31:0] u_c;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_sum <= 22'sd0;
            i_acc <= 21'sd0;
        end else if (upd_d[1]) begin
            i_sum <= i_acc + i_r;                 // 22 bit 加法，独立一级
        end else if (upd_d[2]) begin
            if (i_sum > I8_MAX)      i_c <= I8_MAX;
            else if (i_sum < I8_MIN) i_c <= I8_MIN;
            else                     i_c <= i_sum[20:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty_r  <= 11'd0;
            sat_r   <= 1'b0;
        end else if (upd_d[3]) begin
            // i_c 是 Q8.8，右移 8 位得到 duty
            u_c = p_r + {{19{i_c[20]}}, i_c[20:8]} + $signed({21'd0, ff_duty});

            if (u_c > 32'sd1000) begin
                duty_r <= 11'd1000;
                sat_r  <= 1'b1;
            end else if (u_c < 32'sd0) begin
                duty_r <= 11'd0;
                sat_r  <= 1'b1;
            end else begin
                duty_r <= u_c[10:0];
                sat_r  <= 1'b0;
            end

            // 未饱和才允许积分（抗饱和）
            if ((u_c <= 32'sd1000) && (u_c >= 32'sd0))
                i_acc <= i_c;
        end
    end

    //------------------------------------------------------------------
    // 级4：死区 ±0.2 ℃ —— 保持输出并冻结积分
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 本级的寄存器都在上面初始化，这里只做覆盖
        end else if (upd_d[4]) begin
            if ((e_c < 17'sd51) && (e_c > -17'sd51)) begin
                duty_r <= duty_r;
                i_acc  <= i_acc;
                sat_r  <= 1'b0;
            end
        end
    end

endmodule

