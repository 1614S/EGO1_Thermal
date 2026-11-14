//======================================================================
// pi_ctrl.v —— 位置式 PI + 设定值斜坡 + 抗饱和 + 死区
//
//   e      = Tj_filt - sp_eff（偏热为正 -> 加大风扇）
//   sp_eff : 以 0.25 ℃/样本 逼近 setpoint（上电时从当前温度起步，
//            避免大误差导致积分饱和）
//   p      = (e * kp) >>> 16                kp: Q8.8, duty/℃
//   i_acc : Q16.16 积分器，i += e*ki         ki: Q8.8, duty/(℃·样本)
//   duty  : p + (i_acc >>> 16)，限幅 0…1000
//======================================================================
module pi_ctrl (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               update,
    input  wire               enable,
    input  wire signed [15:0] tj_filt,
    input  wire signed [15:0] setpoint,
    input  wire        [15:0] kp_q8_8,
    input  wire        [15:0] ki_q8_8,
    output wire [10:0]        duty,
    output reg                saturated
);

    localparam signed [31:0] I_MAX = 32'sd1000 * 32'sd65536;

    reg signed [15:0] sp_eff;
    reg signed [31:0] i_acc;
    reg               first;

    reg signed [15:0] e_c;
    reg signed [15:0] sp_c;
    reg signed [16:0] p_c;
    reg signed [31:0] i_c;
    reg signed [31:0] u_c;
    reg        [10:0] duty_r;
    reg               sat_r;

    assign duty = duty_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sp_eff    <= 16'sd0;
            i_acc     <= 32'sd0;
            first     <= 1'b1;
            duty_r    <= 11'd0;
            sat_r     <= 1'b0;
            e_c       <= 17'sd0;
            p_c       <= 17'sd0;
            i_c       <= 32'sd0;
            u_c       <= 32'sd0;
        end else if (update && enable) begin
            //---------------- 设定值斜坡 ----------------
            if (first) begin
                sp_eff <= tj_filt;
                first  <= 1'b0;
            end else if (sp_eff + 17'sd64 < setpoint) begin
                sp_eff <= sp_eff + 16'sd64;
            end else if (sp_eff - 17'sd64 > setpoint) begin
                sp_eff <= sp_eff - 16'sd64;
            end else begin
                sp_eff <= setpoint;
            end

            //---------------- PI ----------------
            // 乘法必须显式扩到 32 bit（否则按 17 bit 上下文截断）
            // 第一个样本用当前温度做等效设定值，保证上电无扰
            sp_c = first ? tj_filt : sp_eff;
            e_c  = tj_filt - sp_c;
            p_c = (e_c * $signed({16'd0, kp_q8_8})) >>> 16;
            i_c = i_acc + (e_c * $signed({16'd0, ki_q8_8}));

            if (i_c > I_MAX) i_c = I_MAX;
            if (i_c < 0)     i_c = 32'sd0;

            u_c = p_c + $signed(i_c[31:16]);

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

            // 死区 ±0.2 ℃：保持输出
            if ((e_c < 17'sd51) && (e_c > -17'sd51)) begin
                duty_r <= duty_r;
                sat_r  <= 1'b0;
            end
        end
    end

endmodule
