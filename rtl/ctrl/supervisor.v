//======================================================================
// supervisor.v —— 四级告警 + 迟滞 + 去抖 + 联锁
//   NORMAL(0) -> WARN(1) -> HOT(2) -> CRIT(3)
//   进入：连续 N 次 eval 超阈值；退出：连续 M 次低于 (阈值 - 3℃)
//   CRIT 锁存，需 clr_alarm 且温度回落才解除。
//======================================================================
`include "ego1_thermal_pkg.vh"

module supervisor (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               eval,          // 100 ms 单拍
    input  wire signed [15:0] tj,
    input  wire        [15:0] th_warn_q8_8,
    input  wire        [15:0] th_hot_q8_8,
    input  wire        [15:0] th_crit_q8_8,
    input  wire               clr_alarm,
    output reg  [1:0]         state,
    output reg                fan_max,       // 强制 100 %
    output reg                gear_low,      // 强制最低档
    output reg                load_en,       // 自热负载使能
    output reg                alarm          // 锁存
);

    localparam [1:0] ST_NORMAL = 2'd0;
    localparam [1:0] ST_WARN   = 2'd1;
    localparam [1:0] ST_HOT    = 2'd2;
    localparam [1:0] ST_CRIT   = 2'd3;

    localparam [15:0] HYST = 16'd768;        // 3.0 ℃ (Q8.8)
    localparam [4:0]  UP_N = 5'd4;           // 0.4 s
    localparam [7:0]  DN_N = 8'd50;          // 5 s

    reg [4:0] up_cnt;
    reg [7:0] dn_cnt;

    wire signed [16:0] tj_s = $signed({1'b0, tj});
    wire signed [16:0] warn_s = $signed({1'b0, th_warn_q8_8});
    wire signed [16:0] hot_s  = $signed({1'b0, th_hot_q8_8});
    wire signed [16:0] crit_s = $signed({1'b0, th_crit_q8_8});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_NORMAL;
            fan_max  <= 1'b0;
            gear_low <= 1'b0;
            load_en  <= 1'b1;
            alarm    <= 1'b0;
            up_cnt   <= 5'd0;
            dn_cnt   <= 8'd0;
        end else if (eval) begin
            // ---------------- 超阈值去抖 ----------------
            if ((state == ST_NORMAL && tj_s >= warn_s) ||
                (state == ST_WARN   && tj_s >= hot_s)  ||
                (state == ST_HOT    && tj_s >= crit_s)) begin
                if (up_cnt >= UP_N-1) up_cnt <= up_cnt;   // 保持
                else                  up_cnt <= up_cnt + 5'd1;
            end else begin
                up_cnt <= 5'd0;
            end

            // ---------------- 回落去抖 ----------------
            if ((state == ST_WARN && tj_s < (warn_s - $signed({1'b0, HYST}))) ||
                (state == ST_HOT  && tj_s < (hot_s  - $signed({1'b0, HYST})))  ||
                (state == ST_CRIT && tj_s < (crit_s - $signed({1'b0, HYST})) && clr_alarm)) begin
                if (dn_cnt >= DN_N-1) dn_cnt <= dn_cnt;
                else                  dn_cnt <= dn_cnt + 8'd1;
            end else begin
                dn_cnt <= 8'd0;
            end

            // ---------------- 状态迁移 ----------------
            case (state)
                ST_NORMAL: begin
                    if (up_cnt >= UP_N-1 && tj_s >= warn_s) state <= ST_WARN;
                end
                ST_WARN: begin
                    if (up_cnt >= UP_N-1 && tj_s >= hot_s)       state <= ST_HOT;
                    else if (dn_cnt >= DN_N-1)                   state <= ST_NORMAL;
                end
                ST_HOT: begin
                    if (up_cnt >= UP_N-1 && tj_s >= crit_s)      state <= ST_CRIT;
                    else if (dn_cnt >= DN_N-1)                   state <= ST_WARN;
                end
                ST_CRIT: begin
                    if (clr_alarm && dn_cnt >= DN_N-1)           state <= ST_NORMAL;
                end
                default: state <= ST_NORMAL;
            endcase

            // ---------------- 输出 ----------------
            if (state == ST_CRIT) begin
                alarm <= 1'b1;
            end else if (clr_alarm) begin
                alarm <= 1'b0;
            end

            fan_max  <= (state == ST_HOT) || (state == ST_CRIT);
            gear_low <= (state == ST_CRIT);
            load_en  <= (state != ST_CRIT);
        end
    end

endmodule
