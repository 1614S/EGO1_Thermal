`timescale 1ns/1ps
//======================================================================
// gear_ctrl.v —— 档位仲裁（手动 / 自动 DVFS 外环 / 保护）
//
// 优先级：保护（supervisor 的 gear_low）> 自动外环 > 手动
//
// 自动外环的判据（秒级，避免和 10 Hz 的内环打架）：
//   占空比 > 95 % 且 Tj > Tset+1 ℃ 持续 5 s   → 降一档（降频降温）
//   占空比 < 10 % 且 Tj < Tset−1 ℃ 持续 10 s  → 升一档（有余量就提性能）
//   换档后至少驻留 5 s，防止来回抖
//
// 这就是赛题"动态频率调节演示"要的 DVFS：不是手动按键换时钟，而是
// "风扇吹不动了 → 自己降频 → 功耗下来 → 温度回落"的闭环因果链。
//======================================================================
module gear_ctrl #(
    parameter integer GEAR_NUM  = 5,      // 0…4
    parameter integer DWELL_MS  = 5000,   // 换档后最小驻留
    parameter integer DOWN_MS   = 5000,   // 降档判据持续时间
    parameter integer UP_MS     = 10000   // 升档判据持续时间
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_ms,
    input  wire               auto_en,       // 1 = 自动外环
    input  wire               prot,          // 1 = 保护动作（强制最低档）
    input  wire [10:0]        duty,
    input  wire signed [15:0] tj,
    input  wire signed [15:0] setpoint,
    input  wire        [2:0]  gear_man,
    output reg         [2:0]  gear,          // 最终档位
    output reg         [2:0]  gear_auto,     // 自动外环自己的档位（监测）
    output reg  [15:0]        n_down,        // 累计降档次数（监测）
    output reg  [15:0]        n_up           // 累计升档次数（监测）
);
    // ±1 ℃ 的门限（Q8.8）
    localparam signed [16:0] HYS = 17'sd256;

    reg [15:0] dwell_ms;
    reg [15:0] up_ms, dn_ms;

    wire signed [16:0] tj_s   = $signed({1'b0, tj});
    wire signed [16:0] sp_s   = $signed({1'b0, setpoint});
    wire               hot    = (duty > 11'd950) && (tj_s > sp_s + HYS);
    wire               cool   = (duty < 11'd100) && (tj_s < sp_s - HYS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gear_auto <= 3'd3;
            dwell_ms  <= 16'd0;
            up_ms     <= 16'd0;
            dn_ms     <= 16'd0;
            n_down    <= 16'd0;
            n_up      <= 16'd0;
        end else if (tick_ms) begin
            if (dwell_ms < DWELL_MS) dwell_ms <= dwell_ms + 16'd1;

            if (!auto_en || dwell_ms < DWELL_MS) begin
                up_ms <= 16'd0;
                dn_ms <= 16'd0;
            end else begin
                // ---- 降档：风扇吹不动了 ----
                if (hot) begin
                    if (dn_ms < DOWN_MS) dn_ms <= dn_ms + 16'd1;
                    if (dn_ms >= DOWN_MS - 1 && gear_auto != 3'd0) begin
                        gear_auto <= gear_auto - 3'd1;
                        n_down    <= n_down + 16'd1;
                        dn_ms     <= 16'd0;
                        dwell_ms  <= 16'd0;
                        up_ms     <= 16'd0;
                    end
                end else dn_ms <= 16'd0;

                // ---- 升档：还有余量，提性能 ----
                if (cool) begin
                    if (up_ms < UP_MS) up_ms <= up_ms + 16'd1;
                    if (up_ms >= UP_MS - 1 && gear_auto != GEAR_NUM - 1) begin
                        gear_auto <= gear_auto + 3'd1;
                        n_up      <= n_up + 16'd1;
                        up_ms     <= 16'd0;
                        dwell_ms  <= 16'd0;
                        dn_ms     <= 16'd0;
                    end
                end else up_ms <= 16'd0;
            end
        end
    end

    // 输出仲裁
    always @(*) begin
        if (prot)          gear = 3'd0;
        else if (auto_en)  gear = gear_auto;
        else               gear = gear_man;
    end

endmodule
