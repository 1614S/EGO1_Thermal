`timescale 1ns/1ps
//======================================================================
// ambient_vplant.v —— 环境温度模型（应对"没有恒温实验室"这件事）
//
// 三种真实扰动叠加：
//   1. 上电基准 25 ℃（可由寄存器强制覆盖，做"20…60 ℃ 全范围"验证）
//   2. 缓慢随机游走：每 100 ms 走 ±128 µK，10 s 量级漂移约 ±0.7 mK
//      （对应"早晚温差、房间气流"这种无关紧要但真实存在的慢漂）
//   3. 空调阶跃：ac_toggle 一次 ±4 ℃（对应"开/关空调"，这才是真会毁掉
//      实测曲线的干扰；虚拟热台里可以随时复现，不用等天气）
//======================================================================
`include "vplant_pkg.vh"

module ambient_vplant (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_ms,
    input  wire               ac_toggle,        // 单拍：开/关空调
    input  wire               force_en,         // 1 = 用 ta_force_uk
    input  wire signed [31:0] ta_force_uk,
    output reg  signed [31:0] ta_uk,
    output reg                ac_on,
    output reg  signed [31:0] drift_uk         // 累计漂移（监测）
);
    reg [15:0] lfsr;
    reg [6:0]  ms_cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            lfsr     <= 16'h7A31;
            ms_cnt   <= 7'd0;
            ac_on    <= 1'b0;
            drift_uk <= 32'sd0;
            ta_uk    <= `VP_TAMB_DEF_UK;
        end else begin
            if (ac_toggle) ac_on <= ~ac_on;
            if (tick_ms) begin
                lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[12] ^ lfsr[10] ^ lfsr[4]};
                if (ms_cnt >= 7'd99) begin
                    ms_cnt   <= 7'd0;
                    // 随机游走：-128…+127 µK / 100 ms
                    drift_uk <= drift_uk + {{24{lfsr[7]}}, lfsr[7:0]} - 32'sd128;
                end else begin
                    ms_cnt <= ms_cnt + 7'd1;
                end
            end
            if (force_en)
                ta_uk <= ta_force_uk + (ac_on ? -`VP_TAMB_STEP_UK : 32'sd0);
            else
                ta_uk <= `VP_TAMB_DEF_UK + drift_uk +
                         (ac_on ? -`VP_TAMB_STEP_UK : 32'sd0);
        end
    end

endmodule


