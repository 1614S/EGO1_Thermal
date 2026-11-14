//======================================================================
// tick_gen.v —— 由 clk_sys 产生 1 ms 时基与 1 µs 时基
// 1 ms 时基参与“仿真加速”（SIM_ACCEL），1 µs 时基始终是真实时间，
// 供 AM2301 这类对时序敏感的器件使用。
//======================================================================
`include "ego1_thermal_pkg.vh"

module tick_gen #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer ACCEL  = `SIM_ACCEL
) (
    input  wire clk,
    input  wire rst_n,
    output reg  tick_ms,        // 1 个 clk 宽
    output reg  tick_us         // 1 个 clk 宽（真实 1 µs，不加速）
);

    // 1 ms = CLK_HZ/1000 个周期；加速后除以 ACCEL
    localparam integer MS_DIV = (CLK_HZ / 1000) / ACCEL;
    localparam integer US_DIV = (CLK_HZ / 1_000_000);

    localparam integer MS_W = (MS_DIV > 1) ? $clog2(MS_DIV) : 1;
    localparam integer US_W = (US_DIV > 1) ? $clog2(US_DIV) : 1;

    reg [MS_W-1:0] ms_cnt;
    reg [US_W-1:0] us_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_cnt  <= {MS_W{1'b0}};
            us_cnt  <= {US_W{1'b0}};
            tick_ms <= 1'b0;
            tick_us <= 1'b0;
        end else begin
            // ---- ms ----
            if (MS_DIV <= 1) begin
                ms_cnt  <= {MS_W{1'b0}};
                tick_ms <= 1'b1;
            end else if (ms_cnt == MS_DIV-1) begin
                ms_cnt  <= {MS_W{1'b0}};
                tick_ms <= 1'b1;
            end else begin
                ms_cnt  <= ms_cnt + 1'b1;
                tick_ms <= 1'b0;
            end

            // ---- us ----
            if (US_DIV <= 1) begin
                us_cnt  <= {US_W{1'b0}};
                tick_us <= 1'b1;
            end else if (us_cnt == US_DIV-1) begin
                us_cnt  <= {US_W{1'b0}};
                tick_us <= 1'b1;
            end else begin
                us_cnt  <= us_cnt + 1'b1;
                tick_us <= 1'b0;
            end
        end
    end

endmodule
