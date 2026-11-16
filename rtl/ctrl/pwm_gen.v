`timescale 1ns/1ps
//======================================================================
// pwm_gen.v —— 12 bit PWM，载频 100 MHz / 4096 = 24.414 kHz
// duty: 0…1000（0.1 %），占空比在计数器回零附近更新，避免窄脉冲。
//======================================================================
`include "ego1_thermal_pkg.vh"

module pwm_gen #(
    parameter integer BITS = `PWM_BITS
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [10:0] duty,        // 0…1000
    output reg         pwm
);

    reg  [BITS-1:0] cnt;
    reg  [10:0]     duty_r;
    wire [21:0]     cmp_full;
    wire [BITS-1:0] cmp;

    // 4096/1000 ≈ 4.194 -> 用 4194/1024 近似
    assign cmp_full = duty_r * 22'd4194;
    assign cmp      = cmp_full[21:10];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt    <= {BITS{1'b0}};
            duty_r <= 11'd0;
            pwm    <= 1'b0;
        end else begin
            cnt <= cnt + 1'b1;
            // 在周期边界锁存占空比
            if (cnt == {BITS{1'b0}})
                duty_r <= duty;
            pwm <= (cnt < cmp);
        end
    end

endmodule
