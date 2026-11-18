`timescale 1ns/1ps
//======================================================================
// reg_file.v —— 32 个 32 bit 寄存器（RW 配置 + RO 状态）
// 地址见 docs/01_架构与仿真方案.md 表 5.3
//======================================================================
`include "ego1_thermal_pkg.vh"

module reg_file (
    input  wire               clk,
    input  wire               rst_n,
    // 写
    input  wire               wr_en,
    input  wire        [5:0]  wr_addr,
    input  wire        [31:0] wr_data,
    // 读
    input  wire        [5:0]  rd_addr,
    output reg         [31:0] rd_data,
    // RW 输出
    output wire        [15:0] setpoint,
    output wire        [15:0] kp,
    output wire        [15:0] ki,
    output wire        [15:0] th_warn,
    output wire        [15:0] th_hot,
    output wire        [15:0] th_crit,
    output wire        [15:0] cal_a,
    output wire signed [15:0] cal_b,
    output wire        [2:0]  gear_man,
    output wire        [10:0] duty_man,
    output wire        [3:0]  control,       // {load_en, fan_man, auto_gear, log_en}
    output wire        [31:0] scratch,
    // RO 输入
    input  wire signed [15:0] tj_f,
    input  wire signed [15:0] tj_raw,
    input  wire signed [15:0] text_f,
    input  wire signed [15:0] text_raw,
    input  wire        [15:0] rh,
    input  wire        [15:0] ext_age,
    input  wire        [10:0] duty_act,
    input  wire        [2:0]  gear_act,
    input  wire        [1:0]  state_act,
    input  wire        [15:0] f_meas,
    input  wire        [15:0] p_est,
    input  wire        [11:0] code_raw,
    input  wire        [15:0] sample_cnt,
    input  wire        [15:0] err_cnt,
    input  wire        [15:0] switch_max
);

    reg [15:0] r_setpoint;
    reg [15:0] r_kp;
    reg [15:0] r_ki;
    reg [15:0] r_warn;
    reg [15:0] r_hot;
    reg [15:0] r_crit;
    reg [15:0] r_cal_a;
    reg [15:0] r_cal_b;
    reg [15:0] r_gear;
    reg [15:0] r_duty_man;
    reg [3:0]  r_ctrl;
    reg [31:0] r_scratch;

    assign setpoint = r_setpoint;
    assign kp       = r_kp;
    assign ki       = r_ki;
    assign th_warn  = r_warn;
    assign th_hot   = r_hot;
    assign th_crit  = r_crit;
    assign cal_a    = r_cal_a;
    assign cal_b    = r_cal_b;
    assign gear_man = r_gear[2:0];
    assign duty_man = r_duty_man[10:0];
    assign control  = r_ctrl;
    assign scratch  = r_scratch;

    initial begin
        r_setpoint = 16'd12800;          // 50.0 ℃
        r_kp       = {5'd0, `KP_DEF_Q8_8};
        r_ki       = {5'd0, `KI_DEF_Q8_8};
        r_warn     = `TH_WARN_Q8_8;
        r_hot      = `TH_HOT_Q8_8;
        r_crit     = `TH_CRIT_Q8_8;
        r_cal_a    = 16'h1000;           // 1.000
        r_cal_b    = 16'h0000;
        r_gear     = 16'd3;              // 100 MHz
        r_duty_man = 16'd0;
        r_ctrl     = 4'b1000;            // load_en=1, fan_man=0, auto_gear=0(手动档，sw5 开自动), log_en=0
        r_scratch  = 32'h0;
    end

    //------------------------------------------------------------------
    // 写
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_setpoint <= 16'd12800;
            r_kp       <= `KP_DEF_Q8_8;
            r_ki       <= `KI_DEF_Q8_8;
            r_warn     <= `TH_WARN_Q8_8;
            r_hot      <= `TH_HOT_Q8_8;
            r_crit     <= `TH_CRIT_Q8_8;
            r_cal_a    <= 16'h1000;
            r_cal_b    <= 16'h0000;
            r_gear     <= 16'd3;
            r_duty_man <= 16'd0;
            r_ctrl     <= 4'b1000;
            r_scratch  <= 32'h0;
        end else if (wr_en) begin
            case (wr_addr)
                6'h03: r_ctrl     <= wr_data[3:0];
                6'h04: r_setpoint <= wr_data[15:0];
                6'h05: r_gear     <= wr_data[15:0];
                6'h06: r_duty_man <= wr_data[15:0];
                6'h07: r_kp       <= wr_data[15:0];
                6'h08: r_ki       <= {4'd0, wr_data[11:0]};   // Ki ≤ 15.996，保证积分乘法位宽可控
                6'h09: r_warn     <= wr_data[15:0];
                6'h0A: r_hot      <= wr_data[15:0];
                6'h0B: r_crit     <= wr_data[15:0];
                6'h0C: r_cal_a    <= wr_data[15:0];
                6'h0D: r_cal_b    <= wr_data[15:0];
                6'h1F: r_scratch  <= wr_data;
                default: ;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 读
    //------------------------------------------------------------------
    always @(*) begin
        case (rd_addr)
            6'h00: rd_data = 32'h4547_4F31;    // "EGO1"
            6'h01: rd_data = 32'h0001_0000;
            6'h02: rd_data = {26'd0, state_act, 1'b0, gear_act};
            6'h03: rd_data = {28'd0, r_ctrl};
            6'h04: rd_data = {16'd0, r_setpoint};
            6'h05: rd_data = {13'd0, r_gear[2:0]};
            6'h06: rd_data = {21'd0, r_duty_man[10:0]};
            6'h07: rd_data = {16'd0, r_kp};
            6'h08: rd_data = {16'd0, r_ki};
            6'h09: rd_data = {16'd0, r_warn};
            6'h0A: rd_data = {16'd0, r_hot};
            6'h0B: rd_data = {16'd0, r_crit};
            6'h0C: rd_data = {16'd0, r_cal_a};
            6'h0D: rd_data = {16'd0, r_cal_b};
            6'h11: rd_data = {{16{tj_f[15]}}, tj_f};
            6'h12: rd_data = {{16{tj_raw[15]}}, tj_raw};
            6'h13: rd_data = {{16{text_f[15]}}, text_f};
            6'h14: rd_data = {{16{text_raw[15]}}, text_raw};
            6'h15: rd_data = {21'd0, duty_act};
            6'h16: rd_data = {29'd0, gear_act};
            6'h17: rd_data = {16'd0, f_meas};
            6'h18: rd_data = {16'd0, p_est};
            6'h19: rd_data = {30'd0, state_act};
            6'h1A: rd_data = {16'd0, code_raw};
            6'h1B: rd_data = {16'd0, rh};
            6'h1C: rd_data = {16'd0, sample_cnt};
            6'h1D: rd_data = {16'd0, err_cnt};
            6'h1E: rd_data = {16'd0, switch_max};
            6'h1F: rd_data = r_scratch;
            default: rd_data = 32'h0;
        endcase
    end

endmodule



