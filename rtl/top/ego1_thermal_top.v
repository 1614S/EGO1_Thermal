`timescale 1ns/1ps
//======================================================================
// ego1_thermal_top.v —— 顶层（只连线，不写算法）
//
// 数据链：XADC/AM2301 -> acq_fsm -> calib -> MA8 -> IIR -> PI -> PWM -> 风扇
// 遥测  ：sample -> frame_pack -> uart_tx
// 执行  ：MMCM DRP 切档 + 自热负载
//======================================================================
`include "ego1_thermal_pkg.vh"
`include "vplant_pkg.vh"

module ego1_thermal_top (
    input  wire        clk,          // 100 MHz
    input  wire        rst_btn,      // 低有效
    input  wire [4:0]  btn,
    input  wire [7:0]  sw,
    output wire [15:0] led,
    output wire [7:0]  seg,
    output wire [3:0]  an,
    output wire        uart_tx,
    input  wire        uart_rx,
    output wire        fan_pwm,
    input  wire        fan_tach,
    inout  wire        dht_dq
);

    //------------------------------------------------------------------
    // 虚拟热台与真实器件的选择：sw[6]=1 用虚拟热台，=0 用真实 XADC/AM2301
    //   （上板默认 sw6=0；没有风扇/加热片时手动拨到 1 就能跑完整闭环演示）
    //------------------------------------------------------------------
    wire plant_sel = sw[6];
    wire signed [31:0] vp_tj_uk;      // 虚拟结温
    wire signed [31:0] vp_tc_uk;      // 虚拟壳温
    wire signed [31:0] vp_ta_uk;      // 虚拟环境温度
    wire        [31:0] vp_p_mw;       // 虚拟功耗
    wire        [10:0] vp_duty;       // 送给虚拟风扇的占空比
    // 命令通道的寄存器写（要用在 reg_file 之前，故提前声明）
    wire        cmd_wr_en;
    wire [5:0]  cmd_wr_addr;
    wire [31:0] cmd_wr_data;

    //------------------------------------------------------------------
    // 时钟与复位
    //------------------------------------------------------------------
    wire clk_sys, clk_core, clk_ref, locked, gear_ack, drp_err;
    wire rst_sys_n;
    wire [2:0] gear;

    clk_mgr u_clk_mgr (
        .clk_in  (clk),
        .rst_n   (rst_btn),
        .gear    (gear),
        .clk_sys (clk_sys),
        .clk_core(clk_core),
        .clk_ref (clk_ref),
        .locked  (locked),
        .gear_ack(gear_ack),
        .drp_err (drp_err)
    );

    reset_sync u_rst_sys (
        .clk    (clk_sys),
        .arst_n (locked & rst_btn),
        .rst_n  (rst_sys_n)
    );

    wire tick_ms, tick_us;

    tick_gen #(
        .CLK_HZ (`SYS_CLK_HZ),
        .ACCEL  (`SIM_ACCEL)
    ) u_tick (
        .clk     (clk_sys),
        .rst_n   (rst_sys_n),
        .tick_ms (tick_ms),
        .tick_us (tick_us)
    );

    //------------------------------------------------------------------
    // 感知
    //------------------------------------------------------------------
    wire signed [15:0] tj_raw_x;
    wire               tj_val_x;
    wire [11:0]        xadc_code;

    xadc_frontend u_xadc_fe (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .tick_ms   (tick_ms),
        .plant_sel (plant_sel),
        .vp_tj_uk  (vp_tj_uk),
        .vp_p_mw   (vp_p_mw),
        .tj_q8_8   (tj_raw_x),
        .tj_valid  (tj_val_x),
        .code_raw  (xadc_code)
    );

    wire signed [15:0] t_ext;
    wire [15:0]        rh_ext;
    wire               ext_val, ext_err;
    wire [15:0]        ext_err_cnt;

`ifdef SIM_MODEL
    localparam integer EXT_PERIOD_US = 1000;       // 仿真 1 ms（事务本身 5.5 ms，忙时自动跳过）
`else
    localparam integer EXT_PERIOD_US = 2_000_000;  // 硬件 2 s（器件要求）
`endif

    am2301_ctrl #(
        .CLK_HZ     (`SYS_CLK_HZ),
        .PERIOD_US  (EXT_PERIOD_US)
    ) u_am2301 (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .tick_us  (tick_us),
        .dq       (dht_dq),
        .t_q8_8   (t_ext),
        .rh_q8_8  (rh_ext),
        .valid    (ext_val),
        .err      (ext_err),
        .err_cnt  (ext_err_cnt)
    );

    //------------------------------------------------------------------
    // 采集调度
    //------------------------------------------------------------------
    wire signed [15:0] tj_s, text_s;
    wire [15:0]        rh_s, ext_age;
    wire [31:0]        ts_ms;
    wire               sample_valid;

    acq_fsm u_acq (
        .clk         (clk_sys),
        .rst_n       (rst_sys_n),
        .tick_ms     (tick_ms),
        .tj_in       (tj_raw_x),
        .text_in     (t_ext),
        .rh_in       (rh_ext),
        .text_new    (ext_val),
        .tj_out      (tj_s),
        .text_out    (text_s),
        .rh_out      (rh_s),
        .ext_age     (ext_age),
        .ts_ms       (ts_ms),
        .sample_valid(sample_valid)
    );

    //------------------------------------------------------------------
    // 处理：标定 -> 滑动平均 -> IIR
    //------------------------------------------------------------------
    wire signed [15:0] tj_cal, tj_ma, tj_filt;
    wire               v_cal, v_ma, v_iir;
    wire signed [15:0] text_cal, text_ma, text_filt;
    wire               vt_cal, vt_ma, vt_iir;

    wire [15:0] cal_a;
    wire signed [15:0] cal_b;

    calib u_cal_j (
        .clk        (clk_sys),
        .rst_n      (rst_sys_n),
        .valid      (sample_valid),
        .din        (tj_s),
        .gain_q4_12 (cal_a),
        .offset_q8_8(cal_b),
        .dout       (tj_cal),
        .dout_valid (v_cal)
    );

    calib u_cal_t (
        .clk        (clk_sys),
        .rst_n      (rst_sys_n),
        .valid      (sample_valid),
        .din        (text_s),
        .gain_q4_12 (16'h1000),
        .offset_q8_8(16'sd0),
        .dout       (text_cal),
        .dout_valid (vt_cal)
    );

    moving_avg #(.N(8), .SHIFT(3)) u_ma_j (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .valid     (v_cal),
        .din       (tj_cal),
        .dout      (tj_ma),
        .dout_valid(v_ma)
    );

    iir_lpf #(.K(2)) u_iir_j (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .valid     (v_ma),
        .din       (tj_ma),
        .dout      (tj_filt),
        .dout_valid(v_iir)
    );

    moving_avg #(.N(8), .SHIFT(3)) u_ma_t (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .valid     (vt_cal),
        .din       (text_cal),
        .dout      (text_ma),
        .dout_valid(vt_ma)
    );

    iir_lpf #(.K(2)) u_iir_t (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .valid     (vt_ma),
        .din       (text_ma),
        .dout      (text_filt),
        .dout_valid(vt_iir)
    );

    //------------------------------------------------------------------
    // 寄存器组
    //------------------------------------------------------------------
    wire [15:0] setpoint;
    wire [15:0] kp, ki;
    wire [15:0] th_warn, th_hot, th_crit;
    wire [2:0]  gear_man;
    wire [10:0] duty_man;
    wire [3:0]  control;
    wire [31:0] scratch;
    wire [10:0] duty;

    reg         rf_we;
    reg  [5:0]  rf_addr;
    reg  [31:0] rf_wdata;
    wire [31:0] rf_rdata;

    reg  [1:0]  sp_idx;
    reg  [3:0]  btn_d;
    wire [3:0]  btn_pressed = ~btn[3:0];          // EGO1 按键按下为低
    wire [3:0]  btn_pulse   = btn_pressed & ~btn_d;

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            sp_idx   <= 2'd1;
            btn_d    <= 4'd0;
            rf_we    <= 1'b0;
            rf_addr  <= 6'd0;
            rf_wdata <= 32'd0;
        end else begin
            btn_d <= btn_pressed;
            rf_we <= 1'b0;

            // btn0：设定值 40 / 50 / 60 循环
            if (btn_pulse[0]) begin
                sp_idx <= sp_idx + 2'd1;
                rf_we  <= 1'b1;
                rf_addr<= 6'h04;
                case (sp_idx)
                    2'd0: rf_wdata <= {16'd0, `T_40_Q8_8};
                    2'd1: rf_wdata <= {16'd0, `T_50_Q8_8};
                    default: rf_wdata <= {16'd0, `T_60_Q8_8};
                endcase
            end

            // btn1 / btn2：档位 +1 / -1
            if (btn_pulse[1]) begin
                rf_we   <= 1'b1;
                rf_addr <= 6'h05;
                rf_wdata<= {13'd0, (gear_man == 3'd4) ? 3'd4 : gear_man + 3'd1};
            end
            if (btn_pulse[2]) begin
                rf_we   <= 1'b1;
                rf_addr <= 6'h05;
                rf_wdata<= {13'd0, (gear_man == 3'd0) ? 3'd0 : gear_man - 3'd1};
            end
        end
    end

    wire [15:0] f_meas;
    wire [17:0] f_khz;
    wire        f_valid;
    wire [15:0] p_est;
    wire [15:0] switch_max_us;
    wire [15:0] sample_cnt;
    wire        alarm;
    wire [1:0]  state;
    wire        fan_max, gear_low, load_en;

    reg  [15:0] sw_timer;
    reg  [15:0] sw_max;
    reg  [2:0]  gear_d;

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            sw_timer <= 16'd0;
            sw_max   <= 16'd0;
            gear_d   <= 3'd0;
        end else begin
            gear_d <= gear;
            if (gear != gear_d)
                sw_timer <= 16'd0;
            else if (sw_timer != 16'hFFFF)
                sw_timer <= sw_timer + 16'd1;

            if (gear_ack && (sw_timer > sw_max))
                sw_max <= sw_timer;
        end
    end

    // 常数除法在 FPGA 里是"乘高位 + 修正"的长进位链，改乘加近似：/100 ≈ ×2621 >> 18
    // 单位仍是 0.1 µs；16 bit 够到 6553 µs
    // sw_max 是 clk_sys 拍数：1 拍 = 10 ns = 0.01 µs。
    // 原来 /100 取整到 1 µs，仿真里 50 拍 → 恒 0，断言形同虚设；
    // 改成 0.1 µs 单位（/10），16 bit 可表示到 6.55 ms，覆盖 1 ms 指标。
    wire [33:0] sw_div10 = sw_max * 18'd205;      // /10 ≈ ×205 >> 11
    assign switch_max_us = sw_div10[26:11];       // 单位 0.1 µs

    reg [15:0] samp_cnt;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)      samp_cnt <= 16'd0;
        else if (sample_valid) samp_cnt <= samp_cnt + 16'd1;
    end
    assign sample_cnt = samp_cnt;

    wire [35:0] f_mhz = (f_khz * 21'd2098) >> 21;  // /1000 ≈ ×2098 >> 21
    assign p_est = 16'd150 + f_mhz[15:0] * 16'd8;

    reg_file u_regs (
        .clk        (clk_sys),
        .rst_n      (rst_sys_n),
        .wr_en      (cmd_wr_en | rf_we),          // 命令通道优先于按键
        .wr_addr    (cmd_wr_en ? cmd_wr_addr : rf_addr),
        .wr_data    (cmd_wr_en ? cmd_wr_data : rf_wdata),
        .rd_addr    (6'd0),
        .rd_data    (rf_rdata),
        .setpoint   (setpoint),
        .kp         (kp),
        .ki         (ki),
        .th_warn    (th_warn),
        .th_hot     (th_hot),
        .th_crit    (th_crit),
        .cal_a      (cal_a),
        .cal_b      (cal_b),
        .gear_man   (gear_man),
        .duty_man   (duty_man),
        .control    (control),
        .scratch    (scratch),
        .tj_f       (tj_filt),
        .tj_raw     (tj_s),
        .text_f     (text_filt),
        .text_raw   (text_s),
        .rh         (rh_s),
        .ext_age    (ext_age),
        .duty_act   (duty),
        .gear_act   (gear),
        .state_act  (state),
        .f_meas     (f_meas),
        .p_est      (p_est),
        .code_raw   (xadc_code),
        .sample_cnt (sample_cnt),
        .err_cnt    (ext_err_cnt),
        .switch_max (switch_max_us)
    );

    //------------------------------------------------------------------
    // 决策：PI + 监督
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // 前馈：ΔT = Tset − 外部温度（两路模式下都可用：虚拟热台/AM2301）
    //   稳态所需占空比 ≈ FF_A − FF_B·ΔT，把 PI 从"爬山"里解放出来
    //------------------------------------------------------------------
    // AM2301 上电后要 5 ms 才出第一帧（仿真加速下等于几十秒物理时间），
    // 这段时间外部温度是 0，前馈会被算成"环境 0 ℃"而失效 → 先用 25 ℃ 占位
    reg ext_seen;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)      ext_seen <= 1'b0;
        else if (ext_val)    ext_seen <= 1'b1;
    end
    wire signed [15:0] ta_est = ext_seen ? text_filt : 16'sd6400;   // 25.00 ℃

    // 三级流水：ΔT 一拍 → 乘加一拍 → 限幅一拍。
    // 不分流水时"外部温度 → 前馈 → PI → 占空比"是一条 26 级逻辑的链（−8.7 ns）。
    reg  signed [16:0] ff_dt_r;
    reg  signed [31:0] ff_calc_r;
    reg         [10:0] ff_duty_r;
    wire signed [16:0] ff_dt   = $signed({1'b0, setpoint}) - $signed({1'b0, ta_est});
    wire signed [24:0] ff_mul  = $signed(ff_dt_r) * 8'sd40;    // 17×8，1 个 DSP
    wire signed [31:0] ff_calc = `VP_FF_A - {{7{ff_mul[24]}}, ff_mul[24:8]};
    wire        [10:0] ff_duty_c = (ff_calc_r >= 32'sd1000) ? 11'd1000 :
                                   (ff_calc_r <= 32'sd0)    ? 11'd0    : ff_calc_r[10:0];

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            ff_dt_r   <= 17'sd0;
            ff_calc_r <= 32'sd0;
            ff_duty_r <= 11'd0;
        end else begin
            ff_dt_r   <= ff_dt;
            ff_calc_r <= (ff_dt_r <= 0) ? 32'sd0 : ff_calc;
            ff_duty_r <= ff_duty_c;
        end
    end
    wire [10:0] ff_duty = ff_duty_r;

    wire [10:0] duty_pi;
    wire        pi_sat;

    pi_ctrl u_pi (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .update   (sample_valid),
        .enable   (control[3]),
        .tj_filt  (tj_filt),
        .setpoint (setpoint),
        .kp_q8_8  (kp),
        .ki_q8_8  (ki),
        .ff_duty  (ff_duty),
        .duty     (duty_pi),
        .saturated(pi_sat)
    );

    supervisor u_sup (
        .clk         (clk_sys),
        .rst_n       (rst_sys_n),
        .eval        (sample_valid),
        .tj          (tj_filt),
        .th_warn_q8_8(th_warn),
        .th_hot_q8_8 (th_hot),
        .th_crit_q8_8(th_crit),
        .clr_alarm   (btn[4]),
        .state       (state),
        .fan_max     (fan_max),
        .gear_low    (gear_low),
        .load_en     (load_en),
        .alarm       (alarm)
    );

    // 档位仲裁：保护 > 自动 DVFS 外环 > 手动；sw5 打开自动外环
    wire        auto_gear_en = sw[5] | control[2];
    wire [2:0]  gear_auto;
    wire [15:0] n_down, n_up;

    gear_ctrl u_gear (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .tick_ms  (tick_ms),
        .auto_en  (auto_gear_en),
        .prot     (gear_low),
        .duty     (duty),
        .tj       (tj_filt),
        .setpoint (setpoint),
        .gear_man (gear_man),
        .gear     (gear),
        .gear_auto(gear_auto),
        .n_down   (n_down),
        .n_up     (n_up)
    );

    assign duty = fan_max ? 11'd1000 :
                  (control[1] ? duty_man : duty_pi);

    // 执行器线性化：PI 的 0…1000 映射到风扇的 6%…100%，跳过 0–6% 的死区。
    // 不这么做，PI 输出落在 0…60 时风扇不转，闭环会出现一个"不可控空档"。
    // 注意：乘法必须放进 32 bit 上下文，否则 1000×940 会在 12 位里被截断（踩过）
    wire [20:0] duty_fan_mul = duty * 10'd940;                 // 11×10，无 DSP
    wire [10:0] duty_fan_c = (duty == 11'd0) ? 11'd0
                                             : (11'd60 + duty_fan_mul[20:10]);
    // 打一拍：这条 mux→乘法→风扇查表→惯性 的链跨了两个模块，单周期放不下
    reg  [10:0] duty_fan_r;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) duty_fan_r <= 11'd0;
        else            duty_fan_r <= duty_fan_c;
    end
    wire [10:0] duty_fan = duty_fan_r;
    assign vp_duty = duty_fan_r;

    pwm_gen u_pwm (
        .clk   (clk_sys),
        .rst_n (rst_sys_n),
        .duty  (duty_fan),
        .pwm   (fan_pwm)
    );

    //------------------------------------------------------------------
    // 执行：自热负载（core 域）+ 测频
    //------------------------------------------------------------------
    wire [15:0] act_cnt;

    heat_load_gen u_load (
        .clk    (clk_core),
        .rst_n  (rst_sys_n),
        .enable (load_en & control[3]),
        .level  (sw[3:0]),
        .act    (act_cnt)
    );

    freq_monitor #(.CLK_HZ(200_000_000)) u_fmon (
        .clk     (clk_ref),
        .rst_n   (rst_sys_n),
        .clk_test(clk_core),
        .f_khz   (f_khz),
        .valid   (f_valid)
    );

    wire [35:0] f_mul = f_khz * 18'd2621;          // /100 ≈ ×2621 >> 18
    assign f_meas = f_mul[35:18];                  // 0.1 MHz

    //------------------------------------------------------------------
    // 虚拟热台（plant_sel=1 时：虚拟风扇 + 虚拟功耗 + 虚拟环境 + 2R2C 芯片）
    //   设计原则：vplant 只在"器件接口层"接管（XADC 的 DRP、AM2301 的单总线），
    //   中间所有算法/控制/遥测 RTL 一行不改 → 真实与虚拟走同一条数据链，
    //   联调时哪一段出问题一望即知。
    //------------------------------------------------------------------
    wire [15:0] vp_rpm;
    wire        fan_fail;
    wire [15:0] fan_rpm, fan_fail_cnt;
    // 虚拟热台模式下用虚拟风扇的 tach，真实模式下用板上 tach（assign 放后面，声明顺序）
    wire        fan_tach_eff;
    wire        vp_tach, vp_stalled;
    wire [15:0] vp_inv_r2;
    wire [7:0]  vp_flow;
    wire [15:0] vp_p_dither;
    wire        vp_ac_on;
    wire signed [31:0] vp_drift;
    wire signed [31:0] vp_q12, vp_q2a;
    wire [39:0] vp_frame;
    wire        vp_frame_ok;
    wire [15:0] vp_conv_cnt, vp_err_inj;


    fan_vplant u_vfan (
        .clk          (clk_sys),
        .rst_n        (rst_sys_n),
        .tick_ms      (tick_ms),
        .duty         (vp_duty),
        .unit_trim_q8 (16'd256),
        .rpm          (vp_rpm),
        .tach         (vp_tach),
        .inv_r2_q12   (vp_inv_r2),
        .flow_q8      (vp_flow),
        .stalled      (vp_stalled)
    );

    assign fan_tach_eff = sw[6] ? vp_tach : fan_tach;

    fan_tach u_ftach (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .tick_ms  (tick_ms),
        .tach     (fan_tach_eff),
        .duty     (duty_fan),
        .rpm      (fan_rpm),
        .fan_fail (fan_fail),
        .fail_cnt (fan_fail_cnt)
    );

    power_model u_vpwr (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .tick_ms  (tick_ms),
        .f_khz    (f_khz),
        .level    (sw[3:0]),
        .p_mw     (vp_p_mw),
        .p_dither (vp_p_dither)
    );

    ambient_vplant u_vamb (
        .clk         (clk_sys),
        .rst_n       (rst_sys_n),
        .tick_ms     (tick_ms),
        .ac_toggle   (1'b0),          // 后续接寄存器位做"开空调"实验
        .force_en    (1'b0),
        .ta_force_uk (32'sd0),
        .ta_uk       (vp_ta_uk),
        .ac_on       (vp_ac_on),
        .drift_uk    (vp_drift)
    );

    thermal_core u_vth (
        .clk        (clk_sys),
        .rst_n      (rst_sys_n),
        .tick_ms    (tick_ms),
        .p_mw       ($signed(vp_p_mw)),
        .ta_uk      (vp_ta_uk),
        .inv_r2_q12 (vp_inv_r2),
        .t_init_uk  (vp_ta_uk),
        .tj_uk      (vp_tj_uk),
        .tc_uk      (vp_tc_uk),
        .q12_uw     (vp_q12),
        .q2a_uw     (vp_q2a)
    );

    am2301_vplant u_vsen (
        .clk        (clk_sys),
        .rst_n      (rst_sys_n),
        .tick_us    (tick_us),
        .en         (plant_sel),
        .ta_uk      (vp_ta_uk),
        .rh_pm      (16'd550),
        .dq         (dht_dq),
        .last_frame (vp_frame),
        .frame_ok   (vp_frame_ok),
        .conv_cnt   (vp_conv_cnt),
        .err_inj_cnt(vp_err_inj)
    );

    //------------------------------------------------------------------
    // 遥测
    //------------------------------------------------------------------
    reg  [15:0] seq;
    wire        tx_busy;
    wire        tx_start;
    wire [7:0]  tx_data;
    wire        fb_tx_start, cmd_tx_start;
    wire [7:0]  fb_tx_data, cmd_tx_data;
    wire        fb_busy, cmd_busy;
    reg  [2:0]  send_d;

    // 命令回复优先占用 UART（人敲命令的频率远低于 10 Hz 遥测）
    assign tx_start = cmd_busy ? cmd_tx_start : fb_tx_start;
    assign tx_data  = cmd_busy ? cmd_tx_data  : fb_tx_data;

    //------------------------------------------------------------------
    // 命令通道（ASCII，115200）：标定、改 Kp/Ki、清报警、读状态
    //------------------------------------------------------------------
    wire        rx_valid_cmd;
    wire [7:0]  rx_data_cmd;
    wire        rx_err_cmd;
    wire [15:0] cmd_n_cmd, cmd_n_err, cmd_n_cal;

    uart_rx #(.DIV(`UART_DIV)) u_uart_rx (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .rx        (uart_rx),
        .data      (rx_data_cmd),
        .valid     (rx_valid_cmd),
        .frame_err (rx_err_cmd)
    );

    cmd_proc u_cmd (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .rx_valid (rx_valid_cmd),
        .rx_byte  (rx_data_cmd),
        .tx_busy  (tx_busy | fb_busy),
        .tx_start (cmd_tx_start),
        .tx_data  (cmd_tx_data),
        .busy     (cmd_busy),
        .wr_en    (cmd_wr_en),
        .wr_addr  (cmd_wr_addr),
        .wr_data  (cmd_wr_data),
        .tj_f     (tj_filt),
        .setpoint (setpoint),
        .duty     (duty),
        .gear     (gear),
        .state    (state),
        .tj_raw   (tj_s),
        .cal_a    (cal_a),
        .cal_b    (cal_b),
        .n_cmd    (cmd_n_cmd),
        .n_err    (cmd_n_err),
        .n_cal    (cmd_n_cal)
    );


    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            send_d <= 3'd0;
            seq    <= 16'd0;
        end else begin
            send_d <= {send_d[1:0], sample_valid};
            if (send_d[2]) seq <= seq + 16'd1;
        end
    end

    frame_pack u_frame (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .send     (send_d[2] & ~cmd_busy),      // 命令回复期间丢弃这一帧，避免字节交织
        .seq_in   (seq),
        .ts_in    (ts_ms),
        .tj_in    (tj_filt),
        .text_in  (text_filt),
        .rh_in    (rh_s),
        .tset_in  (setpoint),
        .duty_in  (duty),
        .gear_in  (gear),
        .state_in (state),
        .f_in     (f_meas),
        .p_in     (p_est),
        .age_in   (ext_age[7:0]),
        .flags_in ({5'd0, fan_fail, drp_err, alarm}),
        .tx_start (fb_tx_start),
        .tx_data  (fb_tx_data),
        .tx_busy  (tx_busy | cmd_busy),
        .busy     (fb_busy)
    );

    uart_tx #(.DIV(`UART_DIV)) u_uart (
        .clk   (clk_sys),
        .rst_n (rst_sys_n),
        .start (tx_start),
        .data  (tx_data),
        .busy  (tx_busy),
        .tx    (uart_tx)
    );

    //------------------------------------------------------------------
    // 显示
    //------------------------------------------------------------------
    wire        text_mode = sw[7];
    // 显示必须用"标定后的原始值"而不是重滤波值：MA8 的窗口是 8×100 ms = 800 ms，
    // 用 tj_filt 显示会把"端到端延迟 ≤200 ms"直接击穿（docs/01 §2.4 本来也是这么定的）
    wire signed [15:0] disp_val = text_mode ? text_cal : tj_cal;
    wire [15:0] disp_abs = disp_val[15] ? (~disp_val + 16'd1) : disp_val;
    wire [15:0] t01 = (disp_abs * 16'd5) >> 7;        // 0.1 ℃ 单位

    // 十进制拆分：/10 ≈ ×205 >> 11，/100 ≈ ×2621 >> 18（避免常数除数位链）
    wire [27:0] d_mul10  = t01 * 11'd205;
    wire [15:0] t_div10  = d_mul10[26:11];               // t01/10
    wire [33:0] d_mul100 = t01 * 18'd2621;
    wire [15:0] t_div100 = d_mul100[33:18];              // t01/100
    wire [3:0]  d0 = (t01 - t_div10 * 16'd10);           // 个位（0.1 ℃）
    wire [3:0]  d1 = (t_div10 - t_div100 * 16'd10);      // 十位（℃ 个位）
    wire [3:0]  d2_raw = t_div100[3:0];
    wire [3:0]  d2 = (d2_raw > 4'd9) ? (d2_raw - 4'd10) : d2_raw;

    seg7_scan #(.DIV(100000)) u_seg (
        .clk   (clk_sys),
        .rst_n (rst_sys_n),
        .digits({4'hF, d2, d1, d0}),
        .dp    (4'b0010),
        .seg   (seg),
        .an    (an)
    );

    reg [23:0] heart;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) heart <= 24'd0;
        else            heart <= heart + 24'd1;
    end

    assign led[0]    = alarm;
    assign led[1]    = (state >= 2'd2);
    assign led[2]    = (state == 2'd3);
    assign led[3]    = ~locked;
    assign led[7:4]  = {1'b0, gear};
    assign led[7]    = fan_fail;          // 风扇堵转/断线指示
    assign led[15:8] = heart[23:16];

endmodule








