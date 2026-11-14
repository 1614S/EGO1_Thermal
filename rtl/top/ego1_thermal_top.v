//======================================================================
// ego1_thermal_top.v —— 顶层（只连线，不写算法）
//
// 数据链：XADC/AM2301 -> acq_fsm -> calib -> MA8 -> IIR -> PI -> PWM -> 风扇
// 遥测  ：sample -> frame_pack -> uart_tx
// 执行  ：MMCM DRP 切档 + 自热负载
//======================================================================
`include "ego1_thermal_pkg.vh"

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
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .tick_ms  (tick_ms),
        .tj_q8_8  (tj_raw_x),
        .tj_valid (tj_val_x),
        .code_raw (xadc_code)
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

    assign switch_max_us = sw_max / 16'd100;

    reg [15:0] samp_cnt;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)      samp_cnt <= 16'd0;
        else if (sample_valid) samp_cnt <= samp_cnt + 16'd1;
    end
    assign sample_cnt = samp_cnt;

    assign p_est = 16'd150 + (f_khz / 18'd1000) * 16'd8;

    reg_file u_regs (
        .clk        (clk_sys),
        .rst_n      (rst_sys_n),
        .wr_en      (rf_we),
        .wr_addr    (rf_addr),
        .wr_data    (rf_wdata),
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

    assign gear = gear_low ? 3'd0 : gear_man;

    assign duty = fan_max ? 11'd1000 :
                  (control[1] ? duty_man : duty_pi);

    pwm_gen u_pwm (
        .clk   (clk_sys),
        .rst_n (rst_sys_n),
        .duty  (duty),
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

    assign f_meas = f_khz / 18'd100;      // 0.1 MHz

    //------------------------------------------------------------------
    // 遥测
    //------------------------------------------------------------------
    reg  [15:0] seq;
    wire        tx_busy;
    wire        tx_start;
    wire [7:0]  tx_data;
    reg  [2:0]  send_d;

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
        .send     (send_d[2]),
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
        .flags_in ({6'd0, drp_err, alarm}),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx_busy  (tx_busy),
        .busy     ()
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
    wire signed [15:0] disp_val = text_mode ? text_filt : tj_filt;
    wire [15:0] disp_abs = disp_val[15] ? (~disp_val + 16'd1) : disp_val;
    wire [15:0] t01 = (disp_abs * 16'd5) >> 7;        // 0.1 ℃ 单位

    wire [3:0] d0 = t01 % 16'd10;
    wire [3:0] d1 = (t01 / 16'd10) % 16'd10;
    wire [3:0] d2 = (t01 / 16'd100) % 16'd10;

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
    assign led[15:8] = heart[23:16];

endmodule
