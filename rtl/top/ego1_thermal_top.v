`timescale 1ns/1ps
//======================================================================
// ego1_thermal_top.v —— 顶层（只连线，不写算法）
//
// 数据链：XADC/AM2302 -> acq_fsm -> calib -> MA8 -> IIR -> PI -> PWM -> 风扇
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
    output wire [7:0]  seg,          // 左 4 位（第 0 组）段选
    output wire [7:0]  seg2,         // 右 4 位（第 1 组）段选
    output wire [7:0]  an,           // 低 4 位扫描，高 4 位恒灭
    output wire        uart_tx,
    input  wire        uart_rx,
    output wire        fan_pwm,
    output wire        heat_pwm,       // 环境加热电阻 PWM（B18 / J5-7）
    inout  wire        dht_dq
);

    //------------------------------------------------------------------
    // 虚拟热台与真实器件的选择：sw[6]=1 用虚拟热台，=0 用真实 XADC/AM2302
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
    wire [15:0]        ext_err_cnt, ext_crc_cnt;

`ifdef SIM_MODEL
    localparam integer EXT_PERIOD_US = 1000;       // 仿真 1 ms（事务本身 5.5 ms，忙时自动跳过）
`else
    localparam integer EXT_PERIOD_US = 3_000_000;  // 硬件 3 s（器件 ≥2 s；实测 2.5 s 仍会锁死总线，再放宽）
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
        .err_cnt  (ext_err_cnt),
        .crc_err_cnt (ext_crc_cnt)
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
    wire [3:0]  btn_pressed = ~btn[3:0];          // EGO1 按键按下为低（S0~S3）
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
    // 前馈：ΔT = Tset − 外部温度（两路模式下都可用：虚拟热台/AM2302）
    //   稳态所需占空比 ≈ FF_A − FF_B·ΔT，把 PI 从"爬山"里解放出来
    //------------------------------------------------------------------
    // AM2302 上电后要 5 ms 才出第一帧（仿真加速下等于几十秒物理时间），
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
    //------------------------------------------------------------------
    // 前馈要跟着实际功耗走。
    // 原来的 u_ff = 1528 − 40·(sp − Ta) 是按 level15/150MHz（≈1.35 W）标定的；
    // 一旦自动调频把档位降下来（例如 750 mW），同样的温升只需要更小的风量，
    // 前馈却仍按 1.35 W 给，积分只能往负里压 —— 上板实测 60→40 ℃ 时停在
    // 38.5 ℃（差 1.54 ℃）。这里按实测功耗对前馈做线性折算。
    //   scale = p_est / 1350，Q8 表示（256 = 1.0），限幅 0.25…1.25
    //------------------------------------------------------------------
    wire [31:0] ff_scale_raw = p_est * 32'd49;            // 49/256 ≈ 1/5.22 → 即 p/1350 的 Q8
    reg  [8:0]  ff_scale_q8;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) ff_scale_q8 <= 9'd256;
        else            ff_scale_q8 <= (ff_scale_raw[16:8] > 9'd320) ? 9'd320
                                      : (ff_scale_raw[16:8] < 9'd64)  ? 9'd64
                                      :  ff_scale_raw[16:8];
    end

    wire        [10:0] ff_base_c = (ff_calc_r >= 32'sd1000) ? 11'd1000 :
                                   (ff_calc_r <= 32'sd0)    ? 11'd0    : ff_calc_r[10:0];
    wire        [19:0] ff_sc_mul = ff_base_c * ff_scale_q8;   // 11×9
    wire        [10:0] ff_duty_c = (ff_sc_mul[18:8] > 11'd1000) ? 11'd1000 : ff_sc_mul[18:8];

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

    // 结温环沿用饱和-PI 混合算法：饱和区（|e|>2 K）靠积分分离把积分冻结，
    // 否则 100 % 期间积分会一直累积，切回 PI 时要很久才能退饱和。
    pi_ctrl #(.E_ILIM(512)) u_pi (
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
        .clr_alarm   (1'b0),             // 清报警移到串口 R 命令；S4 改为切环境温度设定值
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

    //------------------------------------------------------------------
    // 结温环：饱和逼近 + PI 精细调控（阈值 2 ℃，迟滞 3 ℃）
    //   e = Tj − Tset，e>0 过热 → 100 %；e<0 过冷 → 0 %
    //   与加热环同一套思路：
    //     · |e| ≤ 2 K        → PI 精细调节
    //     · |e| > 3 K        → 饱和
    //     · 2 K < |e| ≤ 3 K  → 保持原模式（迟滞，防止在边界反复切换）
    //   输出经斜率限制（100 duty/100 ms，满量程 1 s），切换时风扇不会突变。
    //------------------------------------------------------------------
    wire signed [16:0] fan_err  = $signed(tj_filt) - $signed({1'b0, setpoint});
    wire signed [16:0] fan_aerr = fan_err[16] ? (17'sd0 - fan_err) : fan_err;

    reg fan_bang;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)        fan_bang <= 1'b1;                 // 上电先饱和逼近
        else if (sample_valid) begin
            // 迟滞方向：误差**缩小**到 2 K 才退出饱和；涨过 3 K 才回到饱和。
            //   （反过来的话：退出阈值 3 K > 回退阈值 2 K，误差停在 2~3 K 会反复切换）
            if (fan_bang) fan_bang <= (fan_aerr > 17'sd512);     // 进入 2 K → 交给 PI
            else          fan_bang <= (fan_aerr > 17'sd768);     // 涨过 3 K → 回饱和
        end
    end

    wire [10:0] fan_target = fan_bang ? (fan_err[16] ? 11'd0 : 11'd1000) : duty_pi;

    reg [10:0] fan_duty_hy;
    wire [11:0] fan_tgt12 = {1'b0, fan_target};
    wire [11:0] fan_up12  = {1'b0, fan_duty_hy} + 12'd100;
    wire [11:0] fan_dn12  = ({1'b0, fan_duty_hy} > 12'd100) ?
                            ({1'b0, fan_duty_hy} - 12'd100) : 12'd0;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)        fan_duty_hy <= 11'd0;
        else if (sample_valid) begin
            if      (fan_tgt12 > fan_up12) fan_duty_hy <= fan_up12[10:0];
            else if (fan_tgt12 < fan_dn12) fan_duty_hy <= fan_dn12[10:0];
            else                           fan_duty_hy <= fan_target;
        end
    end

    assign duty = fan_max ? 11'd1000 :
                  (control[1] ? duty_man : fan_duty_hy);

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
    // 环境温度闭环（AM2302 → 加热电阻 PWM，B18/J5-7）
    //   设定值：命令 SE=xx.x 写入寄存器 0x0E（Q8.8），默认 40.0 ℃
    //   增益：先给一组保守常数（环境对象比结温慢、增益小），需要时再改成命令可调
    //   安全：告警/超温/外部数据失效/时钟未锁定时强制 0，避免"故障反而猛加热"
    //------------------------------------------------------------------
    reg signed [15:0] setpoint_ext;
    reg        [1:0]  se_idx;

    // S4（btn[4]）独立边沿检测：原有 btn_pressed/btn_pulse 只有 4 位（S0~S3），
    // 直接引用 btn_pulse[4] 是越界的（上板表现：按 S4 没反应）。
    reg s4_d, s4_armed;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            s4_d     <= 1'b1;                    // 按键低有效，空闲为高
            s4_armed <= 1'b0;
        end else begin
            s4_d <= btn[4];
            if (btn[4]) s4_armed <= 1'b1;        // 上电保护：先看到松开，才允许计数
        end
    end
    wire s4_press = s4_armed & s4_d & ~btn[4];   // 高→低 = 按下

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            setpoint_ext <= 16'sd10240;                          // 40.0 ℃
            se_idx       <= 2'd0;
        end else if (s4_press) begin                             // S4：40 → 50 → 60 → 40
            se_idx       <= (se_idx == 2'd2) ? 2'd0 : se_idx + 2'd1;
            setpoint_ext <= (se_idx == 2'd0) ? `T_50_Q8_8 :
                            (se_idx == 2'd1) ? `T_60_Q8_8 : `T_40_Q8_8;
        end else if (cmd_wr_en && cmd_wr_addr == 6'h0E) begin
            setpoint_ext <= cmd_wr_data[15:0];                   // SE= 命令优先
        end
    end

    wire        heater_en = locked && (state != 2'd3) && (ext_age < 16'd100)
                            && (text_filt < 16'sd19200);       // < 75 ℃ 才允许加热
    wire [10:0] duty_ext;
    wire        ext_bang;        // 1 = 加热环处于饱和区（输出 0/100 %）
    wire        ext_sat;

    ext_loop u_ext (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .update   (sample_valid),
        .t_ext    (text_filt),
        .t_room   (16'sd6400),       // 室温参考 25.0 ℃（前馈基准；未单独测室温时的保守假设）
        .sp_ext   (setpoint_ext),
        .kp_q8_8  (16'd512),         // Kp = 2.0 duty/℃（对象增益仅 0.25 K/duty，不能大）
        .ki_q8_8  (16'd5),           // Ki = 0.02 duty/(℃·样本)；配合积分分离(2 K)防超调
        .heater_en(heater_en),
        .duty     (duty_ext),
        .bang     (ext_bang),
        .saturated(ext_sat)
    );

    pwm_gen u_pwm_heat (
        .clk   (clk_sys),
        .rst_n (rst_sys_n),
        .duty  (duty_ext),
        .pwm   (heat_pwm)
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
    //   设计原则：vplant 只在"器件接口层"接管（XADC 的 DRP、AM2302 的单总线），
    //   中间所有算法/控制/遥测 RTL 一行不改 → 真实与虚拟走同一条数据链，
    //   联调时哪一段出问题一望即知。
    //------------------------------------------------------------------
    wire [15:0] vp_rpm;
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
    //
    // 仲裁要用 busy | tx_start：cmd_proc 在同一拍同时把 busy 和 tx_start 置 1，
    // 只判寄存后的 busy 会漏掉回复的第一个字节（上板实测回复变短/乱码）。
    wire cmd_sel = cmd_busy | cmd_tx_start;
    assign tx_start = cmd_sel ? cmd_tx_start : fb_tx_start;
    assign tx_data  = cmd_sel ? cmd_tx_data  : fb_tx_data;

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
        .tset_ext (setpoint_ext),
        .duty     (duty),
        .duty_heat(duty_ext),
        .gear     (gear),
        .state    (state),
        .tj_raw   (tj_s),
        .text_f   (text_filt),
        .cal_a    (cal_a),
        .cal_b    (cal_b),
        .ext_err_cnt (ext_err_cnt),
        .ext_crc_cnt (ext_crc_cnt),
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
        .tset_ext_in (setpoint_ext),            // 环境温度设定值（byte28-29）
        .duty_in  (duty),
        .duty_heat_in(duty_ext),                // 加热占空比（byte20/21 高位）
        .gear_in  (gear),
        .state_in (state),
        .f_in     (f_meas),
        .p_in     (p_est),
        .age_in   (ext_age[7:0]),
        .flags_in ({6'd0, drp_err, alarm}),          // bit2(堵转) 已随 TACH 删除，恒 0
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
    //   btn[3]（板上 V1）按一下翻一页：0=结温 Tj、1=设定值、2=PWM 占空比、3=外部温度
    //   sw[4]=1 仍显示自检图案 "1.234"
    //
    //   翻页键做 20 ms 去抖 + "必须先见到松开状态才允许翻页"：
    //   上电/重载比特流时 V1 若因按键电容或悬空被读成低电平，
    //   旧写法会当成一次按键把页翻走，默认页就不是结温了（上板遇到过）。
    //------------------------------------------------------------------
    reg [2:0]  pg_sync;
    reg        pg_stable, pg_prev, pg_armed;
    reg [20:0] pg_cnt;                                       // 21 位：20 ms @100 MHz = 2 000 000
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            pg_sync   <= 3'b000;                             // 初值按"低"算，
            pg_stable <= 1'b0;                               // 必须先看到松开（高）
            pg_prev   <= 1'b0;                               // 才 pg_armed=1
            pg_armed  <= 1'b0;
            pg_cnt    <= 21'd0;
        end else begin
            pg_sync <= {pg_sync[1:0], btn[3]};
            if (pg_sync[2] == pg_stable) begin
                pg_cnt <= 21'd0;
            end else if (pg_cnt >= 21'd2_000_000) begin      // 20 ms @100 MHz
                pg_stable <= pg_sync[2];
                pg_cnt    <= 21'd0;
            end else begin
                pg_cnt <= pg_cnt + 21'd1;
            end
            pg_prev <= pg_stable;
            if (pg_stable) pg_armed <= 1'b1;                 // 松开过一次才允许翻页
        end
    end
    wire pg_press = pg_armed & pg_prev & ~pg_stable;         // 按下沿

    reg [1:0] disp_page;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)                 disp_page <= 2'd0;   // 默认第 1 页
        else if (pg_press)              disp_page <= (disp_page == 2'd2) ? 2'd0 : disp_page + 2'd1;
    end

    //------------------------------------------------------------------
    // 8 位数码管：左 4 位 = 结温/设定值/风扇占空比，右 4 位 = 传感器温度/设定值/加热占空比
    //   第 1 页：实测结温 | 实测环境温度
    //   第 2 页：结温设定值 | 环境温度设定值
    //   第 3 页：风扇 PWM 占空比 | 加热 PWM 占空比
    //
    // 拆位全部用扩位中间量（踩过的坑：16 位上下文截断 → 51.2 ℃ 归零 / 占空比 A~F）
    //------------------------------------------------------------------
    // 温度 Q8.8 → 4 位 BCD（左1 空白、左2 十位、左3 个位、左4 小数），小数点放位 2
    function [15:0] to_tdisp(input signed [15:0] q);
        reg [15:0] a, t10, d10, t100, e0, e1;
        reg [33:0] m;
        begin
            a   = q[15] ? (~q + 16'd1) : q;
            m   = a * 21'd5;            // 21 位中间量
            t10 = m[19:7];              // 0.1 ℃
            m   = t10 * 11'd205;  d10  = m[26:11];
            m   = t10 * 18'd2622; t100 = m[33:18];
            e0  = t10 - d10  * 16'd10;
            e1  = d10 - t100 * 16'd10;
            to_tdisp = { e0[3:0], e1[3:0], t100[3:0], 4'hF };
        end
    endfunction

    // 占空比 0…1000 → 4 位 BCD（左1 千位、左2 百位、左3 十位、左4 个位），无小数点
    function [15:0] to_ddisp(input [10:0] d);
        reg [15:0] r, h, t, t10, o10;
        reg [3:0]  k;
        reg [21:0] m;
        begin
            k  = (d >= 11'd1000) ? 4'd1 : 4'd0;
            r  = {5'd0, d} - (k ? 16'd1000 : 16'd0);
            m  = r * 18'd2622; h   = m[21:18];
            m  = r * 11'd205;  t   = m[21:11];
            t10 = t - h * 16'd10;
            o10 = r - t * 16'd10;
            to_ddisp = { o10[3:0], t10[3:0], h[3:0], k };
        end
    endfunction

    wire [15:0] dig_l, dig_r;       // 左/右 4 位的 BCD
    wire [3:0]  dp_l, dp_r;

    assign dig_l = sw[4] ? 16'h4321 :
                   (disp_page == 2'd0) ? to_tdisp(tj_cal)      :
                   (disp_page == 2'd1) ? to_tdisp(setpoint)    :
                                         to_ddisp(duty);
    assign dig_r = sw[4] ? 16'h4321 :
                   (disp_page == 2'd0) ? to_tdisp(text_cal)    :
                   (disp_page == 2'd1) ? to_tdisp(setpoint_ext):
                                         to_ddisp(duty_ext);
    assign dp_l  = sw[4] ? 4'b0001 : ((disp_page == 2'd2) ? 4'b0000 : 4'b0100);
    assign dp_r  = sw[4] ? 4'b0001 : ((disp_page == 2'd2) ? 4'b0000 : 4'b0100);

    wire [3:0] an_lo, an_hi;

    seg7_scan #(.DIV(100000)) u_seg_lo (
        .clk   (clk_sys),
        .rst_n (rst_sys_n),
        .digits(dig_l),
        .dp    (dp_l),
        .seg   (seg),
        .an4   (an_lo)
    );

    seg7_scan #(.DIV(100000)) u_seg_hi (
        .clk   (clk_sys),
        .rst_n (rst_sys_n),
        .digits(dig_r),
        .dp    (dp_r),
        .seg   (seg2),
        .an4   (an_hi)
    );

    // an[3:0] = 第 0 组（左 4 位），an[7:4] = 第 1 组（右 4 位）
    assign an = {an_hi, an_lo};

    reg [23:0] heart;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) heart <= 24'd0;
        else            heart <= heart + 24'd1;
    end

    //------------------------------------------------------------------
    // LED 语义（state 编码：0=NORMAL 1=WARN 2=HOT 3=CRIT）
    //   led[0]  任意告警（WARN 起亮）
    //   led[1]  HOT 及以上
    //   led[2]  CRIT 及以上（只有 state==3 亮）
    //   led[3]  MMCM 未锁定（正常应灭）
    //   led[6:4] 当前档位（二进制 000~100）
    //   led[7]  风扇堵转/断线（无风扇时常亮；虚拟热台模式下应灭）
    //   led[15:8] 心跳
    // 仿真证据（.work/tb_sup_led.sv）：NORMAL 000 → WARN 100 → HOT 110 → CRIT 111
    //   → 降温+clr 回 000；supervisor 的 alarm 只在 CRIT 锁存，仍在遥测 flags bit0。
    //------------------------------------------------------------------
    assign led[0]    = (state != 2'd0);   // 任意告警：WARN/HOT/CRIT 都亮
    assign led[1]    = (state >= 2'd2);
    assign led[2]    = (state == 2'd3);
    assign led[3]    = ~locked;
    assign led[6:4]  = gear;              // 档位（3 bit）；led[7] 单独给风扇故障
    assign led[7]    = ext_sat;           // 加热输出饱和指示（原 TACH 堵转灯已删）
    assign led[15:8] = heart[23:16];

endmodule









