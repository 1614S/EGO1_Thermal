// ============================================================================
//  文件名 : tb_xadc_probe.sv
//  用途   : 为报告证据 E-01 生成 XADC 采集链波形（不参与功能回归）
//
//  为什么不是"改 tb_top_smoke 加 $dumpvars"：
//    本机 xsim 在 -debug / -gui / -tclbatch 模式下会崩：
//        Could not open file mapping object (2).
//        Abnormal program termination (EXCEPTION_ACCESS_VIOLATION)
//    而**不带 -debug 的普通 xsim -runall 完全正常**（三个 TB 都能跑）。
//    所以这里反过来做：**由 TB 自己用 $fdisplay 写标准 VCD 文本**，
//    不碰 xsim 的波形数据库。写出来的文件 sim/tools/vcd2svg.py 可直接渲染。
//
//  实现要点：
//    - 每个时钟比较被观察信号是否有变化，变化才写一行（否则文件会爆）
//    - 标量按 VCD 规范写成 "0#"（值紧贴标识符，无空格），向量写成 "b0101 &"
//    - 时间戳单位是 1 ns，与文件头的 $timescale 1ns 一致
//
//  观察对象（都在 ego1_thermal_top 内部）：
//    u_xadc_fe.st / daddr / den / drdy / drp_do / tj_q8_8 / tj_valid
//
//  运行（在 repo/work 下）：
//    xvlog -sv -d SIM_MODEL -i ../rtl/pkg -i ../sim/tb -f ../sim/filelist.f ^
//          ../sim/tb/tb_xadc_probe.sv
//    xelab tb_xadc_probe -s px_sim -timescale 1ns/1ps
//    xsim px_sim -runall
//    python ../sim/tools/vcd2svg.py --vcd probe_xadc.vcd --out ../docs/figs/E01_xadc_fsm.svg ^
//        --t-start 21230000 --t-end 21310000 --signal "st:digital" ...
//
//  注：本文件由 tb_top_smoke.sv 派生，激励部分保持一致；改动系统级 TB 时请同步。
// ============================================================================

//======================================================================
// tb_top_smoke.sv —— 顶层冒烟测试
// 检查：1) 能收到合法帧；2) CRC 全对；3) 结温字段与 XADC 模型一致。
//======================================================================
`timescale 1ns/1ps
`include "ego1_thermal_pkg.vh"

module tb_xadc_probe;

    reg clk = 1'b0;
    reg rst_btn = 1'b0;

    reg  [4:0] btn = 5'b1_1111;   // 按键低有效，空闲为全 1
    reg  [7:0] sw  = 8'h0F;

    wire [15:0] led;
    wire [7:0]  seg;
    wire [3:0]  an;
    wire        uart_tx;
    wire        fan_pwm;
    wire        dht_dq;

    // 弱上拉（xsim 对 pullup 原语支持不稳定，用带强度的连续赋值代替）
    assign (weak1, weak0) dht_dq = 1'b1;

    always #5 clk = ~clk;

    ego1_thermal_top u_dut (
        .clk     (clk),
        .rst_btn (rst_btn),
        .btn     (btn),
        .sw      (sw),
        .led     (led),
        .seg     (seg),
        .an      (an),
        .uart_tx (uart_tx),
        .uart_rx (1'b1),
        .fan_pwm (fan_pwm),
        .fan_tach(1'b1),
        .dht_dq  (dht_dq)
    );

    am2301_model u_sensor (.dq(dht_dq));

    //------------------------------------------------------------------
    // 串口接收
    //------------------------------------------------------------------
    wire [7:0] rx_byte;
    wire       rx_valid;

    uart_rx #(.DIV(`UART_DIV)) u_rx (
        .clk       (clk),
        .rst_n     (rst_btn),
        .rx        (uart_tx),
        .data      (rx_byte),
        .valid     (rx_valid),
        .frame_err ()
    );

    //------------------------------------------------------------------
    // 帧解析 + CRC 校验
    //------------------------------------------------------------------
    function [15:0] crc_step;
        input [15:0] c;
        input [7:0]  d;
        reg   [15:0] x;
        integer      k;
        begin
            x = c ^ {d, 8'h00};
            for (k = 0; k < 8; k = k + 1)
                x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
            crc_step = x;
        end
    endfunction

    reg [7:0]  frame [0:31];
    integer    bcnt = 0;
    integer    nframe = 0;
    integer    crc_err = 0;
    integer    hdr_err = 0;
    reg [15:0] crc_calc;
    reg signed [15:0] last_tj = 16'sd0;
    reg signed [15:0] last_text = 16'sd0;
    reg        done_d = 1'b0;
    integer    i;

    always @(posedge clk) begin
        done_d <= 1'b0;

        if (rx_valid) begin
            if (bcnt == 0) begin
                if (rx_byte == 8'hA5) bcnt <= 1;
                else hdr_err <= hdr_err + 1;
            end else if (bcnt == 1) begin
                if (rx_byte == 8'h5A) bcnt <= 2;
                else begin
                    bcnt    <= 0;
                    hdr_err <= hdr_err + 1;
                end
            end else begin
                frame[bcnt] <= rx_byte;
                if (bcnt == 31) begin
                    bcnt   <= 0;
                    done_d <= 1'b1;
                end else begin
                    bcnt <= bcnt + 1;
                end
            end
        end

        // 上一拍收满 32 字节，此拍做校验
        if (done_d) begin
            crc_calc = 16'hFFFF;
            for (i = 2; i <= 29; i = i + 1)
                crc_calc = crc_step(crc_calc, frame[i]);
            if ({frame[31], frame[30]} !== crc_calc) begin
                crc_err <= crc_err + 1;
                $display("[ERR] crc %04h != %04h", {frame[31], frame[30]}, crc_calc);
            end
            nframe  <= nframe + 1;
            last_tj <= {frame[11], frame[10]};
            last_text <= {frame[13], frame[12]};
        end
    end

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    initial begin
        // 等模型声明初始化完成后再赋值，避免 time-0 竞争
        #1ns;
        u_sensor.t_c = 40.0;
        u_sensor.rh  = 55.0;
`ifdef SIM_MODEL
        u_dut.u_xadc_fe.u_xadc.tj_c = 45.0;   // 行为模型才有的温度注入点
`else
        $display("[TB] WARNING: 未定义 SIM_MODEL，将使用真实 XADC/MMCM 原语");
`endif

        repeat (20) @(posedge clk);
        rst_btn = 1'b1;

        #4ms;
        btn = 5'b1_1101;          // 按下 btn1：档位 +1（3 -> 4，150 MHz）
        #40ns;
        btn = 5'b1_1111;

        #6ms;                    // 共 10 ms

        $display("---------------------------------------------");
        $display("[TB] frames=%0d  crc_err=%0d  hdr_err=%0d  tj=%0d (%.2f C) ext=%0d (%.2f C)",
                 nframe, crc_err, hdr_err, last_tj, last_tj/256.0, last_text, last_text/256.0);
        $display("[TB] f_meas=%0d  switch_max=%0d us  duty=%0d  seg=%02h an=%b",
                 u_dut.f_meas, u_dut.switch_max_us, u_dut.duty, seg, an);
        $display("[TB] sensor: t_c=%.1f rh=%.1f  am_err=%0d", u_sensor.t_c, u_sensor.rh, u_dut.u_am2301.err_cnt);

        if ((nframe >= 20) && (crc_err == 0) && (hdr_err == 0) &&
            (last_tj > 16'sd11000) && (last_tj < 16'sd11900) &&
            (u_dut.f_meas > 16'd1470) && (u_dut.f_meas < 16'd1530) &&
            (u_dut.switch_max_us < 16'd1000)) begin
            $display("TEST PASSED (tb_top_smoke)");
            if (last_text == 16'sd0)
                $display("NOTE: AM2301 通道仍未输出有效值（已知问题，见 README）");
        end else begin
            $display("TEST FAILED (tb_top_smoke)");
            $fatal(1, "top smoke failed");
        end
        $display("---------------------------------------------");
        $finish;
    end

    // ==================================================================
    //  探针：自己写 VCD 文本，不依赖 xsim 的 dump（本机 -debug/-gui 会崩）
    //  原理：每个时钟检查被观察信号是否变化，变化才写一行。
    //        写出来的就是标准 VCD，sim/tools/vcd2svg.py 可直接渲染。
    // ==================================================================
    integer fd_x;      // XADC 采集
    integer fd_d;      // DRP 切档
    reg [2:0]  x_st_q;   reg [6:0]  x_ad_q;
    reg        x_den_q;  reg        x_drdy_q;
    reg [15:0] x_do_q;   reg [15:0] x_tj_q;
    reg        x_vld_q;
    reg [2:0]  d_st_q;   reg [2:0]  d_gear_q;
    reg        d_den_q;  reg [6:0]  d_ad_q;
    reg [15:0] d_di_q;   reg        d_drdy_q;
    reg        d_ce_q;   reg        d_rst_q;
    reg [15:0] d_seq_q;
    reg        x_first = 1'b1;
    reg        d_first = 1'b1;

    initial begin
        fd_x = $fopen("probe_xadc.vcd", "w");
        $fdisplay(fd_x, "$timescale 1ns $end");
        $fdisplay(fd_x, "$scope module px $end");
        $fdisplay(fd_x, "$var wire 3 ! st $end");
        $fdisplay(fd_x, "$var wire 1 # den $end");
        $fdisplay(fd_x, "$var wire 7 $ daddr $end");
        $fdisplay(fd_x, "$var wire 1 %% drdy $end");
        $fdisplay(fd_x, "$var wire 16 & do $end");
        $fdisplay(fd_x, "$var wire 16 ' tj $end");
        $fdisplay(fd_x, "$var wire 1 ( vld $end");
        $fdisplay(fd_x, "$upscope $end");
        $fdisplay(fd_x, "$enddefinitions $end");

        fd_d = $fopen("probe_drp.vcd", "w");
        $fdisplay(fd_d, "$timescale 1ns $end");
        $fdisplay(fd_d, "$scope module pd $end");
        $fdisplay(fd_d, "$var wire 3 ! st $end");
        $fdisplay(fd_d, "$var wire 3 # gear $end");
        $fdisplay(fd_d, "$var wire 1 $ den $end");
        $fdisplay(fd_d, "$var wire 7 %% addr $end");
        $fdisplay(fd_d, "$var wire 16 & di $end");
        $fdisplay(fd_d, "$var wire 1 ' drdy $end");
        $fdisplay(fd_d, "$var wire 1 ( ce $end");
        $fdisplay(fd_d, "$var wire 1 ) rst_n $end");
        $fdisplay(fd_d, "$var wire 16 * seq $end");
        $fdisplay(fd_d, "$upscope $end");
        $fdisplay(fd_d, "$enddefinitions $end");
    end

    // ---- XADC 采集链 ----
    always @(posedge clk) begin
        if (x_first ||
            u_dut.u_xadc_fe.st       !== x_st_q   ||
            u_dut.u_xadc_fe.den      !== x_den_q  ||
            u_dut.u_xadc_fe.daddr    !== x_ad_q   ||
            u_dut.u_xadc_fe.drdy     !== x_drdy_q ||
            u_dut.u_xadc_fe.drp_do   !== x_do_q   ||
            u_dut.u_xadc_fe.tj_q8_8  !== x_tj_q   ||
            u_dut.u_xadc_fe.tj_valid !== x_vld_q) begin
            $fdisplay(fd_x, "#%0t", $time);
            $fdisplay(fd_x, "b%b !", u_dut.u_xadc_fe.st);
            $fdisplay(fd_x, "%b#",    u_dut.u_xadc_fe.den);
            $fdisplay(fd_x, "b%b $",  u_dut.u_xadc_fe.daddr);
            $fdisplay(fd_x, "%b%%",   u_dut.u_xadc_fe.drdy);
            $fdisplay(fd_x, "b%b &",  u_dut.u_xadc_fe.drp_do);
            $fdisplay(fd_x, "b%b '",  u_dut.u_xadc_fe.tj_q8_8);
            $fdisplay(fd_x, "%b(",    u_dut.u_xadc_fe.tj_valid);
            x_st_q <= u_dut.u_xadc_fe.st;     x_den_q  <= u_dut.u_xadc_fe.den;
            x_ad_q <= u_dut.u_xadc_fe.daddr;  x_drdy_q <= u_dut.u_xadc_fe.drdy;
            x_do_q <= u_dut.u_xadc_fe.drp_do; x_tj_q   <= u_dut.u_xadc_fe.tj_q8_8;
            x_vld_q <= u_dut.u_xadc_fe.tj_valid;
            x_first <= 1'b0;
        end
    end


endmodule
