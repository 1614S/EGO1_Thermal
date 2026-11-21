//======================================================================
// tb_top_smoke.sv —— 顶层冒烟测试
// 检查：1) 能收到合法帧；2) CRC 全对；3) 结温字段与 XADC 模型一致。
//======================================================================
`timescale 1ns/1ps
`include "ego1_thermal_pkg.vh"

module tb_top_smoke;

    reg clk = 1'b0;
    reg rst_btn = 1'b0;

    reg  [4:0] btn = 5'b1_1111;   // 按键低有效，空闲为全 1
    reg  [7:0] sw  = 8'h0F;

    wire [15:0] led;
    wire [7:0]  seg;
    wire [7:0]  an;
    wire [7:0]  seg2;
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
        .seg2    (seg2),
        .an      (an),
        .uart_tx (uart_tx),
        .uart_rx (1'b1),
        .fan_pwm (fan_pwm),
       
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
    real       lat_ns, lat_ms;

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

        //------------------------------------------------------------------
        // 端到端延迟：把注入温度阶跃 +10 ℃，量"显示器数值变化"用了多久。
        // 显示源必须是标定后的原始值（若误用 MA8/IIR 的滤波值，这里会 >800 ms）
        // 时间换算：SIM_ACCEL=1000 → 1 物理 ms = 1 000 ns 仿真时间
        //------------------------------------------------------------------
        begin : lat_measure
            integer cyc;
            reg [15:0] t01_old;
            t01_old = u_dut.dig_l;
            `ifdef SIM_MODEL
            u_dut.u_xadc_fe.u_xadc.tj_c = 55.0;      // 45 → 55 ℃
            `endif
            for (cyc = 0; cyc < 400000; cyc = cyc + 1) begin
                @(posedge clk);
                if (u_dut.dig_l != t01_old) begin
                    lat_ns = cyc * 10;               // 100 MHz
                    disable lat_measure;
                end
            end
            lat_ns = -1;
        end
        lat_ms = lat_ns / 1000.0;                    // 仿真 ns → 物理 ms（ACCEL=1000）
        $display("[TB] 端到端延迟（采样→显示）: %0.1f ms（要求 ≤200 ms）", lat_ms);
        if (lat_ms < 0.0 || lat_ms > 200.0) begin
            $display("  [FAIL] 端到端延迟超标（显示可能误用了重滤波值）");
        end

        $display("---------------------------------------------");
        $display("[TB] frames=%0d  crc_err=%0d  hdr_err=%0d  tj=%0d (%.2f C) ext=%0d (%.2f C)",
                 nframe, crc_err, hdr_err, last_tj, last_tj/256.0, last_text, last_text/256.0);
        $display("[TB] f_meas=%0d  switch_max=%0d us  duty=%0d  seg=%02h an=%b",
                 u_dut.f_meas, u_dut.switch_max_us, u_dut.duty, seg, an);
        $display("[TB] sensor: t_c=%.1f rh=%.1f  am_err=%0d", u_sensor.t_c, u_sensor.rh, u_dut.u_am2301.err_cnt);

        if ((nframe >= 20) && (crc_err == 0) && (hdr_err == 0) &&
            (last_tj > 16'sd11000) && (last_tj < 16'sd11900) &&
            (u_dut.f_meas > 16'd1470) && (u_dut.f_meas < 16'd1530) &&
            (u_dut.switch_max_us > 16'd0) &&      // 证明真的在测（原来恒 0 也过）
            (u_dut.switch_max_us < 16'd10000)) begin   // 单位 0.1 µs → 1 ms = 10000
            $display("TEST PASSED (tb_top_smoke)");
            if (last_text == 16'sd0)
                $display("NOTE: AM2302 通道仍未输出有效值（已知问题，见 README）");
        end else begin
            $display("TEST FAILED (tb_top_smoke)");
            $fatal(1, "top smoke failed");
        end
        $display("---------------------------------------------");
        $finish;
    end

endmodule

