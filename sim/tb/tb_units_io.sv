`timescale 1ns/1ps
//======================================================================
// tb_units_io.sv —— 采集/组帧/UART 的模块级自检（三个独立 top）
//   tb_acq     : 100 ms 采样周期、ts_ms 递增、ext_age 单位（100 ms）
//   tb_frame   : 32 B 帧头/字段/CRC 覆盖范围 2–29
//   tb_uart115 : 真实 115200（DIV=868）回环 —— 之前只在 10 Mbps 下跑过
//======================================================================
`include "tb_common.svh"

//----------------------------------------------------------------------
module tb_acq;
    reg clk = 0, rst_n = 0, tick = 0;
    reg signed [15:0] tj = 16'sd12800, text = 16'sd6400;
    reg [15:0] rh = 16'd14080;
    reg tnew = 0;
    wire signed [15:0] tj_o, text_o;
    wire [15:0] rh_o, age;
    wire [31:0] ts;   // acq_fsm 的 ts_ms 是 32 bit，别写窄了
    wire sv;

    always #5 clk = ~clk;
    integer k = 0;
    always @(posedge clk) begin
        if (k == 9) begin k = 0; tick = 1'b1; end else begin k = k + 1; tick = 1'b0; end
    end

    acq_fsm u (.clk(clk), .rst_n(rst_n), .tick_ms(tick),
               .tj_in(tj), .text_in(text), .rh_in(rh), .text_new(tnew),
               .tj_out(tj_o), .text_out(text_o), .rh_out(rh_o), .ext_age(age),
               .ts_ms(ts), .sample_valid(sv));

    // 用时间戳差测采样周期（对应赛题"采样周期 ≤100 ms"），比数时钟更直观
    integer nsv = 0;
    reg [31:0] ts_prev = 32'd0;
    reg [31:0] per_ms = 32'd0;
    always @(posedge clk) begin
        if (sv) begin
            nsv = nsv + 1;
            if (nsv > 1) per_ms = ts - ts_prev;
            ts_prev = ts;
        end
    end

    initial begin
        `INIT_TB
        $display("---- tb_acq ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        // 一个采样周期 = SAMPLE_MS(100) × 10 拍 = 1000 拍
        repeat (1050) @(posedge clk);
        `CHK(age == 16'd1, "无新数据时 ext_age 应 +1/采样（100 ms 单位）")
        repeat (1050) @(posedge clk);
        `CHK(age == 16'd2, "第二个采样点后 ext_age 应为 2")
        // 给一个 text_new → 下一个采样点 age 归零
        tnew = 1'b1; @(posedge clk); tnew = 1'b0;
        repeat (1050) @(posedge clk);
        `CHK(age == 16'd0, "有新数据后 ext_age 应清零")
        `CHK(per_ms == 32'd100, "相邻样本 ts_ms 差应为 100 ms")
        `CHK(ts >= 16'd300, "ts_ms 应随时基递增")
        $display("  采样 %0d 次，周期 %0d ms，ts_ms=%0d，age=%0d", nsv, per_ms, ts, age);
        `DONE("tb_acq")
    end
endmodule

//----------------------------------------------------------------------
module tb_frame;
    reg clk = 0, rst_n = 0, send = 0;
    wire tx_start;
    wire [7:0] tx_data;
    wire busy;
    always #5 clk = ~clk;

    frame_pack u (
        .clk(clk), .rst_n(rst_n), .send(send),
        .seq_in(16'h1234), .ts_in(32'h0001_0203),
        .tj_in(16'sd12800), .text_in(16'sd6400), .rh_in(16'd14080),
        .tset_in(16'sd12800), .duty_in(11'd500), .gear_in(3'd3),
        .state_in(2'd0), .f_in(16'd1000), .p_in(16'd1200),
        .age_in(8'd0), .flags_in(8'd0),
        .tx_start(tx_start), .tx_data(tx_data), .tx_busy(1'b0), .busy(busy)
    );

    reg [7:0] cap [0:31];
    integer n = 0;
    always @(posedge clk) if (tx_start && n < 32) begin cap[n] = tx_data; n = n + 1; end

    function [15:0] crc_step;
        input [15:0] c; input [7:0] d;
        reg [15:0] x; integer i;
        begin
            x = c ^ {d, 8'h00};
            for (i = 0; i < 8; i = i + 1) x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
            crc_step = x;
        end
    endfunction

    reg [15:0] crc_c;
    integer i;
    initial begin
        `INIT_TB
        $display("---- tb_frame ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        send = 1'b1; @(posedge clk); send = 1'b0;
        repeat (300) @(posedge clk);

        `CHK(n == 32, "应发出 32 字节")
        `CHK(cap[0] == 8'hA5 && cap[1] == 8'h5A, "帧头 A5 5A")
        `CHK(cap[2] == 8'h01 && cap[3] == 8'h01, "版本/类型")
        `CHK(cap[4] == 8'h34 && cap[5] == 8'h12, "SEQ 小端")
        `CHK(cap[10] == 8'd0 && cap[11] == 8'h32, "TJ=12800 = 0x3200 小端")
        crc_c = 16'hFFFF;
        for (i = 2; i <= 29; i = i + 1) crc_c = crc_step(crc_c, cap[i]);
        `CHK({cap[31], cap[30]} == crc_c, "CRC16 覆盖字节 2–29")
        $display("  32 B OK，CRC=%04h", {cap[31], cap[30]});
        `DONE("tb_frame")
    end
endmodule

//----------------------------------------------------------------------
// 真实波特率：显式把 DIV 设成 868（115200 @100 MHz），与 SIM_MODEL 无关
//----------------------------------------------------------------------
module tb_uart115;
    reg clk = 0, rst_n = 0;
    reg [7:0] td = 0;
    reg start = 0;
    wire bz, line;
    wire [7:0] rd;
    wire rv, rerr;
    always #5 clk = ~clk;

    uart_tx #(.DIV(868)) u_tx (.clk(clk), .rst_n(rst_n), .start(start), .data(td), .busy(bz), .tx(line));
    uart_rx #(.DIV(868)) u_rx (.clk(clk), .rst_n(rst_n), .rx(line), .data(rd), .valid(rv), .frame_err(rerr));

    integer i, recv = 0, err = 0;
    always @(posedge clk) begin
        if (rv) begin
            recv = recv + 1;
            if (rd !== td) err = err + 1;
        end
        if (rerr) err = err + 1;
    end

    initial begin
        `INIT_TB
        $display("---- tb_uart115 (DIV=868, 115200 baud) ----");
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);
        for (i = 0; i < 256; i = i + 1) begin
            td = i[7:0]; start = 1'b1; @(posedge clk); start = 1'b0;
            while (bz) @(posedge clk);
            repeat (2) @(posedge clk);
        end
        repeat (868 * 12) @(posedge clk);
        $display("  回环 %0d/256 字节，错误 %0d（1 bit = 868 拍 = 8.68 µs → 115.2 kbaud）", recv, err);
        `CHK(recv == 256 && err == 0, "115200 回环应 256/256 无错")
        `DONE("tb_uart115")
    end
endmodule


