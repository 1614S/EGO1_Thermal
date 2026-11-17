// ============================================================================
//  文件名 : tb_frame_bytes.sv
//  用途   : 验证「上位机解码器」与「RTL 真实帧格式」是否一致（不需要板子）
//
//  为什么需要这个：
//    sw/host_logger.py 目前只用**自己造的帧**验证过（Python 造帧 → Python 解帧），
//    这不构成闭环 —— 如果 RTL 的字节序、CRC 覆盖范围、duty 的位拼接与我的理解
//    不一致，上板那天会表现为「一帧都收不到」，而且现场很难查。
//
//    这个 TB 把 frame_pack 真实吐出的 32 字节抓下来写成二进制文件，
//    再让 host_logger 的解码器去读 —— 这才叫 RTL ↔ 上位机 联调。
//
//  做法：
//    tx_busy 恒为 0（假设 UART 永远有空），于是 tx_start 每个字节脉冲一次；
//    在 tx_start 那一拍用 $fwrite("%c") 把 tx_data 写进 frame_bytes.bin。
//
//  运行（work 目录）：
//    xvlog ../rtl/telemetry/frame_pack.v ../rtl/pkg/ego1_thermal_pkg.vh 不需要
//    xvlog -i ../rtl/pkg ../rtl/telemetry/frame_pack.v tb_frame_bytes.sv
//    xelab tb_frame_bytes -s fb_sim -timescale 1ns/1ps
//    xsim fb_sim -runall
// ============================================================================
`timescale 1ns / 1ps
`include "ego1_thermal_pkg.vh"

module tb_frame_bytes;

    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;

    reg               send   = 1'b0;
    reg  [15:0]       seq_in = 16'd0;
    reg  [31:0]       ts_in  = 32'd0;
    reg  signed [15:0] tj_in = 16'sd0;
    reg  signed [15:0] text_in = 16'sd0;
    reg  [15:0]       rh_in  = 16'd0;
    reg  signed [15:0] tset_in = 16'sd0;
    reg  [10:0]       duty_in = 11'd0;
    reg  [2:0]        gear_in = 3'd0;
    reg  [1:0]        state_in = 2'd0;
    reg  [15:0]       f_in    = 16'd0;
    reg  [15:0]       p_in    = 16'd0;
    reg  [7:0]        age_in  = 8'd0;
    reg  [7:0]        flags_in = 8'd0;

    wire              tx_start;
    wire [7:0]        tx_data;

    frame_pack u_frame (
        .clk     (clk), .rst_n  (rst_n),
        .send    (send),
        .seq_in  (seq_in), .ts_in  (ts_in), .tj_in (tj_in), .text_in (text_in),
        .rh_in   (rh_in),  .tset_in (tset_in), .duty_in (duty_in),
        .gear_in (gear_in), .state_in (state_in), .f_in (f_in), .p_in (p_in),
        .age_in  (age_in), .flags_in (flags_in),
        .tx_start(tx_start), .tx_data (tx_data),
        .tx_busy (1'b0),                 // UART 永远有空
        .busy    ()
    );

    integer fd;
    integer nbyte = 0;

    initial fd = $fopen("frame_bytes.bin", "wb");

    // tx_start 是一拍脉冲，此时 tx_data 有效
    always @(posedge clk) begin
        if (tx_start) begin
            $fwrite(fd, "%c", tx_data);
            nbyte = nbyte + 1;
        end
    end

    task send_one(input [15:0] seq, input [31:0] ts,
                  input signed [15:0] tj, input signed [15:0] text,
                  input [15:0] rh, input signed [15:0] tset,
                  input [10:0] duty, input [2:0] gear, input [1:0] state,
                  input [15:0] fmhz, input [15:0] pmw,
                  input [7:0] age, input [7:0] flags);
        begin
            @(posedge clk);
            seq_in <= seq; ts_in <= ts; tj_in <= tj; text_in <= text;
            rh_in <= rh; tset_in <= tset; duty_in <= duty;
            gear_in <= gear; state_in <= state;
            f_in <= fmhz; p_in <= pmw; age_in <= age; flags_in <= flags;
            send <= 1'b1;
            @(posedge clk);
            send <= 1'b0;
            // 等这一帧发完（33 个字节左右）
            repeat (200) @(posedge clk);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        // 帧 1：正常值（与 host_logger 自检用的那组一致）
        send_one(16'd7, 32'd123456,
                 16'sd11530,   // 45.0390625 ℃  (45.04*256 = 11530.24)
                 16'sd10237,   // 39.98828125 ℃
                 16'd551,      // 55.1 %RH
                 16'sd12800,   // 50.0 ℃
                 11'd592, 3'd3, 2'd0, 16'd1000, 16'd1200, 8'd2, 8'd0);

        // 帧 2：负温度 + 低档位，专门验补码与 duty 位拼接
        send_one(16'd8, 32'd1,
                 -16'sd3200,   // -12.5 ℃
                 -16'sd832,    // -3.25 ℃
                 16'd400, 16'sd10240,
                 11'd7, 3'd0, 2'd3, 16'd250, 16'd600, 8'd0, 8'd16);

        repeat (200) @(posedge clk);
        $display("[FRAME] 抓取字节数 = %0d（期望 64 = 2 帧 × 32）", nbyte);
        $fclose(fd);
        $finish;
    end

endmodule
