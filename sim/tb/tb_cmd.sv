`timescale 1ns/1ps
//======================================================================
// tb_cmd.sv —— 命令通道模块级自检
//   证明：ASCII 解析、寄存器写、两点标定算 a/b、非法命令回 ERR
//
// 标定算例（手算期望）：
//   点1：raw=12544(49.0℃)  ref=12800(50.0℃)
//   点2：raw=15104(59.0℃)  ref=15360(60.0℃)
//   a = Δref/Δraw = 2560/2560 = 1.000 → Q4.12 = 4096
//   b = ref1 − a·raw1 = 12800 − 12544 = 256 （+1.0 ℃）
//======================================================================
`include "tb_common.svh"

module tb_cmd;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        rx_valid = 0;
    reg  [7:0] rx_byte = 0;
    wire       tx_start, busy;
    wire [7:0] tx_data;
    wire       wr_en;
    wire [5:0] wr_addr;
    wire [31:0] wr_data;
    wire [15:0] n_cmd, n_err, n_cal;

    reg signed [15:0] tj_raw = 16'sd12544;
    reg signed [15:0] tj_f   = 16'sd12800;

    // 抓寄存器写
    reg [31:0] cap [0:7];
    integer nwr = 0;
    always @(posedge clk) begin
        if (rst_n && wr_en) begin
            cap[nwr % 8] = {2'd0, wr_addr, wr_data};
            nwr = nwr + 1;
        end
    end

    // 抓回复字节
    reg [7:0] rep [0:63];
    integer nrep = 0;
    always @(posedge clk) begin
        if (rst_n && tx_start) begin
            rep[nrep % 64] = tx_data;
            nrep = nrep + 1;
        end
    end

    cmd_proc u (
        .clk(clk), .rst_n(rst_n),
        .rx_valid(rx_valid), .rx_byte(rx_byte),
        .tx_busy(1'b0), .tx_start(tx_start), .tx_data(tx_data), .busy(busy),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data),
        .tj_f(tj_f), .setpoint(16'sd12800), .duty(11'd204), .gear(3'd3), .state(2'd0),
        .tj_raw(tj_raw), .cal_a(16'd4096), .cal_b(16'sd0),
        .n_cmd(n_cmd), .n_err(n_err), .n_cal(n_cal)
    );

    task send_str(input [8*8-1:0] s, input integer len);
        integer i;
        begin
            for (i = 0; i < len; i = i + 1) begin
                rx_byte  = s[8*(len-1-i) +: 8];
                rx_valid = 1'b1;
                @(posedge clk);
            end
            // 补一个 CR 作为命令结束符
            rx_byte  = 8'h0D;
            rx_valid = 1'b1;
            @(posedge clk);
            rx_valid = 1'b0;
            repeat (40) @(posedge clk);      // 等解析 + 回复
            while (busy) @(posedge clk);
            repeat (4) @(posedge clk);
        end
    endtask

    initial begin
        `INIT_TB
        $display("---- tb_cmd ----");
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        // ---- S=45.0 → 写 0x04 = 45*256 = 11520 ----
        nwr = 0;
        send_str("S=45.0", 6);
        `CHK(nwr >= 1, "S= 应产生一次寄存器写")
        `CHK(cap[0][37:32] == 6'h04, "S= 应写 SETPOINT(0x04)")
        `CHK_NEAR(cap[0][31:0], 11520.0, 40.0, "S=45.0 应得到 Q8.8 11520")

        // ---- 非法命令 → ERR ----
        nrep = 0;
        send_str("Q=1", 3);
        `CHK(n_err >= 1, "非法命令应计入 n_err")
        `CHK(nrep >= 3 && rep[0] == "E", "非法命令应回 ERR")

        // ---- ? → 状态行以 T= 开头 ----
        nrep = 0;
        send_str("?", 1);
        `CHK(nrep > 5, "? 应有回复")
        `CHK(rep[0] == "T" && rep[1] == "=", "? 的回复应以 T= 开头")
        $display("  状态回复：%c%c%c%c%c%c%c%c", rep[0], rep[1], rep[2], rep[3], rep[4], rep[5], rep[6], rep[7]);

        // ---- 标定系数写入（算式在主机脚本里算，硬件只落寄存器）----
        nwr  = 0;
        send_str("CA=1.0", 6);
        send_str("CB=0.5", 6);
        `CHK(nwr >= 2, "CA=/CB= 应各写一次寄存器")
        `CHK(cap[0][37:32] == 6'h0C, "CA= 应写 CAL_A(0x0C)")
        `CHK(cap[1][37:32] == 6'h0D, "CB= 应写 CAL_B(0x0D)")
        `CHK_NEAR(cap[0][15:0], 4096.0, 8.0, "CA=1.0 → Q4.12 应为 4096")
        `CHK_NEAR($signed(cap[1][15:0]), 128.0, 4.0, "CB=0.5 ℃ → Q8.8 应为 128")
        $display("  CA=1.0 → %0d   CB=0.5 → %0d", cap[0][15:0], $signed(cap[1][15:0]));

        `DONE("tb_cmd")
    end
endmodule
