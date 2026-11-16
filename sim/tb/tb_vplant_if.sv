`timescale 1ns/1ps
//======================================================================
// tb_vplant_if.sv —— 虚拟器件的"接口级"自检
//
// 证明两件事（这是整个虚拟热台能不能骗过真实驱动程序的判据）：
//   1. xadc_vplant 的 DRP 从机时序与 XADC 一致 → 现有 xadc_frontend 能读数
//   2. am2301_vplant 的 1-Wire 时序与 AM2301 一致 → 现有 am2301_ctrl 能解出
//      正确的温度/湿度，且注入的校验错会被驱动判成 err
//======================================================================
`include "vplant_pkg.vh"

module tb_vplant_if;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg tick_us = 1'b0;
    always #5 clk = ~clk;

    integer uc;
    always @(posedge clk) begin
        if (!rst_n) begin tick_us <= 1'b0; uc <= 0; end
        else if (uc == 99) begin uc <= 0; tick_us <= 1'b1; end
        else begin uc <= uc + 1; tick_us <= 1'b0; end
    end

    // ---------------- 被测信号 ----------------
    reg  signed [31:0] tj_uk = 32'sd45_000_000;   // 45.000 ℃
    reg  signed [31:0] ta_uk = 32'sd25_000_000;   // 25.000 ℃
    reg  [31:0] p_mw = 32'd1600;

    // --- 1) DRP 从机 ---
    reg         den = 1'b0;
    reg  [6:0]  daddr = 7'h00;
    wire [15:0] vp_dout;
    wire        vp_drdy;
    wire [15:0] vp_code, vp_err_cnt;

    xadc_vplant u_vxadc (
        .clk(clk), .rst_n(rst_n), .en(1'b1), .tj_uk(tj_uk), .p_mw(p_mw),
        .den(den), .dwe(1'b0), .daddr(daddr), .di(16'd0),
        .dout(vp_dout), .drdy(vp_drdy),
        .code_mon(vp_code), .err_cnt(vp_err_cnt)
    );

    // --- 2) 1-Wire 从机 + 真实驱动 ---
    wire dq;
    // 单总线必须上拉，否则空闲时浮空成 z → 驱动看到 x（tb_top_smoke 也是这么做的）
    assign (weak1, weak0) dq = 1'b1;
    wire signed [15:0] t_q8_8;
    wire [15:0] rh_q8_8;
    wire ext_valid, ext_err;
    wire [15:0] ext_err_cnt;

    am2301_ctrl #(.CLK_HZ(100_000_000), .PERIOD_US(2000)) u_host (
        .clk(clk), .rst_n(rst_n), .tick_us(tick_us), .dq(dq),
        .t_q8_8(t_q8_8), .rh_q8_8(rh_q8_8),
        .valid(ext_valid), .err(ext_err), .err_cnt(ext_err_cnt)
    );

    am2301_vplant #(.ERR_PER(3)) u_vsen (
        .clk(clk), .rst_n(rst_n), .tick_us(tick_us), .en(1'b1),
        .ta_uk(ta_uk), .rh_pm(16'd550), .dq(dq),
        .last_frame(), .frame_ok(), .conv_cnt(), .err_inj_cnt()
    );

    // ---------------- 量测 ----------------
    integer fails = 0;
    integer nconv = 0;
    integer nerr = 0;
    integer dbgn = 0;
    real t_dec;
    reg dbg_d = 1'b0;
    reg [3:0] st_d = 4'd0;

    always @(posedge clk) begin
        if (rst_n && (u_host.st != st_d) && (dbgn < 8)) begin
            dbgn = dbgn + 1;
            $display("[ST] t=%0t host=%0d vp=%0d dq=%b", $time, u_host.st, u_vsen.st, dq);
            st_d = u_host.st;
        end
        // 驱动跑到 S_CHECK(4'd7) 时把它解码出来的原始帧打出来对比
        dbg_d <= (u_host.st == 4'd7);
        if (rst_n && u_host.st == 4'd7 && !dbg_d)
            $display("[DBG] drv_sh=%010h sum8=%02h chk=%02h | vplant_sh=%010h",
                     u_host.sh, u_host.sum8, u_host.chk, u_vsen.sh);
        if (rst_n && ext_valid) begin
            nconv = nconv + 1;
            $display("[AM2301] #%0d valid: T=%0d (%.3f C)  RH=%0d (%.2f %%)  err_cnt=%0d",
                     nconv, t_q8_8, $itor(t_q8_8)/256.0,
                     rh_q8_8, $itor(rh_q8_8)/256.0, ext_err_cnt);
        end
        if (rst_n && ext_err) nerr = nerr + 1;
    end

    // ---------------- 主流程 ----------------
    task drp_read(input [6:0] a);
        begin
            daddr = a; den = 1'b1; @(posedge clk); #1 den = 1'b0;   // #1 避免与时钟沿竞争
            repeat (6) @(posedge clk);   // DRP 从机 4 拍出 drdy，固定等 6 拍更稳
            
        end
    endtask

    // 看门狗：任何情况下 150 ms 必须收尾，避免回归卡死
    initial begin
        #150ms;
        $display("[WATCHDOG] 150 ms 到：nconv=%0d nerr=%0d，判 FAIL", nconv, nerr);
        $display("  [dbg] vxadc.st=%0d den=%b daddr=%0d drdy=%b dout=%0d rst_n=%b", u_vxadc.st, den, daddr, vp_drdy, vp_dout, rst_n);
        fails = fails + 1;
        $display("==== tb_vplant_if : FAIL ====");
        $finish;
    end

    initial begin
        $display("==== 虚拟器件接口自检 ====");
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ---- DRP: 温度通道 ----
        drp_read(7'h00);
        t_dec = $itor(vp_dout) * 503.975 / 4096.0 - 273.15;
        // 期望：45.0 ℃ 经 增益0.998 + 偏置+1.8 → 约 46.18 ℃
        $display("[DRP] addr=00 code=%0d → %.3f C（期望 ≈46.18 C，含 ±0.15 C 噪声）", vp_dout, t_dec);
        if (t_dec < 45.4 || t_dec > 46.9) begin
            $display("  [FAIL] 温度码与误差模型不符"); fails = fails + 1;
        end

        // ---- DRP: VCCINT ----
        drp_read(7'h01);
        $display("[DRP] addr=01 VCCINT code=%0d (%.3f V)", vp_dout, $itor(vp_dout)*3.0/4096.0);
        if (vp_dout < 12'd1200 || vp_dout > 12'd1366) begin
            $display("  [FAIL] VCCINT 码不合理"); fails = fails + 1;
        end

        // ---- 1-Wire: 等到"至少 3 次有效 + 至少 1 次注入错误"再判（不依赖绝对时间）----
        fork : wait_conv
            begin wait (nconv >= 3); end
            begin wait (nerr  >= 1); end
        join
        #3ms;
        $display("[1W] 完成 %0d 次有效转换，驱动判错 %0d 次（ERR_PER=3，应有错）", nconv, nerr);
        if (nconv < 3) begin $display("  [FAIL] 有效转换太少（时序没对上）"); fails = fails + 1; end
        if (nerr < 1) begin $display("  [FAIL] 注入的校验错没有被驱动识别"); fails = fails + 1; end
        if (t_q8_8 < 16'sd6480 || t_q8_8 > 16'sd6620) begin
            $display("  [FAIL] 温度解出值不符（期望 25.6 C = 6554）"); fails = fails + 1;
        end

        $display("==== %s (fails=%0d) ====", (fails == 0) ? "PASS" : "FAIL", fails);
        if (fails != 0) $fatal(1, "vplant interface failed");
        $finish;
    end
endmodule






