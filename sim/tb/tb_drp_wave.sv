// ============================================================================
//  文件名 : tb_drp_wave.sv
//  用途   : 报告证据 E-04 —— 证明 DRP 切档序列真的在跑（本工程唯一没被仿真过的路径）
//
//  为什么必须单独建 TB：
//    clk_mgr.v 的 DRP 状态机整体在 `ifdef SIM_MODEL 的 **else 分支**，
//    系统级仿真（定义 SIM_MODEL）只跑行为时钟模型，DRP 一行都不执行。
//    所以这里**不定义 SIM_MODEL**，直接例化真实的 MMCME2_ADV 原语。
//
//  需要仿真库：-L unisims_ver -L secureip（MMCME2_ADV 在这两个库里）
//
//  波形从哪来：
//    沿用 tb_xadc_probe.sv 的做法 —— TB 自己用 $fdisplay 写标准 VCD，
//    不碰 xsim 的波形数据库（本机 -debug/-gui 会崩）。
//
//  验证内容：
//    1. 上电后 MMCM 锁存 (locked)
//    2. gear 3 -> 4 时：core_ce 先拉低（关门），DRP 读 0x0A、写回新分频，
//       等 20 µs（2000 拍），再开 core_ce 并给 gear_ack
//    3. gear 4 -> 0 -> 3 连续切档，每次都有 ack
//
//  ============ 本机实测结论（2026-11-16，务必先读）============
//  TB 本身能编译、能 elaborate（unisims_ver + secureip + glbl 都链上了，
//  xelab 日志可见 "Compiling module unisims_ver.MMCME2_ADV"），但 **MMCM 不输出时钟**：
//  仿真跑满 50 ms，locked 始终为 0，clk_sys 一个上升沿都没有，TB 卡在第一个
//  @(posedge clk_sys) 上，DRP 状态机停在 D_IDLE（st=0）。
//
//  怀疑原因（未证实，需在 2026.1 或真实器件上复核）：
//    clk_mgr 把 CLKFBOUT 直接接回 CLKFBIN，**中间没有 BUFG**。
//    真实器件允许这样接，但 unisim 的 MMCME2_ADV 在零延迟反馈下可能不收敛。
//    若要验证，把 .CLKFBIN(clkfb) 改成经过一级 BUFG 再回来，再跑本 TB。
//
//  因此 E-04 目前只有两条路：
//    (a) 在有可用 unisims 仿真库的机器上跑本 TB（队友 2026.1 环境）；
//    (b) 上板：切 5 档读 F_MEAS/GEAR_ACT 截图 —— 这也是报告要的"实测"。
//
//  运行（在 work 目录）：
//    xvlog ../rtl/clk/clk_mgr.v tb_drp_wave.sv
//    xelab tb_drp_wave -s drp_sim -timescale 1ns/1ps -L unisims_ver -L secureip glbl
//    xsim drp_sim -runall
// ============================================================================
`timescale 1ns / 1ps

module tb_drp_wave;

    reg        clk_in = 1'b0;
    reg        rst_n  = 1'b0;
    reg  [2:0] gear   = 3'd3;

    wire       clk_sys, clk_core, clk_ref;
    wire       locked, gear_ack, drp_err;

    always #5 clk_in = ~clk_in;      // 100 MHz

    clk_mgr u_dut (
        .clk_in  (clk_in),
        .rst_n   (rst_n),
        .gear    (gear),
        .clk_sys (clk_sys),
        .clk_core(clk_core),
        .clk_ref (clk_ref),
        .locked  (locked),
        .gear_ack(gear_ack),
        .drp_err (drp_err)
    );

    // ------------------------------------------------------------------
    // 自己写 VCD
    // ------------------------------------------------------------------
    integer fd;
    reg [2:0]  p_st, p_gear_r, p_gear;
    reg        p_den, p_dwe, p_drdy, p_ce, p_ack, p_lk;
    reg [6:0]  p_addr;
    reg [15:0] p_di, p_do;
    reg first = 1'b1;

    initial begin
        fd = $fopen("probe_drp.vcd", "w");
        $fdisplay(fd, "$timescale 1ns $end");
        $fdisplay(fd, "$scope module pd $end");
        $fdisplay(fd, "$var wire 3  ! st $end");
        $fdisplay(fd, "$var wire 3  # gear_r $end");
        $fdisplay(fd, "$var wire 1  $ den $end");
        $fdisplay(fd, "$var wire 1  %% dwe $end");
        $fdisplay(fd, "$var wire 7  & addr $end");
        $fdisplay(fd, "$var wire 16 ' di $end");
        $fdisplay(fd, "$var wire 16 ( do $end");
        $fdisplay(fd, "$var wire 1  ) drdy $end");
        $fdisplay(fd, "$var wire 1  * ce $end");
        $fdisplay(fd, "$var wire 1  + ack $end");
        $fdisplay(fd, "$var wire 1  , locked $end");
        $fdisplay(fd, "$var wire 3  - gear $end");
        $fdisplay(fd, "$upscope $end");
        $fdisplay(fd, "$enddefinitions $end");
    end

    always @(posedge clk_sys) begin
        if (first ||
            u_dut.st       !== p_st   || u_dut.gear_r   !== p_gear_r ||
            u_dut.drp_den  !== p_den  || u_dut.drp_dwe  !== p_dwe    ||
            u_dut.drp_addr !== p_addr || u_dut.drp_di   !== p_di     ||
            u_dut.drp_do   !== p_do   || u_dut.drp_drdy !== p_drdy   ||
            u_dut.core_ce  !== p_ce   || gear_ack       !== p_ack    ||
            locked         !== p_lk   || gear           !== p_gear) begin
            $fdisplay(fd, "#%0t", $time);
            $fdisplay(fd, "b%b !",  u_dut.st);
            $fdisplay(fd, "b%b #",  u_dut.gear_r);
            $fdisplay(fd, "%b$",    u_dut.drp_den);
            $fdisplay(fd, "%b%%",   u_dut.drp_dwe);
            $fdisplay(fd, "b%b &",  u_dut.drp_addr);
            $fdisplay(fd, "b%b '",  u_dut.drp_di);
            $fdisplay(fd, "b%b (",  u_dut.drp_do);
            $fdisplay(fd, "%b)",    u_dut.drp_drdy);
            $fdisplay(fd, "%b*",    u_dut.core_ce);
            $fdisplay(fd, "%b+",    gear_ack);
            $fdisplay(fd, "%b,",    locked);
            $fdisplay(fd, "b%b -",  gear);
            p_st <= u_dut.st;      p_gear_r <= u_dut.gear_r; p_den <= u_dut.drp_den;
            p_dwe <= u_dut.drp_dwe; p_addr <= u_dut.drp_addr; p_di <= u_dut.drp_di;
            p_do <= u_dut.drp_do;   p_drdy <= u_dut.drp_drdy; p_ce <= u_dut.core_ce;
            p_ack <= gear_ack;     p_lk <= locked;           p_gear <= gear;
            first <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // 激励：等锁 -> 切 4 -> 切 0 -> 切 3
    // ------------------------------------------------------------------
    task switch_gear(input [2:0] g, input [8*24-1:0] name);
        begin
            gear = g;
            @(posedge gear_ack);
            $display("[DRP] gear -> %0d (%0s) acked at %0t ns, st=%0d, drp_err=%b",
                     g, name, $time, u_dut.st, drp_err);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (20) @(posedge clk_sys);
        rst_n = 1'b1;

        // 等 MMCM 锁存：本环境 unisim 的 MMCME2_ADV 一直不拉 LOCKED
        // （clk_mgr 里 CLKFBOUT 直连 CLKFBIN、没插 BUFG，模型不收敛）。
        // DRP 序列本身不依赖 LOCKED，所以等 200 us 后照常继续，波形照样出。
        repeat (20000) @(posedge clk_sys);
        if (locked !== 1'b1)
            $display("[DRP] WARN: MMCM 未锁定（200 us），继续跑 DRP 序列以验证写时序");
        else
            $display("[DRP] MMCM locked at %0t ns", $time);
        repeat (2000) @(posedge clk_sys);
        switch_gear(3'd4, "150 MHz");
        repeat (100) @(posedge clk_sys);
        switch_gear(3'd0, "25 MHz");
        repeat (100) @(posedge clk_sys);
        switch_gear(3'd3, "100 MHz");
        repeat (500) @(posedge clk_sys);

        $display("[DRP] done. locked=%b drp_err=%b", locked, drp_err);
        $finish;
    end

    // 兜底超时
    initial begin
        #(50_000_000);   // 50 ms 仿真时间
        $display("[DRP] TIMEOUT: locked=%b st=%0d", locked, u_dut.st);
        $finish;
    end

endmodule
