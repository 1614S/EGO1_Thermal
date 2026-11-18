`timescale 1ns/1ps
//======================================================================
// xadc_frontend.v —— XADC 结温采集（DRP 轮询 0x00 温度通道）
//
// 非仿真：直接例化 XADC 原语（单通道模式，默认温度通道），
//         不依赖 XADC Wizard IP，Vivado 升级不会破坏工程。
// 仿真  ：例化 sim/models/xadc_model.v，温度由 TB 以 real 写入。
//
// 定点换算：T_q8_8 = code*63/2 - 69926     （0.12304 ℃/LSB）
//======================================================================
`include "ego1_thermal_pkg.vh"

module xadc_frontend #(
    parameter integer POLL_MS = 10
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_ms,
    // 虚拟热台：plant_sel=1 时由 xadc_vplant 接管 DRP，下游逻辑完全无感
    input  wire               plant_sel,
    input  wire signed [31:0] vp_tj_uk,
    input  wire        [31:0] vp_p_mw,
    output reg  signed [15:0] tj_q8_8,
    output reg                tj_valid,      // 单拍
    output reg  [11:0]        code_raw
);

    //------------------------------------------------------------------
    // DRP 总线（真实器件与虚拟器件二选一，靠 plant_sel 切换）
    //------------------------------------------------------------------
    reg         den;
    reg         dwe;
    reg  [6:0]  daddr;
    reg  [15:0] di;
    wire [15:0] x_do, v_do;
    wire        x_drdy, v_drdy;
    wire [15:0] drp_do = plant_sel ? v_do   : x_do;
    wire        drdy   = plant_sel ? v_drdy : x_drdy;

    localparam [2:0] S_HOLD  = 3'd0;
    localparam [2:0] S_WAIT  = 3'd1;
    localparam [2:0] S_ISSUE = 3'd2;
    localparam [2:0] S_DRDY  = 3'd3;
    localparam [2:0] S_CONV  = 3'd4;   // 换算流水级（拆开组合路径，保 100 MHz）

    reg [2:0]  st;
    reg [20:0] drdy_to;         // DRDY 超时计数
    reg [15:0] timeout_cnt;     // 累计超时次数（监测）
    reg [31:0] t_calc_r;
    reg [15:0] ms_cnt;
    reg [15:0] hold_cnt;

    // code*63/2 - 69926，32 位补码；正常范围 -10240…32000
    // XADC 的测量结果在 DO[15:4]（高位对齐，低 4 位恒 0），不是 DO[11:0]。
    // 取低 12 位等于把码值缩小 16 倍，上板表现为 Tj 在 −40…−3 ℃ 之间乱跳。
    wire [11:0] adc_code = drp_do[15:4];
    wire [31:0] t_calc = ((adc_code * 16'd63) >> 1) - 32'd69926;

    // 单通道模式下 XADC 不会自己转换，必须靠 CONVST 的上升沿启动；
    // 原来 CONVST 接死 0，导致温度寄存器一直是无效值（上板实测 0xFFF）。
    wire convst = tick_ms;

    // 单通道模式还可以用 CONVSTCLK 作转换触发（UG480：两者任一有效即可），
    // 这里再给一个 1 MHz 的触发时钟，双保险。
    reg [6:0] ck_div;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) ck_div <= 7'd0;
        else ck_div <= (ck_div >= 7'd99) ? 7'd0 : ck_div + 7'd1;
    wire conv_clk = (ck_div < 7'd50);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_HOLD;
            ms_cnt   <= 16'd0;
            hold_cnt <= 16'd0;
            den      <= 1'b0;
            dwe      <= 1'b0;
            daddr    <= 7'h00;
            di       <= 16'h0000;
            tj_q8_8  <= 16'sd0;
            tj_valid <= 1'b0;
            code_raw <= 12'd0;
            t_calc_r <= 32'd0;
            drdy_to  <= 21'd0;
            timeout_cnt <= 16'd0;
        end else begin
            tj_valid <= 1'b0;
            den      <= 1'b0;
            dwe      <= 1'b0;

            case (st)
                // 上电等 10 ms，让 XADC 完成自校准
                S_HOLD: begin
                    if (tick_ms) begin
                        if (hold_cnt >= 16'd100) st <= S_WAIT;   // 100 ms：自校准留足余量
                        else hold_cnt <= hold_cnt + 16'd1;
                    end
                end

                // 每 POLL_MS 读一次 0x00
                S_WAIT: begin
                    if (tick_ms) begin
                        if (ms_cnt >= POLL_MS-1) begin
                            ms_cnt <= 16'd0;
                            den    <= 1'b1;
                            dwe    <= 1'b0;
                            daddr  <= 7'h00;   // 温度通道
                            st     <= S_ISSUE;
                        end else begin
                            ms_cnt <= ms_cnt + 16'd1;
                        end
                    end
                end

                S_ISSUE: st <= S_DRDY;

                // 第一级：乘法 + 减法（DSP48 + 进位链），结果打一拍
                S_DRDY: begin
                    // 超时保护：DRP 正常 4 拍就回 drdy，200 µs 还没来就是异常，
                    // 直接回 S_WAIT 重发一次。没有这个分支，一次脏读就会让整个
                    // 采集永久卡死（上板实测过：Tj 永远停在 125 ℃ 饱和值）。
                    if (drdy) begin
                        code_raw <= adc_code;
                        t_calc_r <= t_calc;
                        drdy_to  <= 21'd0;
                        st       <= S_CONV;
                    end else if (drdy_to >= 21'd20000) begin
                        drdy_to  <= 21'd0;
                        timeout_cnt <= timeout_cnt + 16'd1;
                        st       <= S_WAIT;
                    end else begin
                        drdy_to  <= drdy_to + 21'd1;
                    end
                end

                // 第二级：饱和 + 输出，组合逻辑只剩比较
                S_CONV: begin
                    tj_valid <= 1'b1;
                    if ($signed(t_calc_r) > 32'sd32000)
                        tj_q8_8 <= 16'sd32000;
                    else if ($signed(t_calc_r) < -32'sd10240)
                        tj_q8_8 <= -16'sd10240;
                    else
                        tj_q8_8 <= t_calc_r[15:0];
                    st <= S_WAIT;
                end

                default: st <= S_WAIT;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 器件例化
    //------------------------------------------------------------------
`ifdef SIM_MODEL
    xadc_model u_xadc (
        .dclk  (clk),
        .den   (den),
        .daddr (daddr),
        .di    (di),
        .dwe   (dwe),
        .dout  (x_do),
        .drdy  (x_drdy)
    );
`else
    //------------------------------------------------------------------
    // 配置值来源：XADC Wizard（Continuous + 只使能温度 + DRP）生成的
    // INIT 序列，直接从它的 xadc_probe.v 里抄出来，不再手猜寄存器位。
    //   0x31A0 : CFG1，SEQ=0011 → 连续序列模式并在序列里使能温度通道
    //   0x0400 : CFG2，DCLK/ADCCLK 分频 = 4 → ADCCLK = 100/4 = 25 MHz（≤26 MHz）
    //   0x0100 : CFG4，序列通道选择（温度通道）
    // 之前手写的 0x0000/0x0800 是"单通道模式 + 分频 8"，
    // 结果是温度寄存器永远读到无效值 0xFFF（上板实测）。
    //------------------------------------------------------------------
    XADC #(
        .INIT_40 (16'h0000),   // CFG0：无平均
        .INIT_41 (16'h31A0),   // CFG1：连续序列 + 使能温度
        .INIT_42 (16'h0400),   // CFG2：DCLK 分频 4
        .INIT_48 (16'h0100),   // CFG4：序列通道选择
        .INIT_49 (16'h0000),
        .INIT_4A (16'h0000),
        .INIT_4B (16'h0000)
    ) u_xadc (
        .CONVST      (1'b0),          // 连续序列模式内部自动触发
        .CONVSTCLK   (1'b0),
        .DADDR       (daddr),
        .DCLK        (clk),
        .DEN         (den),
        .DI          (di),
        .DWE         (dwe),
        .RESET       (~rst_n),
        .ALM         (),
        .BUSY        (),
        .CHANNEL     (),
        .DO          (x_do),
        .DRDY        (x_drdy),
        .EOC         (),
        .EOS         (),
        .JTAGBUSY    (),
        .JTAGLOCKED  (),
        .JTAGMODIFIED(),
        .MUXADDR     (),
        .OT          (),
        .VAUXN       (16'h0000),
        .VAUXP       (16'h0000),
        .VN          (1'b0),
        .VP          (1'b0)
    );
`endif

    //------------------------------------------------------------------
    // 虚拟 XADC：同一个 DRP 口，plant_sel 时接管
    //------------------------------------------------------------------
    wire [15:0] vp_code_mon, vp_drp_cnt;

    xadc_vplant u_vxadc (
        .clk       (clk),
        .rst_n     (rst_n),
        .en        (plant_sel),
        .tj_uk     (vp_tj_uk),
        .p_mw      (vp_p_mw),
        .den       (den),
        .dwe       (dwe),
        .daddr     (daddr),
        .di        (di),
        .dout      (v_do),
        .drdy      (v_drdy),
        .code_mon  (vp_code_mon),
        .err_cnt   (vp_drp_cnt)
    );

endmodule






