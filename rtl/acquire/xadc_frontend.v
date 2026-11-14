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
    output reg  signed [15:0] tj_q8_8,
    output reg                tj_valid,      // 单拍
    output reg  [11:0]        code_raw
);

    //------------------------------------------------------------------
    // DRP 总线
    //------------------------------------------------------------------
    reg         den;
    reg         dwe;
    reg  [6:0]  daddr;
    reg  [15:0] di;
    wire [15:0] drp_do;
    wire        drdy;

    localparam [1:0] S_HOLD  = 2'd0;
    localparam [1:0] S_WAIT  = 2'd1;
    localparam [1:0] S_ISSUE = 2'd2;
    localparam [1:0] S_DRDY  = 2'd3;

    reg [1:0]  st;
    reg [15:0] ms_cnt;
    reg [15:0] hold_cnt;

    // code*63/2 - 69926，32 位补码；正常范围 -10240…32000
    wire [31:0] t_calc = ((drp_do[11:0] * 16'd63) >> 1) - 32'd69926;

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
        end else begin
            tj_valid <= 1'b0;
            den      <= 1'b0;
            dwe      <= 1'b0;

            case (st)
                // 上电等 10 ms，让 XADC 完成自校准
                S_HOLD: begin
                    if (tick_ms) begin
                        if (hold_cnt >= 16'd10) st <= S_WAIT;
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
                            daddr  <= 7'h00;
                            st     <= S_ISSUE;
                        end else begin
                            ms_cnt <= ms_cnt + 16'd1;
                        end
                    end
                end

                S_ISSUE: st <= S_DRDY;

                S_DRDY: begin
                    if (drdy) begin
                        code_raw <= drp_do[11:0];
                        tj_valid <= 1'b1;
                        st       <= S_WAIT;
                        if ($signed(t_calc) > 32'sd32000)
                            tj_q8_8 <= 16'sd32000;
                        else if ($signed(t_calc) < -32'sd10240)
                            tj_q8_8 <= -16'sd10240;
                        else
                            tj_q8_8 <= t_calc[15:0];
                    end
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
        .dout  (drp_do),
        .drdy  (drdy)
    );
`else
    XADC #(
        .INIT_40 (16'h0000),
        .INIT_41 (16'h0000),
        .INIT_42 (16'h0800)
    ) u_xadc (
        .CONVST      (1'b0),
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
        .DO          (drp_do),
        .DRDY        (drdy),
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

endmodule
