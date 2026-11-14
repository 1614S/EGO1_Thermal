//======================================================================
// clk_mgr.v —— 时钟管理
//   clk_sys  : 100 MHz 常开（MMCM CLKOUT0）
//   clk_core : 25/50/75/100/150 MHz（MMCM CLKOUT1 + BUFGCE + DRP 改分频）
//
// 仿真：行为模型，gear 改变后半个周期生效，无锁定时间。
// 综合：MMCME2_ADV + DRP 读改写 CLKOUT1 Reg1
//       Reg1 = 0x1000 | (high<<6) | low，high = low = divide/2
//       （与 Clocking Wizard 生成的编码一致）
//======================================================================
`include "ego1_thermal_pkg.vh"

module clk_mgr (
    input  wire       clk_in,
    input  wire       rst_n,
    input  wire [2:0] gear,
    output wire       clk_sys,
    output wire       clk_core,
    output wire       clk_ref,       // 200 MHz 测频参考（> 最高档 150 MHz）
    output wire       locked,
    output reg        gear_ack,
    output reg        drp_err
);

`ifdef SIM_MODEL
    //------------------------------------------------------------------
    // 行为模型
    //------------------------------------------------------------------
    reg  clk_sys_r  = 1'b0;
    reg  clk_core_r = 1'b0;
    reg  clk_ref_r  = 1'b0;
    real half;
    reg  [2:0] gear_d;
    reg  [7:0] ack_cnt;

    initial begin
        forever #5.0 clk_sys_r = ~clk_sys_r;          // 100 MHz
    end

    initial begin
        forever #2.5 clk_ref_r = ~clk_ref_r;          // 200 MHz
    end

    initial begin
        forever begin
            case (gear)
                3'd0:    half = 20.0000;              // 25 MHz
                3'd1:    half = 10.0000;              // 50 MHz
                3'd2:    half =  6.6667;              // 75 MHz
                3'd3:    half =  5.0000;              // 100 MHz
                3'd4:    half =  3.3333;              // 150 MHz
                default: half =  5.0000;
            endcase
            #(half) clk_core_r = ~clk_core_r;
        end
    end

    assign clk_sys  = clk_sys_r;
    assign clk_core = clk_core_r;
    assign clk_ref  = clk_ref_r;
    assign locked   = 1'b1;

    always @(posedge clk_sys_r or negedge rst_n) begin
        if (!rst_n) begin
            gear_d   <= 3'd3;
            ack_cnt  <= 8'd0;
            gear_ack <= 1'b0;
        end else begin
            gear_d   <= gear;
            gear_ack <= 1'b0;
            if (gear != gear_d)       ack_cnt <= 8'd50;
            else if (ack_cnt != 8'd0) ack_cnt <= ack_cnt - 8'd1;
            if (ack_cnt == 8'd1)      gear_ack <= 1'b1;
        end
    end

    initial drp_err = 1'b0;

`else
    //------------------------------------------------------------------
    // 真实实现
    //------------------------------------------------------------------
    wire clkfb, clk0, clk1, clk2;
    wire mmcm_locked;

    reg        drp_den;
    reg        drp_dwe;
    reg  [6:0] drp_addr;
    reg [15:0] drp_di;
    wire [15:0] drp_do;
    wire        drp_drdy;

    reg  [15:0] do_r;
    reg  [2:0]  st;
    reg  [15:0] settle_cnt;
    reg  [15:0] hi, lo;
    reg         core_ce;
    reg  [2:0]  gear_r;

    localparam [2:0] D_IDLE   = 3'd0;
    localparam [2:0] D_RD     = 3'd1;
    localparam [2:0] D_RDW    = 3'd2;
    localparam [2:0] D_WR     = 3'd3;
    localparam [2:0] D_WRW    = 3'd4;
    localparam [2:0] D_SETTLE = 3'd5;

    assign locked = mmcm_locked;

    MMCME2_ADV #(
        .BANDWIDTH           ("OPTIMIZED"),
        .CLKOUT4_CASCADE     ("FALSE"),
        .COMPENSATION        ("ZHOLD"),
        .STARTUP_WAIT        ("FALSE"),
        .DIVCLK_DIVIDE       (1),
        .CLKFBOUT_MULT_F     (12.000),
        .CLKFBOUT_PHASE      (0.000),
        .CLKFBOUT_USE_FINE_PS("FALSE"),
        .CLKOUT0_DIVIDE_F    (12.000),
        .CLKOUT0_PHASE       (0.000),
        .CLKOUT0_DUTY_CYCLE  (0.500),
        .CLKOUT0_USE_FINE_PS ("FALSE"),
        .CLKOUT1_DIVIDE      (8),
        .CLKOUT1_PHASE       (0.000),
        .CLKOUT1_DUTY_CYCLE  (0.500),
        .CLKOUT1_USE_FINE_PS ("FALSE"),
        .CLKOUT2_DIVIDE      (6),
        .CLKOUT2_PHASE       (0.000),
        .CLKOUT2_DUTY_CYCLE  (0.500),
        .CLKOUT2_USE_FINE_PS ("FALSE"),
        .CLKIN1_PERIOD       (10.000),
        .REF_JITTER1         (0.010)
    ) u_mmcm (
        .CLKFBOUT        (clkfb),
        .CLKFBOUTB       (),
        .CLKOUT0         (clk0),
        .CLKOUT0B        (),
        .CLKOUT1         (clk1),
        .CLKOUT1B        (),
        .CLKOUT2         (clk2),
        .CLKOUT2B        (),
        .CLKOUT3         (),
        .CLKOUT3B        (),
        .CLKOUT4         (),
        .CLKOUT5         (),
        .CLKOUT6         (),
        .CLKFBIN         (clkfb),
        .CLKIN1          (clk_in),
        .CLKIN2          (1'b0),
        .CLKINSEL        (1'b1),
        .DADDR           (drp_addr),
        .DCLK            (clk_sys),
        .DEN             (drp_den),
        .DI              (drp_di),
        .DO              (drp_do),
        .DRDY            (drp_drdy),
        .DWE             (drp_dwe),
        .LOCKED          (mmcm_locked),
        .PWRDWN          (1'b0),
        .RST             (~rst_n),
        .PSDONE          (),
        .PSCLK           (1'b0),
        .PSEN            (1'b0),
        .PSINCDEC        (1'b0),
        .CLKINSTOPPED    ()
    );

    BUFG   u_bufg_sys    (.I(clk0), .O(clk_sys));
    BUFGCE u_bufgce_core (.I(clk1), .CE(core_ce), .O(clk_core));
    BUFG   u_bufg_ref    (.I(clk2), .O(clk_ref));

    //------------------------------------------------------------------
    // 档位 -> 分频
    //------------------------------------------------------------------
    always @(*) begin
        case (gear)
            3'd0:    begin hi = 16'd24; lo = 16'd24; end   // /48 = 25 MHz
            3'd1:    begin hi = 16'd12; lo = 16'd12; end   // /24 = 50 MHz
            3'd2:    begin hi = 16'd8;  lo = 16'd8;  end   // /16 = 75 MHz
            3'd3:    begin hi = 16'd6;  lo = 16'd6;  end   // /12 = 100 MHz
            3'd4:    begin hi = 16'd4;  lo = 16'd4;  end   // /8  = 150 MHz
            default: begin hi = 16'd6;  lo = 16'd6;  end
        endcase
    end

    //------------------------------------------------------------------
    // DRP 序列：读 0x0A -> 改分频 -> 写回 -> 等 20 us -> 开时钟
    //------------------------------------------------------------------
    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) begin
            st         <= D_IDLE;
            drp_den    <= 1'b0;
            drp_dwe    <= 1'b0;
            drp_addr   <= 7'h0A;
            drp_di     <= 16'h0000;
            do_r       <= 16'h0000;
            settle_cnt <= 16'd0;
            core_ce    <= 1'b0;
            gear_r     <= 3'd4;
            gear_ack   <= 1'b0;
            drp_err    <= 1'b0;
        end else begin
            drp_den  <= 1'b0;
            drp_dwe  <= 1'b0;
            gear_ack <= 1'b0;

            case (st)
                D_IDLE: begin
                    core_ce <= 1'b1;
                    if (gear != gear_r) begin
                        core_ce <= 1'b0;              // 先关门，避免窄脉冲
                        st      <= D_RD;
                    end
                end

                D_RD: begin
                    drp_den  <= 1'b1;
                    drp_dwe  <= 1'b0;
                    drp_addr <= 7'h0A;
                    st       <= D_RDW;
                end

                D_RDW: begin
                    if (drp_drdy) begin
                        do_r <= drp_do;
                        st   <= D_WR;
                    end
                end

                D_WR: begin
                    drp_den  <= 1'b1;
                    drp_dwe  <= 1'b1;
                    drp_addr <= 7'h0A;
                    drp_di   <= (do_r & 16'h1000) | (hi << 6) | lo;
                    st       <= D_WRW;
                end

                D_WRW: begin
                    if (drp_drdy) begin
                        settle_cnt <= 16'd2000;       // 20 us @100 MHz
                        gear_r     <= gear;
                        st         <= D_SETTLE;
                    end
                end

                D_SETTLE: begin
                    if (settle_cnt != 16'd0) begin
                        settle_cnt <= settle_cnt - 16'd1;
                    end else begin
                        core_ce  <= 1'b1;
                        gear_ack <= 1'b1;
                        st       <= D_IDLE;
                    end
                end

                default: st <= D_IDLE;
            endcase
        end
    end
`endif

endmodule
