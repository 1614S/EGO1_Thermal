`timescale 1ns/1ps
//======================================================================
// frame_pack.v —— 32 字节遥测帧 + CRC16-CCITT
// 布局见 docs/01_架构与仿真方案.md 表 5.4
//======================================================================
`include "ego1_thermal_pkg.vh"

module frame_pack (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               send,
    input  wire        [15:0] seq_in,
    input  wire        [31:0] ts_in,
    input  wire signed [15:0] tj_in,
    input  wire signed [15:0] text_in,
    input  wire        [15:0] rh_in,
    input  wire signed [15:0] tset_in,
    input  wire        [10:0] duty_in,
    input  wire        [2:0]  gear_in,
    input  wire        [1:0]  state_in,
    input  wire        [15:0] f_in,
    input  wire        [15:0] p_in,
    input  wire        [7:0]  age_in,
    input  wire        [7:0]  flags_in,
    output reg                tx_start,
    output reg         [7:0]  tx_data,
    input  wire               tx_busy,
    output reg                busy
);

    //------------------------------------------------------------------
    // 字段锁存
    //------------------------------------------------------------------
    reg [15:0] seq_r, rh_r, f_r, p_r;
    // 湿度字段按文档发 0.1 %RH：rh 内部是 Q8.8，×10>>8 换算（14080 → 550）
    wire [31:0] rh_tmp = rh_r * 32'd10;        // 32 位上下文，否则 14080×10 会截断（踩过）
    wire [15:0] rh10   = rh_tmp[23:8];
    reg [31:0] ts_r;
    reg signed [15:0] tj_r, text_r, tset_r;
    reg [10:0] duty_r;
    reg [2:0]  gear_r;
    reg [1:0]  state_r;
    reg [7:0]  age_r, flags_r;

    //------------------------------------------------------------------
    // CRC16-CCITT-FALSE：poly 0x1021, init 0xFFFF
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

    reg [15:0] crc;

    //------------------------------------------------------------------
    // 当前要发送的字节
    //------------------------------------------------------------------
    function [7:0] byte_at;
        input [4:0] i;
        begin
            case (i)
                5'd0:  byte_at = 8'hA5;
                5'd1:  byte_at = 8'h5A;
                5'd2:  byte_at = `FRAME_VER;
                5'd3:  byte_at = `FRAME_TYPE_SAMPLE;
                5'd4:  byte_at = seq_r[7:0];
                5'd5:  byte_at = seq_r[15:8];
                5'd6:  byte_at = ts_r[7:0];
                5'd7:  byte_at = ts_r[15:8];
                5'd8:  byte_at = ts_r[23:16];
                5'd9:  byte_at = ts_r[31:24];
                5'd10: byte_at = tj_r[7:0];
                5'd11: byte_at = tj_r[15:8];
                5'd12: byte_at = text_r[7:0];
                5'd13: byte_at = text_r[15:8];
                5'd14: byte_at = rh10[7:0];
                5'd15: byte_at = rh10[15:8];
                5'd16: byte_at = tset_r[7:0];
                5'd17: byte_at = tset_r[15:8];
                5'd18: byte_at = {5'd0, duty_r[2:0]};
                5'd19: byte_at = duty_r[10:3];
                5'd20: byte_at = {5'd0, gear_r};
                5'd21: byte_at = {6'd0, state_r};
                5'd22: byte_at = f_r[7:0];
                5'd23: byte_at = f_r[15:8];
                5'd24: byte_at = p_r[7:0];
                5'd25: byte_at = p_r[15:8];
                5'd26: byte_at = age_r;
                5'd27: byte_at = flags_r;
                5'd28: byte_at = 8'h00;
                5'd29: byte_at = 8'h00;
                5'd30: byte_at = crc[7:0];
                5'd31: byte_at = crc[15:8];
                default: byte_at = 8'h00;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_ISSUE = 2'd1;
    localparam [1:0] S_WAIT  = 2'd2;
    localparam [1:0] S_HOLD  = 2'd3;

    reg [1:0] st;
    reg [4:0] idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            idx      <= 5'd0;
            crc      <= 16'hFFFF;
            tx_start <= 1'b0;
            tx_data  <= 8'h00;
            busy     <= 1'b0;
            seq_r    <= 16'd0;
            ts_r     <= 32'd0;
            tj_r     <= 16'sd0;
            text_r   <= 16'sd0;
            rh_r     <= 16'd0;
            tset_r   <= 16'sd0;
            duty_r   <= 11'd0;
            gear_r   <= 3'd0;
            state_r  <= 2'd0;
            f_r      <= 16'd0;
            p_r      <= 16'd0;
            age_r    <= 8'd0;
            flags_r  <= 8'd0;
        end else begin
            case (st)
                S_IDLE: begin
                    tx_start <= 1'b0;
                    busy     <= 1'b0;
                    if (send) begin
                        seq_r   <= seq_in;
                        ts_r    <= ts_in;
                        tj_r    <= tj_in;
                        text_r  <= text_in;
                        rh_r    <= rh_in;
                        tset_r  <= tset_in;
                        duty_r  <= duty_in;
                        gear_r  <= gear_in;
                        state_r <= state_in;
                        f_r     <= f_in;
                        p_r     <= p_in;
                        age_r   <= age_in;
                        flags_r <= flags_in;
                        idx     <= 5'd0;
                        crc     <= 16'hFFFF;
                        busy    <= 1'b1;
                        st      <= S_ISSUE;
                    end
                end

                S_ISSUE: begin
                    tx_data  <= byte_at(idx);
                    tx_start <= 1'b1;
                    if ((idx >= 5'd2) && (idx <= 5'd29))
                        crc <= crc_step(crc, byte_at(idx));
                    st <= S_WAIT;
                end

                S_WAIT: begin
                    tx_start <= 1'b0;
                    st       <= S_HOLD;
                end

                S_HOLD: begin
                    if (!tx_busy) begin
                        if (idx == 5'd31) begin
                            busy <= 1'b0;
                            st   <= S_IDLE;
                        end else begin
                            idx <= idx + 5'd1;
                            st  <= S_ISSUE;
                        end
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

