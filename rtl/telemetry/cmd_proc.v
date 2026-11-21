`timescale 1ns/1ps
//======================================================================
// cmd_proc.v —— ASCII 命令通道（115200，PuTTY 直接敲）
//
// 命令（大小写不敏感，CR 或 LF 结尾）：
//   ?          状态行：T=45.0 A=25.0 S=50.0 D=0204 G=3 ST=0
//              （T=结温、A=AM2302 环境温度、duty 固定 4 位满量程 1000）
//   S=50.0     设目标温度（℃）
//   KP=200     设 Kp（duty/℃，可带小数）
//   KI=1.0     设 Ki（duty/(℃·样本)）
//   R          清报警
//   RT=25.0    告诉系统"现在的真实温度"（参考温度计读数）
//   CA=1.0     直接写标定增益 a（Q4.12，1.0 = 4096）
//   CB=0.5     直接写标定偏置 b（℃）
//
// 两点标定的分工：硬件负责 RT= 记参考温度与 Z 回读原始值，a/b 的算式
// （a=Δref/Δraw、b=ref1−a·raw1）在上位机脚本里算，再用 CA=/CB= 写回。
// 这样避免在 100 MHz 里塞一个变量除法器（实测那条路径会把时序拖垮）。
//   Z          打印 a / b / 原始值（十六进制）
//
// 两个可综合性教训（都踩过）：
//   1. `while (idx < blen)` 这种依赖信号的循环，Vivado 直接报
//      "loop condition does not converge after 2000 iterations"；
//      而把 8 位十进制乘加一次组合展开又会把时序拖垮。
//      所以解析必须**一个字符一个时钟**（S_NUM 状态，8 拍）。
//   2. 两点标定的除法用移位相减序列（33 拍），不用变量除法器。
//======================================================================
module cmd_proc (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               rx_valid,
    input  wire        [7:0]  rx_byte,
    input  wire               tx_busy,
    output reg                tx_start,
    output reg         [7:0]  tx_data,
    output reg                busy,
    output reg                wr_en,
    output reg         [5:0]  wr_addr,
    output reg  [31:0]        wr_data,
    input  wire signed [15:0] tj_f,
    input  wire        [15:0] setpoint,
    input  wire        [10:0] duty,
    input  wire        [2:0]  gear,
    input  wire        [1:0]  state,
    input  wire signed [15:0] tj_raw,
    input  wire signed [15:0] text_f,           // AM2302 环境温度（Q8.8），用于 ? 状态行的 A= 字段
    input  wire        [15:0] cal_a,
    input  wire signed [15:0] cal_b,
    input  wire        [15:0] ext_err_cnt,      // AM2302 总错误数
    input  wire        [15:0] ext_crc_cnt,      // AM2302 校验错数
    output reg         [15:0] n_cmd,
    output reg         [15:0] n_err,
    output reg         [15:0] n_cal
);
    localparam [2:0] S_RX = 3'd0, S_DISP = 3'd1, S_NUM = 3'd2,
                     S_ACT = 3'd3, S_SEND = 3'd4, S_FMT = 3'd5, S_FMT2 = 3'd6;

    localparam [3:0] C_Q = 4'd0, C_Z = 4'd1, C_R = 4'd2, C_S = 4'd3,
                     C_KP = 4'd4, C_KI = 4'd5, C_RT = 4'd6,
                     C_CA = 4'd7, C_CB = 4'd8, C_BAD = 4'd15;

    reg [2:0]  st;
    reg [7:0]  lbuf [0:23];
    reg [4:0]  blen;
    reg [7:0]  rbuf [0:47];
    reg [5:0]  rlen, ridx;
    reg [3:0]  code;
    reg [4:0]  nstart, nk;
    reg        nact;
    reg        tx_gap;         // 发完一个字节后强制空一拍

    reg [15:0] v_int, v_frac;
    reg        v_dot, v_any;

    reg signed [15:0] ref_q;      // 参考温度（RT= 设定），由主机侧算标定时用

    reg        wr2_pend;
    reg [5:0]  wr2_addr;
    reg [31:0] wr2_data;

    integer i;
    reg [7:0] ch;
    // 回复格式化的中间量（不能在表达式上直接取位，Verilog 会报语法错）
    reg [15:0] f_d100, f_d10, f_x1, f_x0;
    // '?' 状态行的格式化结果（分两拍算出来，再在 S_ACT 里只做拼接）
    reg [15:0] t_tj10, t_sp10, t_du10, t_ex10;
    reg        t_tjneg, t_exneg;
    reg  [3:0] tj_d2, tj_d1, tj_d0, sp_d2, sp_d1, sp_d0, du_d3, du_d2, du_d1, du_d0;
    reg  [3:0] ex_d2, ex_d1, ex_d0;

    //------------------------------------------------------------------
    // 回复构造
    //------------------------------------------------------------------
    task putc(input [7:0] c);
        begin
            if (rlen < 6'd46) begin
                rbuf[rlen] = c;
                rlen = rlen + 6'd1;
            end
        end
    endtask

    // 常数除法在 FPGA 里是"乘高位 + 修正"的长进位链，这里全换乘加近似：
    //   /10 ≈ ×205 >> 11      /100 ≈ ×2622 >> 18
    // 注意：拆位必须向下取整。原来加了半个 LSB 做四舍五入，35.5 ℃ 的 t10=355
    // 会被 div10 算成 36，d0 = 355 - 360 = -5 → 低 4 位 = 11 = ';'，
    // 上板实测回复里出现了 "T=35.;"。
    function [15:0] div10(input [15:0] x);
        reg [26:0] m;
        begin m = x * 11'd205; div10 = m[26:11]; end
    endfunction
    function [15:0] div100(input [15:0] x);
        reg [33:0] m;
        begin m = x * 18'd2622; div100 = m[33:18]; end
    endfunction

    task put_temp(input signed [15:0] q);
        reg [15:0] t10, d2, d1, d0;
        reg        neg;
        begin
            neg = q[15];
            t10 = neg ? (((~q + 16'd1) * 21'd5) >> 7) : ((q * 21'd5) >> 7);
            if (neg) putc(8'h2D);
            d2 = div100(t10);
            d1 = div10(t10) - div100(t10) * 16'd10;
            d0 = t10 - div10(t10) * 16'd10;
            putc(8'h30 + {4'd0, d2[3:0]});
            putc(8'h30 + {4'd0, d1[3:0]});
            putc(8'h2E);
            putc(8'h30 + {4'd0, d0[3:0]});
        end
    endtask

    task put_hex4(input [15:0] v);
        integer k;
        reg [3:0] nib;
        begin
            for (k = 3; k >= 0; k = k - 1) begin
                nib = v[k*4 +: 4];
                putc((nib < 4'd10) ? (8'h30 + {4'd0, nib}) : (8'h37 + {4'd0, nib}));
            end
        end
    endtask

    task put_ok;
        begin rlen = 6'd0; putc(8'h4F); putc(8'h4B); putc(8'h0D); putc(8'h0A); end
    endtask

    task put_err;
        begin
            rlen = 6'd0;
            putc(8'h45); putc(8'h52); putc(8'h52); putc(8'h0D); putc(8'h0A);
            n_err = n_err + 16'd1;
        end
    endtask

    // 增益是 Q4.12（×4096），偏置是 Q8.8（×256），别用错
    function [15:0] to_q412(input [15:0] vi, input [15:0] vf);
        reg [31:0] tmp;
        begin
            tmp = (vi * 16'd10 + vf) * 32'd104858;   // ×409.6 = °C→Q4.12
            to_q412 = tmp[31:8];
        end
    endfunction

    function [15:0] to_q88(input [15:0] vi, input [15:0] vf);
        reg [31:0] tmp;
        begin
            tmp = (vi * 16'd10 + vf) * 32'd6554;     // ×25.6 = °C→Q8.8
            to_q88 = tmp[23:8];
        end
    endfunction

    //------------------------------------------------------------------
    // 主状态机
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_RX;
            blen     <= 5'd0;
            rlen     <= 6'd0;
            ridx     <= 6'd0;
            tx_start <= 1'b0;
            tx_data  <= 8'd0;
            busy     <= 1'b0;
            tx_gap   <= 1'b0;
            wr_en    <= 1'b0;
            wr_addr  <= 6'd0;
            wr_data  <= 32'd0;
            n_cmd    <= 16'd0;
            n_err    <= 16'd0;
            n_cal    <= 16'd0;
            code     <= C_BAD;
            nstart   <= 5'd0;
            nk       <= 5'd0;
            nact     <= 1'b1;
            v_int    <= 16'd0;
            v_frac   <= 16'd0;
            v_dot    <= 1'b0;
            v_any    <= 1'b0;
            ref_q    <= 16'sd6400;
            wr2_pend <= 1'b0;
            wr2_addr <= 6'd0;
            wr2_data <= 32'd0;
        end else begin
            tx_start <= 1'b0;
            wr_en    <= 1'b0;

            if (wr2_pend) begin
                wr_en    <= 1'b1;
                wr_addr  <= wr2_addr;
                wr_data  <= wr2_data;
                wr2_pend <= 1'b0;
            end

            case (st)
                S_RX: begin
                    busy <= 1'b0;
                    if (rx_valid) begin
                        if (rx_byte == 8'h0D || rx_byte == 8'h0A) begin
                            if (blen != 5'd0) st <= S_DISP;
                        end else if (blen < 5'd24) begin
                            lbuf[blen] <= rx_byte;
                            blen       <= blen + 5'd1;
                        end
                    end
                end

                S_DISP: begin
                    n_cmd <= n_cmd + 16'd1;
                    for (i = 0; i < 24; i = i + 1)
                        if (lbuf[i] >= 8'h61 && lbuf[i] <= 8'h7A) lbuf[i] <= lbuf[i] - 8'h20;

                    nk    <= 5'd0;
                    v_int <= 16'd0;
                    v_frac<= 16'd0;
                    v_dot <= 1'b0;
                    v_any <= 1'b0;
                    nact  <= 1'b1;
                    if (lbuf[0] == 8'h3F) begin                    // ?
                        code <= C_Q;  st <= S_FMT;
                    end else if (lbuf[0] == 8'h5A) begin           // Z
                        code <= C_Z;  st <= S_ACT;
                    end else if (lbuf[0] == 8'h52 && lbuf[1] == 8'h54 && lbuf[2] == 8'h3D) begin
                        code <= C_RT; nstart <= 5'd3; st <= S_NUM;
                    end else if (lbuf[0] == 8'h52 && blen == 5'd1) begin
                        code <= C_R;  st <= S_ACT;
                    end else if (lbuf[0] == 8'h53 && lbuf[1] == 8'h3D) begin
                        code <= C_S;  nstart <= 5'd2; st <= S_NUM;
                    end else if (lbuf[0] == 8'h4B && lbuf[1] == 8'h50 && lbuf[2] == 8'h3D) begin
                        code <= C_KP; nstart <= 5'd3; st <= S_NUM;
                    end else if (lbuf[0] == 8'h4B && lbuf[1] == 8'h49 && lbuf[2] == 8'h3D) begin
                        code <= C_KI; nstart <= 5'd3; st <= S_NUM;
                    end else if (lbuf[0] == 8'h43 && lbuf[1] == 8'h41 && lbuf[2] == 8'h3D) begin
                        code <= C_CA; nstart <= 5'd3; st <= S_NUM;   // CA=
                    end else if (lbuf[0] == 8'h43 && lbuf[1] == 8'h42 && lbuf[2] == 8'h3D) begin
                        code <= C_CB; nstart <= 5'd3; st <= S_NUM;   // CB=
                    end else begin
                        code <= C_BAD; st <= S_ACT;
                    end
                end

                // 逐字符解析：8 拍，一个时钟处理一个字符
                S_NUM: begin
                    if (nk < 5'd8) begin
                        ch = (nstart + nk < 5'd24) ? lbuf[nstart + nk] : 8'h00;
                        if (nact) begin
                            if (ch >= 8'h30 && ch <= 8'h39) begin
                                if (!v_dot)
                                    v_int  <= v_int * 16'd10 + {12'd0, (ch - 8'h30)};
                                else if (v_frac < 16'd100)
                                    v_frac <= v_frac * 16'd10 + {12'd0, (ch - 8'h30)};
                                v_any <= 1'b1;
                            end else if (ch == 8'h2E) begin
                                v_dot <= 1'b1;
                            end else begin
                                nact  <= 1'b0;
                            end
                        end
                        nk <= nk + 5'd1;
                    end else begin
                        st <= S_ACT;
                    end
                end

                // 格式化第 1 拍：Q8.8 → 0.1 ℃ 单位（只做一次乘加）
                S_FMT: begin
                    t_tjneg <= tj_f[15];
                    t_tj10  <= tj_f[15] ? (((~tj_f + 16'd1) * 21'd5) >> 7)
                                        : ((tj_f * 21'd5) >> 7);
                    t_sp10  <= setpoint[15] ? (((~setpoint + 16'd1) * 21'd5) >> 7)
                                            : ((setpoint * 21'd5) >> 7);
                    t_du10  <= {5'd0, duty};
                    t_exneg <= text_f[15];
                    t_ex10  <= text_f[15] ? (((~text_f + 16'd1) * 21'd5) >> 7)
                                          : ((text_f * 21'd5) >> 7);
                    st      <= S_FMT2;
                end

                // 格式化第 2 拍：按十进制拆位（表达式不能直接取位，先落到 scratch reg）
                S_FMT2: begin
                    f_d100 = div100(t_tj10); f_d10 = div10(t_tj10);
                    f_x1   = f_d10 - f_d100 * 16'd10;
                    f_x0   = t_tj10 - f_d10 * 16'd10;
                    tj_d2 <= f_d100[3:0]; tj_d1 <= f_x1[3:0]; tj_d0 <= f_x0[3:0];

                    f_d100 = div100(t_sp10); f_d10 = div10(t_sp10);
                    f_x1   = f_d10 - f_d100 * 16'd10;
                    f_x0   = t_sp10 - f_d10 * 16'd10;
                    sp_d2 <= f_d100[3:0]; sp_d1 <= f_x1[3:0]; sp_d0 <= f_x0[3:0];

                    f_d100 = div100(t_ex10); f_d10 = div10(t_ex10);
                    f_x1   = f_d10 - f_d100 * 16'd10;
                    f_x0   = t_ex10 - f_d10 * 16'd10;
                    ex_d2 <= f_d100[3:0]; ex_d1 <= f_x1[3:0]; ex_d0 <= f_x0[3:0];

                    f_d100 = div100(t_du10); f_d10 = div10(t_du10);
                    f_x1   = f_d10 - f_d100 * 16'd10;
                    f_x0   = t_du10 - f_d10 * 16'd10;
                    // duty 满量程 1000，需要第 4 位；<1000 时首位为 0
                    du_d3 <= (t_du10 >= 16'd1000) ? 4'd1 : 4'd0;
                    du_d2 <= f_d100[3:0] - ((t_du10 >= 16'd1000) ? 4'd10 : 4'd0);
                    du_d1 <= f_x1[3:0]; du_d0 <= f_x0[3:0];

                    st <= S_ACT;
                end

                S_ACT: begin
                    rlen = 6'd0;
                    case (code)
                        C_Q: begin
                            putc(8'h54); putc(8'h3D);
                            if (t_tjneg) putc(8'h2D);
                            putc(8'h30 + {4'd0, tj_d2}); putc(8'h30 + {4'd0, tj_d1});
                            putc(8'h2E); putc(8'h30 + {4'd0, tj_d0});
                            // 环境温度（AM2302），放在结温和设定值之间
                            putc(8'h20); putc(8'h41); putc(8'h3D);
                            if (t_exneg) putc(8'h2D);
                            putc(8'h30 + {4'd0, ex_d2}); putc(8'h30 + {4'd0, ex_d1});
                            putc(8'h2E); putc(8'h30 + {4'd0, ex_d0});
                            putc(8'h20); putc(8'h53); putc(8'h3D);
                            putc(8'h30 + {4'd0, sp_d2}); putc(8'h30 + {4'd0, sp_d1});
                            putc(8'h2E); putc(8'h30 + {4'd0, sp_d0});
                            putc(8'h20); putc(8'h44); putc(8'h3D);
                            putc(8'h30 + {4'd0, du_d3}); putc(8'h30 + {4'd0, du_d2});
                            putc(8'h30 + {4'd0, du_d1}); putc(8'h30 + {4'd0, du_d0});
                            putc(8'h20); putc(8'h47); putc(8'h3D); putc(8'h30 + {5'd0, gear});
                            putc(8'h20); putc(8'h53); putc(8'h54); putc(8'h3D);
                            putc(8'h30 + {6'd0, state});
                            putc(8'h0D); putc(8'h0A);
                        end
                        C_Z: begin
                            putc(8'h41); putc(8'h3D); put_hex4(cal_a);
                            putc(8'h20); putc(8'h42); putc(8'h3D); put_hex4(cal_b);
                            putc(8'h20); putc(8'h52); putc(8'h3D); put_hex4(tj_raw);
                            // AM2302 诊断：E=总错误数，C=校验错数（C 增长说明传感器应答了但位解码错）
                            putc(8'h20); putc(8'h45); putc(8'h3D); put_hex4(ext_err_cnt);
                            putc(8'h20); putc(8'h43); putc(8'h3D); put_hex4(ext_crc_cnt);
                            putc(8'h0D); putc(8'h0A);
                        end
                        C_R: begin
                            wr_en   <= 1'b1;
                            wr_addr <= 6'h03;
                            wr_data <= 32'h0010;
                            put_ok;
                        end
                        C_S: begin
                            wr_en   <= 1'b1;
                            wr_addr <= 6'h04;
                            wr_data <= {16'd0, to_q88(v_int, v_frac)};
                            put_ok;
                        end
                        C_KP: begin
                            wr_en   <= 1'b1;
                            wr_addr <= 6'h07;
                            wr_data <= {16'd0, to_q88(v_int, v_frac)};
                            put_ok;
                        end
                        C_KI: begin
                            wr_en   <= 1'b1;
                            wr_addr <= 6'h08;
                            wr_data <= {16'd0, to_q88(v_int, v_frac)};
                            put_ok;
                        end
                        C_RT: begin
                            ref_q <= to_q88(v_int, v_frac);
                            put_ok;
                        end
                        C_CA: begin
                            wr_en   <= 1'b1;
                            wr_addr <= 6'h0C;                      // 增益 a（Q4.12）
                            wr_data <= {16'd0, to_q412(v_int, v_frac)};
                            put_ok;
                        end
                        C_CB: begin
                            wr_en   <= 1'b1;
                            wr_addr <= 6'h0D;                      // 偏置 b（Q8.8）
                            wr_data <= {16'd0, to_q88(v_int, v_frac)};
                            put_ok;
                        end
                        default: put_err;
                    endcase

                    blen  <= 5'd0;
                    nact  <= 1'b1;
                    if (rlen != 6'd0) begin
                        ridx <= 6'd0;
                        st   <= S_SEND;
                    end else st <= S_RX;
                end

                //================ 发送 ================
                // 每发一个字节后强制空一拍再发下一个。
                //   uart_tx 要到 start 的下一拍才拉高 busy；若当拍 !tx_busy 就继续发，
                //   下一拍的 start 会落在 uart 正在发送上一字节时被忽略，
                //   回复就变成"隔一个丢一个"（上板实测 OK\r\n 只剩 O\r，30 字节只剩 15）。
                //   这里只空一拍、不依赖 busy 拉高，因为 tb_cmd 把 tx_busy 恒接 0，
                //   等 busy 会死锁。
                S_SEND: begin
                    busy <= 1'b1;
                    if (tx_gap) begin
                        tx_gap <= 1'b0;
                    end else if (!tx_busy) begin
                        if (ridx < rlen) begin
                            tx_data  <= rbuf[ridx];
                            tx_start <= 1'b1;
                            ridx     <= ridx + 6'd1;
                            tx_gap   <= 1'b1;
                        end else begin
                            busy <= 1'b0;
                            st   <= S_RX;
                        end
                    end
                end
            endcase
        end
    end

endmodule





