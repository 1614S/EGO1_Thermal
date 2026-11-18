//======================================================================
// xadc_model.v —— XADC DRP 行为模型（仅仿真）
// 0x00 温度、0x01 VCCINT、0x02 VCCAUX；读延迟 3 拍后给 DRDY。
// TB 可通过层次引用直接改 tj_c（单位 ℃）。
//======================================================================
`timescale 1ns/1ps

module xadc_model (
    input  wire        dclk,
    input  wire        den,
    input  wire [6:0]  daddr,
    input  wire [15:0] di,
    input  wire        dwe,
    output reg  [15:0] dout,
    output reg         drdy
);

    real tj_c   = 45.0;
    real vccint = 1.000;
    real vccaux = 1.800;

    reg [1:0]  dly;
    reg        pend;
    reg [6:0]  addr_r;
    integer    code;

    always @(posedge dclk) begin
        drdy <= 1'b0;

        if (den) begin
            addr_r <= daddr;
            pend   <= 1'b1;
            dly    <= 2'd0;
        end else if (pend) begin
            if (dly == 2'd2) begin
                pend <= 1'b0;
                case (addr_r)
                    7'h00: begin
                        code = (tj_c + 273.15) * 4096.0 / 503.975;
                        if (code < 0)    code = 0;
                        if (code > 4095) code = 4095;
                        dout <= {code[11:0], 4'h0};   // 与真实 XADC 一致：高位对齐（DO[15:4]）
                    end
                    7'h01: begin
                        code = vccint * 4096.0 / 3.0;
                        dout <= {code[11:0], 4'h0};   // 与真实 XADC 一致：高位对齐（DO[15:4]）
                    end
                    7'h02: begin
                        code = vccaux * 4096.0 / 3.0;
                        dout <= {code[11:0], 4'h0};   // 与真实 XADC 一致：高位对齐（DO[15:4]）
                    end
                    default: dout <= 16'h0000;
                endcase
                drdy <= 1'b1;
            end else begin
                dly <= dly + 2'd1;
            end
        end
    end

endmodule

