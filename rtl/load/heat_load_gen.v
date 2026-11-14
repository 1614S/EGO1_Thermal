//======================================================================
// heat_load_gen.v —— 可调自热负载
// 16 条并行 LFSR，level 控制开启条数；翻转计入 act，作为功率代理。
// 目的不是“算得对”，而是在 25~150 MHz 下稳定消耗功率。
//======================================================================
module heat_load_gen (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [3:0]  level,
    output reg  [15:0] act
);

    reg [31:0] l [0:15];
    integer    i;
    reg        last;
    reg [15:0] acc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 16; i = i + 1)
                l[i] <= 32'h1234_5678 ^ (i * 32'h0101_0101);
            acc  <= 16'd0;
            last <= 1'b0;
            act  <= 16'd0;
        end else begin
            for (i = 0; i < 16; i = i + 1) begin
                if (enable && (level > i))
                    l[i] <= {l[i][30:0], l[i][31] ^ l[i][21] ^ l[i][1] ^ l[i][0]};
            end

            acc <= l[0][15:0] ^ l[1][31:16] ^ l[2][7:0] ^ l[3][23:8];

            last <= acc[0];
            if (last ^ acc[0]) act <= act + 16'd1;
        end
    end

endmodule
