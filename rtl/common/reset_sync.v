//======================================================================
// reset_sync.v —— 异步置位、同步释放
// 每个时钟域一个；输出高有效 rst_n。
//======================================================================
module reset_sync (
    input  wire clk,
    input  wire arst_n,        // 异步复位输入（低有效）
    output wire rst_n          // 同步释放后的复位（低有效）
);

    (* ASYNC_REG = "TRUE" *) reg meta;
    (* ASYNC_REG = "TRUE" *) reg sync;

    always @(posedge clk or negedge arst_n) begin
        if (!arst_n) begin
            meta <= 1'b0;
            sync <= 1'b0;
        end else begin
            meta <= 1'b1;
            sync <= meta;
        end
    end

    assign rst_n = sync;

endmodule
