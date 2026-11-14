//======================================================================
// am2301_model.v —— AM2301 单总线从机模型（仅仿真）
// 检测主机 >=1 ms 的低电平后应答 40 bit。
// TB 可直接改 t_c（℃）与 rh（%）。
//======================================================================
`timescale 1ns/1ps

module am2301_model (
    inout wire dq
);

    real t_c  = 45.0;
    real rh   = 50.0;

    reg drive_low = 1'b0;
    assign dq = drive_low ? 1'b0 : 1'bz;

    reg [15:0] rh_raw;
    reg [15:0] t_raw;
    reg [7:0]  chk;
    reg [39:0] frame;
    integer    i;
    integer    code;

    task automatic send_frame;
        begin
            rh_raw = $rtoi(rh * 10.0);
            if (t_c < 0.0) t_raw = 16'h8000 | $rtoi(-t_c * 10.0);
            else           t_raw = $rtoi(t_c * 10.0);
            chk    = (rh_raw[15:8] + rh_raw[7:0] + t_raw[15:8] + t_raw[7:0]) & 8'hFF;
            frame  = {rh_raw, t_raw, chk};
            $display("[MODEL] t_c=%.1f rh=%.1f rh_raw=%0d t_raw=%0d chk=%02h frame=%010h",
                     t_c, rh, rh_raw, t_raw, chk, frame);

            // 应答：80us 低 + 80us 高
            drive_low = 1'b1; #80000;
            drive_low = 1'b0; #80000;

            for (i = 39; i >= 0; i = i - 1) begin
                drive_low = 1'b1; #50000;
                drive_low = 1'b0;
                if (frame[i]) #70000;
                else          #27000;
            end
            // 数据结束后总线拉低 50us 再释放（与真实器件一致，
            // 也是主机判定第 40 位结束的下降沿）
            drive_low = 1'b1; #50000;
            drive_low = 1'b0;
            #50000;
        end
    endtask

    initial begin
        forever begin
            @(negedge dq);
            #900000;                 // 主机仍在拉低（1.2 ms 窗口）
            if (dq === 1'b0) begin
                @(posedge dq);       // 等主机释放
                #30000;              // 20~40 us 后开始应答
                send_frame;
            end
        end
    end

endmodule
