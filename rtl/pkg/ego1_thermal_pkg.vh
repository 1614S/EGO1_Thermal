//======================================================================
// ego1_thermal_pkg.vh
// 全局参数：唯一事实源。RTL / TB / 脚本都只引用这里。
//======================================================================
`ifndef EGO1_THERMAL_PKG_VH
`define EGO1_THERMAL_PKG_VH

//--------------------------------------------------------------------
// 时钟与采样
//--------------------------------------------------------------------
`define SYS_CLK_HZ          32'd100_000_000
`define SAMPLE_MS           16'd100          // 统一采样周期 100 ms
`define PWM_BITS            12               // 100 MHz / 2^12 = 24.414 kHz

//--------------------------------------------------------------------
// 仿真加速：只缩放“ms 时基”，不缩放 UART/AM2302 等真实时序
// 用法：xvlog -d SIM_MODEL -d SIM_ACCEL=100 ...
//--------------------------------------------------------------------
`ifdef SIM_MODEL
  `ifndef SIM_ACCEL
    `define SIM_ACCEL         1000        // 仿真默认加速 1000x
  `endif
`else
  `ifndef SIM_ACCEL
    `define SIM_ACCEL         1           // 硬件构建必须为 1
    // 曾经这里无条件写 1000（不在 SIM_MODEL 里判），导致真实比特流的
    // tick_ms 快 1000 倍：帧率 357 帧/s、XADC 来不及转换、Tj 直接读到 125℃ 饱和。
    // 仿真永远带 -DSIM_ACCEL，所以永远测不出这个问题 —— 只有上板才会暴露。
  `endif
`endif

`ifdef SIM_MODEL
  `define UART_DIV            16'd10           // 10 Mbps，仿真用
`else
  `define UART_DIV            16'd868          // 115200 @100 MHz
`endif

//--------------------------------------------------------------------
// 温度（Q8.8：1 LSB = 1/256 ℃）
//--------------------------------------------------------------------
`define T_ZERO_Q8_8         32'sd0
`define T_MIN_Q8_8          (-32'sd10240)     // -40.0 ℃
`define T_MAX_Q8_8          (32'sd32000)      // +125.0 ℃
`define T_40_Q8_8           (16'sd10240)
`define T_50_Q8_8           (16'sd12800)
`define T_60_Q8_8           (16'sd15360)

// 告警阈值默认值（可由寄存器改写）
`define TH_WARN_Q8_8        (16'sd16128)      // 63.0 ℃
`define TH_HOT_Q8_8         (16'sd17664)      // 69.0 ℃
`define TH_CRIT_Q8_8        (16'sd19456)      // 76.0 ℃

// PI 默认参数
`define KP_DEF_Q8_8         (16'd51200)        // 200.0 duty/℃（虚拟热台上整定，见 .work/03_整定记录.md）
`define KI_DEF_Q8_8         (16'd256)          // 1.0 duty/(℃·样本)

//--------------------------------------------------------------------
// 档位：MMCM VCO = 1200 MHz，CLKOUT1 整数分频
// gear 0..4 -> 25 / 50 / 75 / 100 / 150 MHz
// CLKOUT1 Reg1 = 0x1000 | (high<<6) | low, high=low=divide/2
//--------------------------------------------------------------------
`define GEAR_NUM            5
`define GEAR_DEF            3                // 上电默认 100 MHz

//--------------------------------------------------------------------
// UART 帧
//--------------------------------------------------------------------
`define FRAME_LEN           32
`define FRAME_H0            8'hA5
`define FRAME_H1            8'h5A
`define FRAME_VER           8'h01
`define FRAME_TYPE_SAMPLE   8'h01
`define FRAME_TYPE_EVENT    8'h02

`endif // EGO1_THERMAL_PKG_VH

