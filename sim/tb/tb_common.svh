`ifndef TB_COMMON_SVH
`define TB_COMMON_SVH
//======================================================================
// tb_common.svh —— 所有模块级 TB 共用的判据模板
//   用法（放在 module 体内、第一个 initial 之前）：
//     `include "tb_common.svh"
//     `CHK(cond, "说明")
//     `CHK_NEAR(got, want, tol, "说明")
//     `DONE("tb_xxx")
//   约定：每个 TB 只证一件事，失败必须打印"哪个期望破了"。
//======================================================================
integer tb_fails;

`define CHK(cond, msg) \
    if (!(cond)) begin tb_fails = tb_fails + 1; $display("  [FAIL] %s", msg); end

`define CHK_NEAR(got, want, tol, msg) \
    if ((((got) - (want)) > (tol)) || (((want) - (got)) > (tol))) begin \
        tb_fails = tb_fails + 1; \
        $display("  [FAIL] %s : got %0.4f want %0.4f (tol %0.4f)", msg, got, want, tol); \
    end

`define DONE(name) \
    begin \
        $display("==== %s : %s (fails=%0d) ====", name, (tb_fails == 0) ? "PASS" : "FAIL", tb_fails); \
        if (tb_fails != 0) $fatal(1, name); \
        $finish; \
    end

`define INIT_TB begin tb_fails = 0; end

`endif
