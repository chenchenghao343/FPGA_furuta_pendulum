//=============================================================================
// tb_adc_prod.v  --  用【真机参数】直接验 v6.1 的 angle_adc 有没有撞拍
//-----------------------------------------------------------------------------
// 背景（必须写清楚，因为这是我前面判断出错的地方）：
//
//   我之前的 tb_adc_compare.v 用了 SMP_STEP=9 / SMP_3MS=159（窗口周期 160）。
//   v6.1 的旧 tb_adc_window.v 用的是 SMP_STEP=9 / SMP_3MS=160（窗口周期 161）。
//   这两个只差 1，但对"第 16 次累加会不会与输出撞拍"是决定性的：
//
//      窗口周期 161，步周期 10：cnt_187 在 10,20,...,160 产生 16 次 step；
//                                out 在 160 拍。但 cnt_187==0 与 cnt_3ms==160
//                                是否同拍取决于相位 —— 周期互质，迟早会撞。
//      窗口周期 160，步周期 10：16 步正好铺满，端点对齐 -> 第16步与 out 同拍。
//
//   而真机参数是 SMP_STEP=9373（步周期 9374）、SMP_3MS=149999（窗口周期 150000）。
//   16 步占 16*9374 = 149984 < 150000，留 16 拍余量。
//
//   ★ 所以本 tb 的目的就一个：用真机参数把撞拍数【数出来】。
//     不加速、不近似，就跑 150000 拍（3ms 仿真时间，可接受）。
//     三种参数各跑一遍：
//        (a) SMP_STEP=9373, SMP_3MS=149999   <- 真机
//        (b) SMP_STEP=9373, SMP_3MS=150000   <- 原始旧 tb 的比例
//        (c) SMP_STEP=9,    SMP_3MS=159      <- 我上一轮错用的参数
//=============================================================================
`timescale 1ns/1ps

module tb_adc_prod;

    reg clk = 0, rst_n = 0;
    reg [9:0] adc_data_in = 10'd25;      // 25*4 = 100
    wire adc_clk, adc_oe_n;
    wire [11:0] raw_a, raw_b, raw_c;
    wire        vld_a, vld_b, vld_c;

    always #10 clk = ~clk;   // 50MHz，周期 20ns

    // 三个 DUT，参数不同，同时跑
    angle_adc #(.SMP_3MS(149_999), .SMP_STEP(9_373)) dut_a (
        .clk_50m(clk), .rst_n(rst_n), .adc_clk(), .adc_oe_n(),
        .adc_data_in(adc_data_in), .angle_raw(raw_a), .angle_vld(vld_a));

    angle_adc #(.SMP_3MS(150_000), .SMP_STEP(9_373)) dut_b (
        .clk_50m(clk), .rst_n(rst_n), .adc_clk(), .adc_oe_n(),
        .adc_data_in(adc_data_in), .angle_raw(raw_b), .angle_vld(vld_b));

    angle_adc #(.SMP_3MS(159), .SMP_STEP(9)) dut_c (
        .clk_50m(clk), .rst_n(rst_n), .adc_clk(), .adc_oe_n(),
        .adc_data_in(adc_data_in), .angle_raw(raw_c), .angle_vld(vld_c));

    // 撞拍计数：step_pulse 与 out_pulse 同一拍
    integer col_a, col_b, col_c;
    integer out_a, out_b, out_c;
    integer acc_a, acc_b, acc_c;          // 每窗累加计数
    integer last_acc_a, last_acc_b, last_acc_c;

    always @(posedge clk) begin
        if (!rst_n) begin
            col_a <= 0; col_b <= 0; col_c <= 0;
            out_a <= 0; out_b <= 0; out_c <= 0;
            acc_a <= 0; acc_b <= 0; acc_c <= 0;
        end else begin
            // ---- A ----
            if (dut_a.step_pulse && dut_a.out_pulse) col_a <= col_a + 1;
            if (dut_a.step_pulse) acc_a <= acc_a + 1;
            if (dut_a.out_pulse) begin
                out_a <= out_a + 1; last_acc_a <= acc_a; acc_a <= 0;
            end
            // ---- B ----
            if (dut_b.step_pulse && dut_b.out_pulse) col_b <= col_b + 1;
            if (dut_b.step_pulse) acc_b <= acc_b + 1;
            if (dut_b.out_pulse) begin
                out_b <= out_b + 1; last_acc_b <= acc_b; acc_b <= 0;
            end
            // ---- C ----
            if (dut_c.step_pulse && dut_c.out_pulse) col_c <= col_c + 1;
            if (dut_c.step_pulse) acc_c <= acc_c + 1;
            if (dut_c.out_pulse) begin
                out_c <= out_c + 1; last_acc_c <= acc_c; acc_c <= 0;
            end
        end
    end

    // negedge 采输出值
    integer ca, cb, cc;
    always @(negedge clk) begin
        if (vld_a === 1'b1) ca = raw_a;
        if (vld_b === 1'b1) cb = raw_b;
        if (vld_c === 1'b1) cc = raw_c;
    end

    integer i;
    initial begin
        col_a=0; col_b=0; col_c=0; out_a=0; out_b=0; out_c=0;
        acc_a=0; acc_b=0; acc_c=0; last_acc_a=0; last_acc_b=0; last_acc_c=0;
        ca=-1; cb=-1; cc=-1;

        $display("================================================================");
        $display(" tb_adc_prod : v6.1 angle_adc 在【真机参数】下有没有撞拍");
        $display(" adc_data_in = 25 -> 每点 x4 = 100（16 点平均应恰为 100）");
        $display("================================================================");

        rst_n = 0;
        #200;
        rst_n = 1;

        // 跑 12 个真机窗口（12 * 150000 拍 = 36ms 仿真时间）
        repeat (150_000 * 12) @(posedge clk);
        #1;

        $display("");
        $display("--- (a) 真机参数  SMP_STEP=9373 (步周期9374)  SMP_3MS=149999 (窗口150000) ---");
        $display("    out_pulse 次数        = %0d", out_a);
        $display("    撞拍次数              = %0d", col_a);
        $display("    最后一窗累加点数      = %0d  (应为 16)", last_acc_a);
        $display("    最后输出 angle_raw    = %0d  (16点同值应为 100)", ca);
        $display("    判定: %s", (ca==100) ? "正确 —— 无 15/16 压缩" : "★ 偏低！");
        $display("");
        $display("--- (b) 旧 tb 比例  SMP_STEP=9373  SMP_3MS=150000 (窗口150001) ---");
        $display("    out_pulse 次数        = %0d", out_b);
        $display("    撞拍次数              = %0d", col_b);
        $display("    最后一窗累加点数      = %0d", last_acc_b);
        $display("    最后输出 angle_raw    = %0d", cb);
        $display("    判定: %s", (cb==100) ? "正确" : "★ 偏低！");
        $display("");
        $display("--- (c) 我上一轮用的  SMP_STEP=9  SMP_3MS=159 (窗口160) ---");
        $display("    out_pulse 次数        = %0d", out_c);
        $display("    撞拍次数              = %0d", col_c);
        $display("    最后一窗累加点数      = %0d", last_acc_c);
        $display("    最后输出 angle_raw    = %0d", cc);
        $display("    判定: %s", (cc==100) ? "正确" : "★ 偏低！");
        $display("================================================================");
        // 机读结论：三组参数都必须给出「每窗 16 点、输出满值 100」
        if (ca == 100 && cb == 100 && cc == 100)
            $display("TEST_RESULT: tb_adc_prod PASS  (a=%0d b=%0d c=%0d, 撞拍 %0d/%0d/%0d)",
                     ca, cb, cc, col_a, col_b, col_c);
        else
            $display("TEST_RESULT: tb_adc_prod FAIL  (a=%0d b=%0d c=%0d)", ca, cb, cc);
        $finish;
    end

    initial begin
        #300_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
