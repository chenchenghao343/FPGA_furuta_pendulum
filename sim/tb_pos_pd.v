//=============================================================================
// tb_pos_pd.v —— 验证 v5.8 新外环模块 pos_pd
//-----------------------------------------------------------------------------
// 设计原则：**只用与相位无关的判据**
//   外环分频器的相位取决于复位释放的时刻，任何"第 N 拍应该怎样"的绝对
//   拍号断言都是脆的（第一版 TB 就因此误报了两项 FAIL）。
//   这里一律改成"相邻两次事件之间的间隔"或"稳定段的每拍增量"。
//
// 用例清单（每项都写清"逻辑错了会怎样 FAIL"）：
//   T1  分频周期 ：线性斜坡喂 err_x(+1/拍)，phi_fast 只应每 DIV 拍变一次。
//                  DIV 没生效 -> 每拍都变，间隔 1 != 10，FAIL
//   T2  正向饱和 ：kp=1.0, err=1000 -> 钳 +250
//   T2b 负向饱和 ：err=-1000       -> 钳 -250
//   T3  防微分冲击：停机期间 err 顶在 8000，投入后 phi 必须仍为 0。
//                  没有 d_armed -> 第一个外环节拍算出 derr=8000，
//                  经 kd=1.0 放大后饱和到 250，FAIL
//   T4  D 取差口径：err 8000->8100，phi 应等于 100（= kd x Δerr / 4096），
//                  而不是 8100（说明取的是差值不是绝对值），
//                  也不是 0（说明 D 通路没接）
//   T5  D 与节拍  ：线性斜坡 +1/拍、kd=1.0 -> 每个外环节拍 Δerr = DIV，
//                  故 phi = DIV = 10。这条同时验证"D 用的是外环节拍差值"
//   T6  速率限幅  ：生产参数 BIAS_RATE=2，稳定段每拍必须恰好 +2
//   T7  生产实例饱和：大目标下 phi 停在 +250，不会越界
//=============================================================================
`timescale 1ns/1ps

module tb_pos_pd;

    reg clk = 0, rst_n = 0;
    reg ctrl_tick = 0;
    reg run_en = 0;
    reg signed [15:0] err_x = 0;
    reg signed [31:0] kp_x  = 0;
    reg signed [31:0] kd_x  = 0;

    wire signed [15:0] phi_prod;   // 生产参数：BIAS_MAX=250, BIAS_RATE=2
    wire signed [15:0] phi_fast;   // 不限速（BIAS_RATE=30000），单看 PD 标度

    integer errors = 0;
    integer i, n, d, bad, prev, peak;
    integer last_chg, interval;

    localparam integer DIV = 10;

    always #10 clk = ~clk;                 // 50MHz

    //-------------------------------------------------------------------------
    // 控制节拍发生器：每 TICK_CYC 个时钟一个 1 拍宽的脉冲
    //-------------------------------------------------------------------------
    localparam integer TICK_CYC = 20;
    integer tick_cnt = 0;
    always @(posedge clk) begin
        if (tick_cnt == TICK_CYC-1) begin
            tick_cnt  <= 0;
            ctrl_tick <= 1'b1;
        end else begin
            tick_cnt  <= tick_cnt + 1;
            ctrl_tick <= 1'b0;
        end
    end

    pos_pd #(
        .W(16), .GAIN_W(32), .SUM_W(56), .Q_FMT(12), .DIV(DIV),
        .BIAS_MAX(16'sd250), .BIAS_RATE(16'sd2)
        // ★ v7.2：外环积分已删除，本 TB 的 T1~T7 判据本来就是按纯 PD 写的，
        //   现在模块就是纯 PD，不再需要 I_EN=0 这种开关。
    ) dut_prod (
        .clk(clk), .rst_n(rst_n), .ctrl_tick(ctrl_tick), .run_en(run_en),
        .err_x(err_x), .kp_x(kp_x), .kd_x(kd_x),
        .phi_bias(phi_prod)                        // ★ v7.2：i_shift 端口已删除
    );

    pos_pd #(
        .W(16), .GAIN_W(32), .SUM_W(56), .Q_FMT(12), .DIV(DIV),
        .BIAS_MAX(16'sd250), .BIAS_RATE(16'sd30000)
    ) dut_fast (
        .clk(clk), .rst_n(rst_n), .ctrl_tick(ctrl_tick), .run_en(run_en),
        .err_x(err_x), .kp_x(kp_x), .kd_x(kd_x),
        .phi_bias(phi_fast)
    );

    //-------------------------------------------------------------------------
    // 推进一个 ctrl_tick。ctrl_tick 由 posedge clk 以非阻塞方式拉高，
    // 被测模块要到【下一个】posedge clk 才消费它，所以必须等两个沿再采样。
    //-------------------------------------------------------------------------
    task run_tick;
        begin
            @(posedge ctrl_tick);
            @(posedge clk);
            #1;
        end
    endtask

    task chk;
        input [255:0] name;
        input integer got;
        input integer exp;
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("  [FAIL] %0s  got=%0d  exp=%0d", name, got, exp);
            end else begin
                $display("  [ ok ] %0s  = %0d", name, got);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    initial begin
        rst_n = 0; run_en = 0; err_x = 0; kp_x = 0; kd_x = 0;
        repeat (5) @(posedge clk);
        // 复位期间先经历一个 ctrl_tick，保证分频器从 0 起算
        @(posedge ctrl_tick); @(posedge clk); #1;
        rst_n = 1;
        @(posedge clk); #1;

        $display("===== tb_pos_pd =====");

        //================= T1: 外环分频周期 =================
        // 线性斜坡 err_x +1/拍。外环每 DIV 拍才采一次样，故 phi_fast
        // 只应每 DIV 拍变化一次 —— 与复位相位无关。
        kp_x = 32'sd4096; kd_x = 32'sd0; err_x = 0; run_en = 1'b1;
        prev = phi_fast; last_chg = -1; bad = 0; n = 0;
        for (i = 0; i < 90; i = i + 1) begin
            run_tick;
            n = n + 1;
            if (phi_fast !== prev) begin
                if (last_chg >= 0) begin
                    interval = n - last_chg;
                    if (interval !== DIV) bad = bad + 1;
                end
                last_chg = n;
                prev = phi_fast;
            end
            err_x = err_x + 16'sd1;
        end
        chk("T1  外环相邻两次更新间隔 = DIV", bad, 0);
        // 斜坡跟随：滞后 = 等到下一个外环节拍(最多 DIV-1 拍) + 1 拍(bias_target_r 寄存)
        //           + 1 拍(phi 再寄存) = 最多 DIV+1
        d = err_x - phi_fast;
        $display("       [info] T1b: err_x=%0d phi_fast=%0d d=%0d", err_x, phi_fast, d);
        chk("T1b 斜坡跟随: 0 <= err_x-phi <= DIV+1", ((d >= 0) && (d <= DIV+1)) ? 1 : 0, 1);

        //================= T2: 饱和 =================
        err_x = 16'sd1000;
        for (i = 0; i < 30; i = i + 1) run_tick;
        chk("T2  正向饱和 -> +250", phi_fast, 250);

        err_x = -16'sd1000;
        for (i = 0; i < 30; i = i + 1) run_tick;
        chk("T2b 负向饱和 -> -250", phi_fast, -250);

        //================= T3: 投入瞬间无微分冲击 =================
        run_en = 1'b0; kp_x = 32'sd0; kd_x = 32'sd4096; err_x = 16'sd8000;
        for (i = 0; i < 25; i = i + 1) run_tick;
        chk("T3a 未投入时 phi 恒为 0", phi_fast, 0);

        run_en = 1'b1;
        for (i = 0; i < 25; i = i + 1) run_tick;   // 跨过两个外环节拍
        chk("T3b 投入后无假微分 -> 仍为 0", phi_fast, 0);

        //================= T4: D 取"外环节拍差值" =================
        // ★ 注意 D 是微分：err_x 阶跃之后只在【一个外环节拍】内给出一份
        //   与 Δerr 成正比的偏置，随后因为 Δerr 回到 0 而立刻衰减回 0。
        //   所以这里要抓的是"脉冲峰值"，不是稳态值。
        //   （第一版 TB 直接查 25 拍之后的稳态值，必然读到 0 —— 那是 TB 错。）
        err_x = 16'sd8100;                          // 相对上次外环采样 +100
        peak = 0;
        for (i = 0; i < 25; i = i + 1) begin
            run_tick;
            if (phi_fast > peak) peak = phi_fast;
        end
        chk("T4a D 脉冲峰值 = kd x dErr/4096 = 100", peak, 100);
        chk("T4b 阶跃过后 D 归零（是微分不是积分）", phi_fast, 0);

        //================= T5: D 与节拍的关系 =================
        // kp=0, kd=1.0, 斜坡 +1/拍 -> 每个外环节拍(10拍) Δerr = DIV
        // => phi = DIV x 1 = 10
        run_en = 1'b0; kp_x = 32'sd0; kd_x = 32'sd4096; err_x = 16'sd0;
        for (i = 0; i < 15; i = i + 1) run_tick;    // 停机清历史
        run_en = 1'b1;
        for (i = 0; i < 15; i = i + 1) run_tick;    // 武装并稳定
        for (i = 0; i < 60; i = i + 1) begin        // 斜坡
            run_tick;
            err_x = err_x + 16'sd1;
        end
        chk("T5  D 按外环节拍取差: 斜坡 -> phi = DIV", phi_fast, DIV);

        //================= T6: 速率限幅（生产参数） =================
        run_en = 1'b0;
        for (i = 0; i < 3; i = i + 1) run_tick;
        chk("T6a 未投入时生产实例归零", phi_prod, 0);

        kp_x = 32'sd4096; kd_x = 32'sd0; err_x = 16'sd0; run_en = 1'b1;
        prev = 0; bad = 0;
        for (i = 0; i < 80; i = i + 1) begin
            run_tick;
            d = phi_prod - prev;
            // 跳过起始的相位不定段：目标需求是每拍 +10，限速只给 +2，
            // 所以进入稳定段后每一拍的增量必须恰好是 2
            if (i >= 15 && d !== 2) bad = bad + 1;
            prev = phi_prod;
            err_x = err_x + 16'sd10;
        end
        chk("T6b 稳定段每拍增量恒为 BIAS_RATE(2)", bad, 0);

        //================= T7: 生产实例饱和 =================
        err_x = 16'sd1000;                          // 目标远超 250
        for (i = 0; i < 200; i = i + 1) run_tick;
        chk("T7  生产实例正向饱和 -> +250", phi_prod, 250);

        //================= 结果 =================
        $display("---------------------------------------");
        if (errors == 0) $display("  PASS: 全部用例通过");
        else             $display("  FAIL: %0d 项不通过", errors);
        if (errors == 0) $display("TEST_RESULT: tb_pos_pd PASS");
        else             $display("TEST_RESULT: tb_pos_pd FAIL  失败项数=%0d", errors);
        $display("=======================================");
        $finish;
    end

    // 兜底：跑飞时强制结束
    initial begin
        #20_000_000;
        $display("  [FAIL] 仿真超时");
        $finish;
    end

endmodule
