//=============================================================================
// 文件名  : tb_swing_pump.v
// 模块名  : tb_swing_pump
// 项目    : J280 旋转倒立摆 FPGA 控制系统
// 功能    : 起摆能量泵 自检 testbench
//-----------------------------------------------------------------------------
// 验证点
//   T1  run_en=0：u_swing=0、captured=0（安全停机）
//   T2  捕获判据：角度与角速度**同时**落带内才 captured（"与"逻辑）
//        T2a 两者都在带内 → captured=1，且泵停止输出
//        T2b 角度在带内但角速度超限 → 不捕获（防高速掠过顶部误判）
//        T2c 角速度在带内但角度超限 → 不捕获
//   T3  捕获带边界：恰好 |φ|=130 / |θ̇|=55 不捕获（严格小于）
//   T4  能量泵方向律：a 与 sign(θ̇·cosθ) 反向
//        T4a |φ|=1000 (cos≈+24), θ̇=+100 → 积为正 → u=-SWING_DUTY
//        T4b |φ|=1000, θ̇=-100 → 积为负 → u=+SWING_DUTY
//        T4c |φ|=2000 (cos≈-976，已过 90°), θ̇=+100 → 积为负 → u=+SWING_DUTY
//        T4d |φ|>=2048 (cos 饱和到 -1024), θ̇=+100 → u=+SWING_DUTY
//        ★ T4c/T4d 证明 cos 变号会导致泵方向翻转，这是起摆"来回增幅"的来源
//   T5  起摆死区踢摆：|θ̇| < KICK_MIN_DTH(1) 时改用固定方向踢摆
//       ★ 含量级回归用例（T5b~T5f），锁死 KICK_MIN_DTH 必须 ≪ 4
//   T6  极性参数：PUMP_POLARITY=-1 时输出整体取反（上板纠极性用）
//=============================================================================

`timescale 1ns/1ns

module tb_swing_pump;

    localparam integer SWING_DUTY   = 2600;
    localparam integer CAPTURE_BAND = 130;
    localparam integer CAPTURE_RATE = 55;
    localparam integer KICK_DUTY    = 1400;

    reg               clk, rst_n, tick, run_en;
    reg signed [15:0] phi, dtheta;

    wire signed [15:0] u_swing_p, u_swing_n;
    wire               cap_p, cap_n;

    integer fail_cnt, check_cnt;

    task check_i;
        input [255:0] name;
        input integer expected;
        input integer got;
        begin
            check_cnt = check_cnt + 1;
            if (expected === got) begin
                $display("[CHECK] %0s: expected=%0d, got=%0d => OK", name, expected, got);
            end else begin
                $display("[CHECK] %0s: expected=%0d, got=%0d => FAIL", name, expected, got);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    task tick_once;
        begin
            tick = 1'b0; @(posedge clk);
            tick = 1'b1; @(posedge clk);
            tick = 1'b0; @(posedge clk);
        end
    endtask

    initial clk = 1'b0;
    always #10 clk = ~clk;

    // 正极性实例
    swing_pump #(
        .W(16),  .CAPTURE_BAND(16'sd130),
        .CAPTURE_RATE(16'sd55), .KICK_MIN_DTH(16'sd1), .KICK_DUTY(16'sd1400),
        .KICK_DIR(1), .PUMP_POLARITY(1)
    ) dut_p (
        .clk(clk), .rst_n(rst_n), .tick(tick), .run_en(run_en),
        .phi(phi), .dtheta(dtheta), .swing_duty(16'sd2600),
        .u_swing(u_swing_p), .captured(cap_p)
    );

    // 负极性实例（验证 PUMP_POLARITY 参数有效）
    swing_pump #(
        .W(16),  .CAPTURE_BAND(16'sd130),
        .CAPTURE_RATE(16'sd55), .KICK_MIN_DTH(16'sd1), .KICK_DUTY(16'sd1400),
        .KICK_DIR(1), .PUMP_POLARITY(-1)
    ) dut_n (
        .clk(clk), .rst_n(rst_n), .tick(tick), .run_en(run_en),
        .phi(phi), .dtheta(dtheta), .swing_duty(16'sd2600),
        .u_swing(u_swing_n), .captured(cap_n)
    );

    initial begin
        $display("================================================================");
        $display(" tb_swing_pump 开始");
        $display("================================================================");

        fail_cnt = 0; check_cnt = 0;
        rst_n = 1'b0; tick = 1'b0; run_en = 1'b0;
        phi = 16'sd0; dtheta = 16'sd0;
        #100; rst_n = 1'b1; #100;

        //---------------------------------------------------------------------
        // T1 停机
        //---------------------------------------------------------------------
        run_en = 1'b0; phi = 16'sd1000; dtheta = 16'sd200;
        tick_once();
        check_i("T1a 停机 u_swing=0", 0, $signed(u_swing_p));
        check_i("T1b 停机 captured=0", 0, cap_p);

        run_en = 1'b1;

        //---------------------------------------------------------------------
        // T2 捕获判据（角度与角速度必须同时落带内）
        //---------------------------------------------------------------------
        phi = 16'sd100; dtheta = 16'sd10;        // 都在带内
        tick_once();
        check_i("T2a 两者带内 → captured=1", 1, cap_p);
        check_i("T2a2 捕获后泵停止输出", 0, $signed(u_swing_p));

        phi = 16'sd100; dtheta = 16'sd200;       // 角度带内、角速度超限
        tick_once();
        check_i("T2b 角速度超限 → 不捕获", 0, cap_p);

        phi = 16'sd1000; dtheta = 16'sd10;       // 角度超限、角速度带内
        tick_once();
        check_i("T2c 角度超限 → 不捕获", 0, cap_p);

        //---------------------------------------------------------------------
        // T3 捕获带边界（严格小于）
        //---------------------------------------------------------------------
        phi = 16'sd129; dtheta = 16'sd54;
        tick_once();
        check_i("T3a 边界内(129,54) → captured=1", 1, cap_p);

        phi = 16'sd130; dtheta = 16'sd10;        // 恰等于 BAND，不满足 "<"
        tick_once();
        check_i("T3b 角度恰=130 → 不捕获", 0, cap_p);

        phi = 16'sd10; dtheta = 16'sd55;         // 恰等于 RATE
        tick_once();
        check_i("T3c 角速度恰=55 → 不捕获", 0, cap_p);

        //---------------------------------------------------------------------
        // T4 能量泵方向律
        //   cos_approx = clamp(1024 - |phi|, -1024, 1024)
        //   PUMP_POLARITY=+1 时： sign(θ̇·cos) >= 0 → u = -SWING_DUTY
        //                         sign(θ̇·cos) <  0 → u = +SWING_DUTY
        //---------------------------------------------------------------------
        phi = 16'sd1000; dtheta = 16'sd3;        // ★真实量级(原为100)：|φ|=1000 → cos=+24, 积=+72
        tick_once();
        check_i("T4a |φ|=1000,θ̇=+3(真实量级) → u=-2600", -2600, $signed(u_swing_p));

        phi = 16'sd1000; dtheta = -16'sd3;       // ★真实量级：积 = -72
        tick_once();
        check_i("T4b |φ|=1000,θ̇=-3(真实量级) → u=+2600", 2600, $signed(u_swing_p));

        phi = 16'sd2000; dtheta = 16'sd4;        // ★真实量级：|φ|=2000 → cos=-976
        tick_once();
        check_i("T4c |φ|=2000(过90°),θ̇=+4 → u=+2600（cos变号）",
                2600, $signed(u_swing_p));

        phi = 16'sd2100; dtheta = 16'sd4;        // |φ|>=2048 → cos 饱和 -1024
        tick_once();
        check_i("T4d |φ|>=2048,θ̇=+4 → u=+2600", 2600, $signed(u_swing_p));

        phi = -16'sd2000; dtheta = 16'sd4;       // 负角度：|φ|=2000 → cos=-976
        tick_once();
        check_i("T4e φ=-2000,θ̇=+4 → u=+2600（取绝对值计算量）",
                2600, $signed(u_swing_p));

        //---------------------------------------------------------------------
        // T5 ★★ 起摆死区踢摆 + 量级回归（本条是上板缺陷的回归测试）★★
        //
        //   ★ 真实量级说明：dtheta 的单位是「每控制周期的角度增量(ADC counts)」，
        //       dtheta = ω × Ts × S_TH = ω × 0.89
        //     摆杆在下垂点摆动 ±30° 时 ω_max≈4.2 rad/s → dtheta 峰值仅 ≈±3.8。
        //     因此 KICK_MIN_DTH 必须远小于 4，否则方向判据永不生效、
        //     电路会一直停在固定方向的踢摆分支（上板表现：摆臂单向连续转圈）。
        //     本组用例专门锁死这一点。
        //---------------------------------------------------------------------
        phi = 16'sd1000; dtheta = 16'sd0;        // 真正静止：差分恰为 0
        tick_once();
        check_i("T5a θ̇=0（真静止）→ 踢摆 u=+1400", 1400, $signed(u_swing_p));

        phi = 16'sd1000; dtheta = 16'sd1;        // 最小可分辨非零值
        tick_once();
        check_i("T5b ★θ̇=1(最小非零) → 必须走能量泵 u=-2600", -2600, $signed(u_swing_p));

        phi = 16'sd2000; dtheta = 16'sd3;        // 典型起摆量级，下垂侧
        tick_once();
        check_i("T5c ★θ̇=3(典型起摆) → 能量泵 u=+2600", 2600, $signed(u_swing_p));

        phi = 16'sd2000; dtheta = -16'sd3;       // 反向
        tick_once();
        check_i("T5d ★θ̇=-3 → 能量泵 u=-2600（方向翻转）", -2600, $signed(u_swing_p));

        phi = 16'sd2000; dtheta = -16'sd1;
        tick_once();
        check_i("T5e ★θ̇=-1 → 能量泵 u=-2600", -2600, $signed(u_swing_p));

        // ★ 这条是真正能抓住"KICK_MIN_DTH 设成 8"的用例：
        //   若死区 ≥4，下面这个典型的起摆差分会被误判为静止而走踢摆分支。
        phi = 16'sd2000; dtheta = 16'sd4;
        tick_once();
        check_i("T5f ★★θ̇=4(起摆上限) 不得落入死区 → 能量泵 u=+2600",
                2600, $signed(u_swing_p));

        //---------------------------------------------------------------------
        // T6 极性参数
        //---------------------------------------------------------------------
        phi = 16'sd1000; dtheta = 16'sd3;
        tick_once();
        check_i("T6a PUMP_POLARITY=+1 → u=-2600", -2600, $signed(u_swing_p));
        check_i("T6b PUMP_POLARITY=-1 → u=+2600（极性取反）",
                2600, $signed(u_swing_n));
        check_i("T6c 两实例 captured 一致", cap_p, cap_n);

        //---------------------------------------------------------------------
        $display("================================================================");
        $display(" 检查点总数 = %0d，失败 = %0d", check_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("TEST_RESULT: tb_swing_pump PASS");
        else
            $display("TEST_RESULT: tb_swing_pump FAIL  失败项数=%0d", fail_cnt);
        $display("================================================================");
        $finish;
    end

    initial begin
        #500000;
        $display("TEST_RESULT: tb_swing_pump FAIL  失败项数=999 (TIMEOUT)");
        $finish;
    end

endmodule
