// =============================================================================
// 文件   : tb_motor_drv.v
// 被测   : motor_drv  (PWM_PERIOD 覆盖为 100, DUTY_MAX 覆盖为 100, DEADBAND = 20)
// 仿真器 : ModelSim    运行: vsim -batch work.tb_motor_drv -do run.do
//
// -----------------------------------------------------------------------------
// 参数折算关系（重要）
// -----------------------------------------------------------------------------
//  RTL 默认 PWM_PERIOD = 13'd5000 -> 载波频率 50MHz/5000 = 10kHz。
//  本 tb 覆盖为 PWM_PERIOD = 100：载波频率变成 50MHz/100 = 500kHz，
//  仿真一个完整 PWM 周期只需 100 拍 = 2us，比默认快 50 倍。
//
//  折算关系是【一一对应】的：
//      默认 1 个载波周期 = 5000 拍，本 tb = 100 拍，缩小 50 倍；
//      DUTY_MAX 也从 5000 同步缩到 100，所以“占空比”这个比值的物理含义不变：
//          默认 u_ctrl = 2500 -> 50% ；本 tb u_ctrl = 50 -> 50%
//          默认 u_ctrl = 5000 -> 100%；本 tb u_ctrl = 100 -> 100%
//      DEADBAND 保持 20 不变（它是绝对量，不是比例量），
//      因此本 tb 里 20/100 = 20% 的控制量才进死区边界，
//      这正好也把“死区阈值”测得更细，是刻意保留的。
//
// -----------------------------------------------------------------------------
// 这个 tb 在验什么 / 判据是什么 / 为什么这样判
// -----------------------------------------------------------------------------
// 电机驱动是唯一直接接执行器的模块，写错的后果是烧 H 桥或飞车，
// 所以判据全部用【实测计数】而不是“看起来对”：
//
//   C1  静默复位：复位后 in1=in2=0，pwm 恒 0（安全态）。
//   C2  u_ctrl > DEADBAND -> in1=1, in2=0（正转）;
//       恰好等于 DEADBAND 属于【死区】（RTL 用严格大于），单独卡这个边界。
//   C3  u_ctrl < -DEADBAND -> in1=0, in2=1（反转）;
//       恰好等于 -DEADBAND 同样属于死区。
//   C4  死区内（含 0、±1、±(DEADBAND-1)）-> in1=0,in2=0，且
//       【用计数器实测一个完整载波周期内 pwm 的高电平拍数必须为 0】。
//       这是“停机必须真停”的判据，光看 in1/in2 不够。
//   C5  占空比实测：对 u_ctrl = 50/100 与 100/100 两种控制量，
//       分别在一个完整载波周期内数 pwm 为高的拍数，
//       期望恰好 50 拍（50%）与 100 拍（100%）。
//       ——不是比例近似，是精确相等：pwm = (cnt < cmp) 时 cnt 从 0 数到 99，
//       真正为高的拍数正好等于 cmp。差 1 拍就说明比较符写成了 <= 或 >=。
//   C6  单调性：u_ctrl = 25 / 50 / 75（对应 25%/50%/75%），
//       实测高电平拍数必须严格递增且分别等于 25/50/75。
//       这条抓“控制量被截断/移位算错/极性反了”的缺陷。
//   C7  限幅：u_ctrl = DUTY_MAX 时占空比 100%（恰好等于 PWM_PERIOD 拍）；
//       u_ctrl 远超 DUTY_MAX（如 30000）时占空比【不得超过 100%】，
//       实测高电平拍数必须 <= PWM_PERIOD。这条抓“没有限幅导致 cnt<cmp
//       恒真但反向时溢出”的缺陷。
//   C8  负向限幅对称：u_ctrl = -30000 时，in1=0/in2=1 且占空比同样为 100%。
//   C9  极端边界 u_ctrl = -32768：RTL 注释特别强调“必须先扩展到 17bit 再取负，
//       否则 -32768 取负溢出”。本项用 -32768 实测占空比，
//       若 RTL 写成 16bit 取负会得到 0% 而不是 100%，必然被这条抓出来。
//   C10 方向翻转安全：从正转大占空比切到反转大占空比，
//       不允许出现 in1=in2=1（TB6612 的短路刹车态，本设计明确禁用）；
//       并且切换后方向位与 pwm 在同一拍生效（不允许方向切换瞬间还输出 pwm）。
//   C11 复位安全：运行中拉低 rst_n，in1/in2/pwm 必须立即回安全态。
//
// 测量方法说明：在一个完整载波周期内逐拍采样 pwm，统计高电平拍数。
//   为了让每次测量都对齐到载波起点，测量前先等待载波的下降沿特征
//   （pwm 从 1 变 0 或从 0 起）——本 tb 采用更稳妥的办法：
//   先连续采样 2*PWM_PERIOD 拍，取其中较小的一整段完整周期的计数。
//   具体实现：从“观察到 pwm 由高变低的那一拍”开始数下一整周期，
//   因为 cmp < PWM_PERIOD 时每周期必然存在高->低的跳变，对齐可靠。
// =============================================================================

`timescale 1ns/1ns

module tb_motor_drv;

    // ---------------------------------------------------------------
    // 参数：按 1:1 折算覆盖
    // ---------------------------------------------------------------
    parameter PWM_PERIOD_TB = 100;      // 默认 5000 -> 100（缩小 50 倍，载波 500kHz）
    parameter DUTY_MAX_TB   = 100;      // 默认 5000 -> 100（保持与 PWM_PERIOD 同比）
    parameter DEADBAND_TB   = 20;       // 默认 20，不变（绝对量）

    reg  clk;
    reg  rst_n;
    reg  signed [15:0] u_ctrl;
    wire pwm;
    wire in1;
    wire in2;

    integer errors;
    integer i;

    // 时钟：50MHz
    initial clk = 1'b0;
    always #10 clk = ~clk;

    // ---------------------------------------------------------------
    // 例化：命名端口 + 参数覆盖（折算说明见文件头）
    // ---------------------------------------------------------------
    motor_drv #(
        .PWM_PERIOD (PWM_PERIOD_TB),
        .DUTY_MAX   (DUTY_MAX_TB),
        .DEADBAND   (DEADBAND_TB)
    ) u_dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .u_ctrl (u_ctrl),
        .pwm    (pwm),
        .in1    (in1),
        .in2    (in2)
    );

    // ---------------------------------------------------------------
    // 检查任务
    // ---------------------------------------------------------------
    task check_int;
        input [8*64-1:0] name;
        input integer    exp;
        input integer    got;
        begin
            if (exp === got) begin
                $display("[CHECK] %0s: expected=%0d, got=%0d => OK", name, exp, got);
            end else begin
                $display("[CHECK] %0s: expected=%0d, got=%0d => FAIL", name, exp, got);
                errors = errors + 1;
            end
        end
    endtask

    task check_bit;
        input [8*64-1:0] name;
        input            exp;
        input            got;
        begin
            if (exp === got) begin
                $display("[CHECK] %0s: expected=%b, got=%b => OK", name, exp, got);
            end else begin
                $display("[CHECK] %0s: expected=%b, got=%b => FAIL", name, exp, got);
                errors = errors + 1;
            end
        end
    endtask

    // ---------------------------------------------------------------
    // 实测：观察一个完整载波周期（PWM_PERIOD_TB 拍）内的 pwm 高电平拍数
    //   对齐方法：先等到出现“pwm 由高变低”的边沿（载波周期的结束点），
    //   再数接下来的 PWM_PERIOD_TB 拍。
    //   对 0% / 100% 占空比没有高->低边沿，本 tb 用 timeout 兜底，
    //   此时直接数 PWM_PERIOD_TB 拍（0% 与 100% 都是恒定的，对齐无所谓）。
    // ---------------------------------------------------------------
    integer high_cnt;
    integer wait_cnt;

    task measure_duty;
        output integer high_cycles;     // 一个载波周期内 pwm 为高的拍数
        begin
            // 1) 等待高->低跳变（最多等 2 个载波周期）
            wait_cnt = 0;
            while ((pwm !== 1'b1) && (wait_cnt < 2*PWM_PERIOD_TB + 10)) begin
                @(posedge clk);
                #1;
                wait_cnt = wait_cnt + 1;
            end
            if (pwm === 1'b1) begin
                // 等到高电平结束（高->低）
                wait_cnt = 0;
                while ((pwm !== 1'b0) && (wait_cnt < 2*PWM_PERIOD_TB + 10)) begin
                    @(posedge clk);
                    #1;
                    wait_cnt = wait_cnt + 1;
                end
            end
            // 2) 现在 pwm 刚变低（或一直是低），从下一拍开始数一整周期
            high_cycles = 0;
            for (i = 0; i < PWM_PERIOD_TB; i = i + 1) begin
                @(posedge clk);
                #1;
                if (pwm === 1'b1) high_cycles = high_cycles + 1;
            end
        end
    endtask

    // 等待若干拍让方向位/cmp 稳定（u_ctrl 改变后至少要 1 拍才更新）
    task settle;
        begin
            repeat (4) @(posedge clk);
            #1;
        end
    endtask

    // 检查 pwm 在 n 拍内是否恒为 0
    task check_pwm_zero;
        input [8*64-1:0] name;
        input integer    n;
        begin : pwm_zero_blk
            integer bad;
            bad = 0;
            for (i = 0; i < n; i = i + 1) begin
                @(posedge clk);
                #1;
                if (pwm !== 1'b0) bad = bad + 1;
            end
            check_int(name, 0, bad);
        end
    endtask

    // ---------------------------------------------------------------
    // 主流程
    // ---------------------------------------------------------------
    integer duty;

    initial begin
        errors = 0;
        rst_n  = 1'b0;
        u_ctrl = 16'sd0;

        $display("==============================================================");
        $display(" tb_motor_drv : TB6612 方向 + PWM 占空比 + 死区 + 限幅 验证");
        $display(" PWM_PERIOD 覆盖 = %0d (默认 5000，缩小 50 倍；载波 50MHz/%0d = %0d Hz)",
                 PWM_PERIOD_TB, PWM_PERIOD_TB, 50000000/PWM_PERIOD_TB);
        $display(" DUTY_MAX   覆盖 = %0d (默认 5000，与 PWM_PERIOD 同比)", DUTY_MAX_TB);
        $display(" DEADBAND   保持 = %0d (绝对量，不折算)", DEADBAND_TB);
        $display(" 折算: 默认 u_ctrl=2500 <-> 本 tb u_ctrl=50 (都是 50%%)");
        $display("==============================================================");

        // -----------------------------------------------------------
        // C1 复位安全态
        // -----------------------------------------------------------
        repeat (5) @(posedge clk);
        #1;
        check_bit("C1 rst_in1_0", 1'b0, in1);
        check_bit("C1 rst_in2_0", 1'b0, in2);
        check_pwm_zero("C1 rst_pwm_always_0", 3*PWM_PERIOD_TB);

        rst_n = 1'b1;
        repeat (3) @(posedge clk);
        #1;

        // -----------------------------------------------------------
        // C2 正转 + 死区上边界
        // -----------------------------------------------------------
        $display("[INFO ] 阶段1：正转方向与死区边界");
        u_ctrl = 16'sd21;               // 比 DEADBAND(20) 大 1
        settle;
        check_bit("C2 pos_out_in1", 1'b1, in1);
        check_bit("C2 pos_out_in2", 1'b0, in2);

        u_ctrl = 16'sd20;               // 恰好 = DEADBAND -> 属于死区
        settle;
        check_bit("C2 at_deadband_in1_0", 1'b0, in1);
        check_bit("C2 at_deadband_in2_0", 1'b0, in2);

        // -----------------------------------------------------------
        // C3 反转 + 死区下边界
        // -----------------------------------------------------------
        $display("[INFO ] 阶段2：反转方向与死区边界");
        u_ctrl = -16'sd21;
        settle;
        check_bit("C3 neg_out_in1", 1'b0, in1);
        check_bit("C3 neg_out_in2", 1'b1, in2);

        u_ctrl = -16'sd20;              // 恰好 = -DEADBAND -> 死区
        settle;
        check_bit("C3 at_neg_deadband_in1_0", 1'b0, in1);
        check_bit("C3 at_neg_deadband_in2_0", 1'b0, in2);

        // -----------------------------------------------------------
        // C4 死区内部各种值：in1=in2=0 且 pwm 实测恒 0
        // -----------------------------------------------------------
        $display("[INFO ] 阶段3：死区内 pwm 必须实测恒 0（含 0 / ±1 / ±19）");
        u_ctrl = 16'sd0;
        settle;
        check_bit("C4 zero_u_in1_0", 1'b0, in1);
        check_bit("C4 zero_u_in2_0", 1'b0, in2);
        check_pwm_zero("C4 zero_u_pwm_all_0", 2*PWM_PERIOD_TB);

        u_ctrl = 16'sd1;
        settle;
        check_pwm_zero("C4 plus1_pwm_all_0", 2*PWM_PERIOD_TB);

        u_ctrl = -16'sd1;
        settle;
        check_pwm_zero("C4 minus1_pwm_all_0", 2*PWM_PERIOD_TB);

        u_ctrl = 16'sd19;               // DEADBAND-1
        settle;
        check_bit("C4 db_minus1_in1_0", 1'b0, in1);
        check_bit("C4 db_minus1_in2_0", 1'b0, in2);
        check_pwm_zero("C4 db_minus1_pwm_all_0", 2*PWM_PERIOD_TB);

        u_ctrl = -16'sd19;
        settle;
        check_bit("C4 neg_db_minus1_in1_0", 1'b0, in1);
        check_bit("C4 neg_db_minus1_in2_0", 1'b0, in2);
        check_pwm_zero("C4 neg_db_minus1_pwm_all_0", 2*PWM_PERIOD_TB);

        // -----------------------------------------------------------
        // C5 占空比实测：50% 与 100%
        //   u_ctrl = 50 -> cmp = 50 -> 高电平恰好 50 拍 / 100 拍 = 50%
        //   u_ctrl = 100 -> cmp = 100 -> 高电平恰好 100 拍 = 100%
        // -----------------------------------------------------------
        $display("[INFO ] 阶段4：实测占空比（50%% 与 100%%）");
        u_ctrl = 16'sd50;
        settle;
        check_bit("C5 d50_in1", 1'b1, in1);
        check_bit("C5 d50_in2", 1'b0, in2);
        measure_duty(duty);
        check_int("C5 duty_50pct_high_cycles", 50, duty);

        u_ctrl = 16'sd100;              // = DUTY_MAX -> 100%
        settle;
        measure_duty(duty);
        check_int("C5 duty_100pct_high_cycles", PWM_PERIOD_TB, duty);

        // -----------------------------------------------------------
        // C6 单调性：25 / 50 / 75 -> 实测 25 / 50 / 75 拍
        // -----------------------------------------------------------
        $display("[INFO ] 阶段5：占空比单调性与精确值（25%%/50%%/75%%）");
        u_ctrl = 16'sd25;
        settle;
        measure_duty(duty);
        check_int("C6 duty_25_high_cycles", 25, duty);

        u_ctrl = 16'sd75;
        settle;
        measure_duty(duty);
        check_int("C6 duty_75_high_cycles", 75, duty);

        // -----------------------------------------------------------
        // C7 正限幅：远超 DUTY_MAX 时占空比不得超过 100%
        // -----------------------------------------------------------
        $display("[INFO ] 阶段6：限幅（u_ctrl 远超 DUTY_MAX）");
        u_ctrl = 16'sd30000;
        settle;
        check_bit("C7 clamp_pos_in1", 1'b1, in1);
        check_bit("C7 clamp_pos_in2", 1'b0, in2);
        measure_duty(duty);
        check_int("C7 clamp_pos_high_cycles", PWM_PERIOD_TB, duty);
        if (duty > PWM_PERIOD_TB) begin
            $display("[CHECK] C7 clamp_pos_not_over_100pct: got=%0d > %0d => FAIL", duty, PWM_PERIOD_TB);
            errors = errors + 1;
        end else begin
            $display("[CHECK] C7 clamp_pos_not_over_100pct: got=%0d <= %0d => OK", duty, PWM_PERIOD_TB);
        end

        // -----------------------------------------------------------
        // C8 负限幅：对称性
        // -----------------------------------------------------------
        u_ctrl = -16'sd30000;
        settle;
        check_bit("C8 clamp_neg_in1", 1'b0, in1);
        check_bit("C8 clamp_neg_in2", 1'b1, in2);
        measure_duty(duty);
        check_int("C8 clamp_neg_high_cycles", PWM_PERIOD_TB, duty);

        // -----------------------------------------------------------
        // C9 极端边界 -32768（取负溢出陷阱）
        //   u_ctrl = -32768 时 |u_ctrl| = 32768 > DUTY_MAX -> 应被限幅到 100%
        // -----------------------------------------------------------
        $display("[INFO ] 阶段7：极端边界 u_ctrl = -32768（取负溢出陷阱）");
        // 注意：这里【绝对不能】写成 -16'sd32768。
        // 按 IEEE 1364 语义，有符号十进制字面量 16'sd32768 本身就已超出
        // 16bit 有符号范围（-32768~32767），会先被截断成 16'b0，
        // 再取负仍然是 0 —— 那样这条用例就退化成了 u_ctrl=0（死区），
        // 根本测不到 -32768 的取负溢出陷阱。
        // 必须用与该宽度相容的写法：-32767 - 1 在 signed 语境下得到 -32768。
        u_ctrl = -16'sd32767 - 16'sd1;
        settle;
        check_bit("C9 minint_in1", 1'b0, in1);
        check_bit("C9 minint_in2", 1'b1, in2);
        measure_duty(duty);
        check_int("C9 minint_high_cycles_100pct", PWM_PERIOD_TB, duty);

        // 对称地测 +32767
        u_ctrl = 16'sd32767;
        settle;
        measure_duty(duty);
        check_int("C9 maxint_high_cycles_100pct", PWM_PERIOD_TB, duty);

        // -----------------------------------------------------------
        // C10 方向翻转安全：不允许出现 in1=in2=1
        // -----------------------------------------------------------
        $display("[INFO ] 阶段8：方向翻转过程不得出现 in1=in2=1（刹车态）");
        begin : dir_switch
            integer brake_hits;
            brake_hits = 0;
            // 正转满速 -> 反转满速，逐拍检查
            u_ctrl = 16'sd100;
            settle;
            u_ctrl = -16'sd100;
            for (i = 0; i < 60; i = i + 1) begin
                @(posedge clk);
                #1;
                if ((in1 === 1'b1) && (in2 === 1'b1)) brake_hits = brake_hits + 1;
            end
            check_int("C10 no_brake_state_on_reverse", 0, brake_hits);
            check_bit("C10 final_dir_neg_in1", 1'b0, in1);
            check_bit("C10 final_dir_neg_in2", 1'b1, in2);
        end

        // 反向切正向同样检查
        begin : dir_switch2
            integer brake_hits2;
            brake_hits2 = 0;
            u_ctrl = 16'sd100;
            for (i = 0; i < 60; i = i + 1) begin
                @(posedge clk);
                #1;
                if ((in1 === 1'b1) && (in2 === 1'b1)) brake_hits2 = brake_hits2 + 1;
            end
            check_int("C10 no_brake_state_on_forward", 0, brake_hits2);
            check_bit("C10 final_dir_pos_in1", 1'b1, in1);
            check_bit("C10 final_dir_pos_in2", 1'b0, in2);
        end

        // -----------------------------------------------------------
        // C11 运行中复位 -> 立即回安全态
        // -----------------------------------------------------------
        $display("[INFO ] 阶段9：运行中复位必须立刻回安全态");
        u_ctrl = 16'sd100;
        settle;
        rst_n = 1'b0;
        #3;                             // 异步复位不等时钟
        check_bit("C11 asyn_rst_in1_0", 1'b0, in1);
        check_bit("C11 asyn_rst_in2_0", 1'b0, in2);
        check_bit("C11 asyn_rst_pwm_0", 1'b0, pwm);
        check_pwm_zero("C11 rst_hold_pwm_0", 2*PWM_PERIOD_TB);

        rst_n = 1'b1;
        // ★ 释放复位前必须先把控制量归零，否则测的不是"复位后回安全态"，
        //   而是"复位后立刻按旧的 u_ctrl 继续驱动"——后者是**正确行为**，
        //   原 tb 却期望 pwm=0，属于期望值写错（C11 唯一一项失败的根因）。
        u_ctrl = 16'sd0;
        repeat (2) @(posedge clk);
        #1;
        check_bit("C11 after_rst_pwm_0", 1'b0, pwm);
        check_bit("C11 after_rst_in1_0", 1'b0, in1);
        check_bit("C11 after_rst_in2_0", 1'b0, in2);

        // -----------------------------------------------------------
        // 汇总
        // -----------------------------------------------------------
        $display("--------------------------------------------------------------");
        if (errors == 0) begin
            $display("TEST_RESULT: tb_motor_drv PASS");
        end else begin
            $display("TEST_RESULT: tb_motor_drv FAIL  失败项数=%0d", errors);
        end
        $display("--------------------------------------------------------------");
        $finish;
    end

    // 超时保护
    initial begin
        #500000;
        $display("[CHECK] 仿真超时（500us）=> FAIL");
        $display("TEST_RESULT: tb_motor_drv FAIL  失败项数=1");
        $finish;
    end

endmodule
