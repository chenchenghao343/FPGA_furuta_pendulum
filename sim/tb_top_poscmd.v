//=============================================================================
// tb_top_poscmd.v  --  v6.3 端到端：真实 UART 发位置指令 → x_settle 变化
//-----------------------------------------------------------------------------
// 这个 tb 要证的是【整条链路】，不是单个模块：
//
//   真实 9600 UART 字节  ->  uart_rx  ->  pid_tuner
//                                      ->  pos_cmd（'M' 进入模式、编辑、'!' 发送）
//                                      ->  traj_ramp.delta_ext（锁存）
//                                      ->  x_settle 变化
//                                      ->  pos_ctrl -> 外环
//
// ★★ 必须验的核心风险（v6.2 就是栽在这里）★★
//   命令是 1 拍脉冲（20ns），而 traj_ramp 每 3ms（加速后 3.2us）才动一次。
//   如果脉冲没被锁存，命中概率只有 0.6%，表现就是"发了命令没反应"。
//   本 tb 刻意【不把脉冲对准 tick】，让它落在任意相位上 —— 这才是真实情况。
//   与 tb_traj_ramp 的旧写法（把脉冲架在 tick 沿上）相反。
//
// ★★ 还要验的第二个风险：模式门控 ★★
//   进入位置模式后调数值，PID 系数【必须不变】。
//   否则用户以为在调"要走多远"，其实把 kd_x 也改了 —— 隐性破坏。
//
// 仿真加速：只缩 angle_adc 的窗口（9/159），保持每窗 16 点结构。
//   UART 保持真实 9600（加速会破坏 uart_tx 的 13bit baud_cnt 行为）。
//   按键消抖保持真实 20ms。
//=============================================================================
`timescale 1ns/1ps

module tb_top_poscmd;

    localparam integer BIT_CNT = 5208;          // 50MHz/9600
    localparam integer BIT_NS  = 104160;        // 一拍 = 5208 拍 x 20ns
    localparam integer CNT_MAX_REAL = 20'd1_000_000;   // 20ms 消抖

    reg        clk_50m = 0, rst_n = 0;
    reg  [3:0] key_sw = 4'b1111;
    wire       uart_tx_pin;
    reg        uart_rx_pin = 1'b1;
    wire       adc_clk, adc_oe_n;
    reg  [9:0] adc_data_in = 10'd736;      // 736*4=2944, phi=+39 -> 能进 BALANCE
    wire       PWMA, AN1, AN2, PWMB, BN1, BN2;
    reg        enc_a = 0, enc_b = 0, enc2_a = 0, enc2_b = 0;
    wire [3:0] led;

    integer errors = 0, checks = 0;

    always #10 clk_50m = ~clk_50m;

    pendulum_top dut (
        .clk_50m(clk_50m), .rst_n(rst_n), .key_sw(key_sw),
        .uart_tx(uart_tx_pin), .uart_rx(uart_rx_pin),
        .adc_clk(adc_clk), .adc_oe_n(adc_oe_n), .adc_data_in(adc_data_in),
        .enc_a(enc_a), .enc_b(enc_b),
        .PWMA(PWMA), .AN1(AN1), .AN2(AN2),
        .PWMB(PWMB), .BN1(BN1), .BN2(BN2),
        .enc2_a(enc2_a), .enc2_b(enc2_b),
        // ★ v6.6 新增端口：外部 6 键置"不按"，OLED 两线留空
        .ext_key(5'b11111), .ext_rst_key(1'b1), .oled_scl(), .oled_sda(),
        .led(led)
    );

    // 仿真加速（只动角度窗口，保持每窗 16 点）
    defparam dut.u_angle_adc.SMP_STEP = 14'd9;
    defparam dut.u_angle_adc.SMP_3MS  = 18'd159;

    task chk;
        input [255:0] name;
        input integer exp, got;
        begin
            checks = checks + 1;
            if (exp === got)
                $display("[CHECK] %0s: expected=%0d, got=%0d => OK", name, exp, got);
            else begin
                $display("[CHECK] %0s: expected=%0d, got=%0d => FAIL", name, exp, got);
                errors = errors + 1;
            end
        end
    endtask

    // 真实 9600 发一个字节（对齐 negedge，避免与采样沿竞争）
    task uart_send_byte;
        input [7:0] d;
        integer k;
        begin
            @(negedge clk_50m);
            uart_rx_pin = 1'b0;                      // 起始位
            repeat (BIT_CNT) @(posedge clk_50m);
            for (k = 0; k < 8; k = k + 1) begin
                @(negedge clk_50m);
                uart_rx_pin = d[k];                  // LSB first
                repeat (BIT_CNT) @(posedge clk_50m);
            end
            @(negedge clk_50m);
            uart_rx_pin = 1'b1;                      // 停止位
            repeat (BIT_CNT) @(posedge clk_50m);
        end
    endtask

    // 发一串字符
    task uart_send_str;
        input [255:0] s;
        input integer n;
        integer k;
        reg [7:0] c;
        begin
            for (k = 0; k < n; k = k + 1) begin
                c = s[8*(n-1-k) +: 8];               // 从左到右取字符
                uart_send_byte(c);
            end
        end
    endtask

    // 记录 x_settle 出现过的最大值/最小值，用来判"有没有真的动"
    integer xs_max, xs_min, xs_last;
    always @(posedge clk_50m) begin
        if ($signed(dut.x_settle) > xs_max) xs_max = $signed(dut.x_settle);
        if ($signed(dut.x_settle) < xs_min) xs_min = $signed(dut.x_settle);
        xs_last = $signed(dut.x_settle);
    end

    // 抓发送脉冲，确认它真的产生过（否则是上位机/解析问题，不是锁存问题）
    integer send_seen, send_width, sw_tmp;
    reg sp_prev;
    always @(posedge clk_50m) begin
        if (dut.pos_send_pulse) begin
            send_seen = send_seen + 1;
            sw_tmp = sp_prev ? (sw_tmp + 1) : 1;
            if (sw_tmp > send_width) send_width = sw_tmp;
            sp_prev = 1'b1;
        end else begin
            sw_tmp = 0;
            sp_prev = 1'b0;
        end
    end

    integer kd_x_before, kp_x_before;
    integer i;
    initial begin
        xs_max = 0; xs_min = 0; xs_last = 0;
        send_seen = 0; send_width = 0; sw_tmp = 0; sp_prev = 0;

        $display("================================================================");
        $display(" tb_top_poscmd : 真实 UART -> pos_cmd -> traj_ramp -> x_settle");
        $display(" 加速: 仅 angle_adc 窗口 (9/159)；UART 与按键消抖用真实参数");
        $display("================================================================");

        rst_n = 1'b0;
        #200;
        rst_n = 1'b1;
        repeat (600) @(posedge clk_50m); #1;

        //-----------------------------------------------------------------
        // ★ T0 先按 SW1 启动
        //   踩过的坑：第一版 tb 忘了按 SW1，FSM 一直停在 IDLE(mode=0)，
        //   于是 T0 断言 BALANCE 失败。而 x_settle 仍然会正确更新
        //   —— 因为 traj_ramp 的累加器【不受 run_en 门控】，
        //   只有 x_cmd(斜坡) / x_final(定点投入) 受门控。
        //   所以"x_settle 变了"不能证明"系统在平衡态"，
        //   这两件事必须分别验。本 tb 两个都验。
        //-----------------------------------------------------------------
        @(negedge clk_50m);
        key_sw[0] = 1'b0;                                   // SW1 按下
        repeat (CNT_MAX_REAL + 2000) @(posedge clk_50m);     // 真实 20ms 消抖
        @(negedge clk_50m);
        key_sw[0] = 1'b1;                                   // 释放
        repeat (CNT_MAX_REAL + 2000) @(posedge clk_50m);
        repeat (400) @(posedge clk_50m); #1;

        $display("  [INFO] 上电后 mode=%0d (2=BALANCE)  phi=%0d  x_settle=%0d",
                 dut.mode, $signed(dut.phi), $signed(dut.x_settle));
        chk("T0 已进入 BALANCE", 2, dut.mode);
        chk("T0b BALANCE 时 run_pos = 1", 1, dut.run_pos);

        //-----------------------------------------------------------------
        // T1 未进入位置模式时按编辑键：pos_pend 必须保持 0
        //-----------------------------------------------------------------
        uart_send_byte(8'h3E);   // '>'  （此时不在位置模式）
        uart_send_byte(8'h2B);   // '+'
        repeat (200) @(posedge clk_50m); #1;
        chk("T1a 未进模式 '>' 不改待发送值", 0, $signed(dut.pos_pend));
        chk("T1b 未进模式 '+' 不改待发送值", 0, $signed(dut.pos_pend));
        // ★ v6.4 回归点：进 'M' 之前，定点闸必须是关的
        //   （关着的时候 x_final 恒为 0 —— 这正是缺陷时用户看到的现象）
        chk("T1c 未进模式时 pos_mode_eff = 0", 0, dut.pos_mode_eff);
        chk("T1d 未进模式时 x_final = 0", 0, $signed(dut.x_final));

        //-----------------------------------------------------------------
        // T2 'M' 进入位置指令模式
        //-----------------------------------------------------------------
        uart_send_byte(8'h4D);   // 'M'
        repeat (200) @(posedge clk_50m); #1;
        chk("T2 'M' 后 sel_loop = 6", 6, dut.sel_loop);
        chk("T2b 位置指令模式已使能", 1, dut.pos_cmd_active);
        //-----------------------------------------------------------------
        // ★★ v6.4 缺陷回归断言（v6.3 就是漏了这两条，缺陷才溜过去）★★
        //   位置环真正追的是 x_final = pos_mode_eff ? x_cmd : 0。
        //   v6.3 里 'M' 只改了 sel_loop，没把 pos_mode 拉高，
        //   于是 x_final 恒为 0 —— 表现为"发了定点但摆臂不动"。
        //   这里直接把闸和 x_final 都断言出来，堵住这个缺口。
        //-----------------------------------------------------------------
        chk("T2c ★ 'M' 必须打开定点闸 pos_mode_eff = 1", 1, dut.pos_mode_eff);
        chk("T2d ★ 'M' 后 tuner_pos_mode = 1", 1, dut.u_pid_tuner.pos_mode);

        //-----------------------------------------------------------------
        // T3 记录当前系数，然后在位置模式下调数值
        //    ★ 调完之后系数必须【一个都没变】
        //-----------------------------------------------------------------
        kd_x_before = $signed(dut.u_pid_tuner.kd_x);
        kp_x_before = $signed(dut.u_pid_tuner.kp_x);

        //   用 '+' 十次 = 640，再用 '*' 一次 = 1280
        for (i = 0; i < 10; i = i + 1) uart_send_byte(8'h2B);   // '+'
        uart_send_byte(8'h2A);                                   // '*'
        repeat (400) @(posedge clk_50m); #1;
        $display("  [INFO] 调完之后 pos_pend = %0d", $signed(dut.pos_pend));
        chk("T3a 待发送值 = 1280", 1280, $signed(dut.pos_pend));
        chk("T3b ★ 位置模式下 kd_x 未被改动", kd_x_before,
            $signed(dut.u_pid_tuner.kd_x));
        chk("T3c ★ 位置模式下 kp_x 未被改动", kp_x_before,
            $signed(dut.u_pid_tuner.kp_x));

        //-----------------------------------------------------------------
        // T4 发 '!'：x_settle 应增加 1280
        //    ★ 关键：脉冲落在任意相位，验证锁存有效
        //-----------------------------------------------------------------
        xs_max = $signed(dut.x_settle);
        uart_send_byte(8'h21);   // '!'
        repeat (2000) @(posedge clk_50m); #1;

        $display("  [INFO] 发送后 x_settle = %0d  (send 脉冲出现 %0d 次, 宽 %0d 拍)",
                 $signed(dut.x_settle), send_seen, send_width);
        chk("T4a 发送脉冲恰好 1 次", 1, send_seen);
        chk("T4b 发送脉冲宽 1 拍", 1, send_width);
        chk("T4c ★ x_settle 增加了 1280", 1280, $signed(dut.x_settle));
        chk("T4d 发送后待发送值清 0", 0, $signed(dut.pos_pend));
        //-----------------------------------------------------------------
        // ★★ v6.4 缺陷回归：闸开了之后 x_final 必须真的跟随 x_cmd ★★
        //   v6.3 的 tb 只查到 x_settle 变对了就收工，
        //   没查"位置环实际拿到的目标" —— 而那才是决定摆臂动不动的东西。
        //
        //   ★ 等待拍数要按 tb 自己的节拍算，不能拍脑袋：
        //     本 tb 用 defparam 把 SMP_3MS 改成 159，所以控制节拍 = 160 拍
        //     （真实设计是 150000 拍 = 3ms）。x_cmd 每节拍走 SLEW counts，
        //     从 0 爬到 1280 需要 1280/SLEW 个节拍。
        //     第一版这里只写了 repeat(4000)（= 25 个节拍），x_cmd 才爬到 424，
        //     断言 T4e 因此误报失败 —— 是测试等得不够，不是设计错。
        //
        //   ★★ v6.7：SLEW 由 8 降到 2 后这里【又栽了一次】：
        //     固定等 40000 拍只够 SLEW=8（160 节拍×160 拍=25600）；
        //     SLEW=2 需要 640 节拍×160 拍=102400 拍，实测 x_cmd 只爬到 556，
        //     T4e 报 FAIL —— 同样是"测试等得不够"，不是设计错。
        //     现改成【直接读 DUT 里真实的 SLEW 参数来算】，乘 1.6 倍余量，
        //     以后 SLEW 再改（1/2/3/4/8…）都不会再误报。
        //-----------------------------------------------------------------
        begin : wait_ramp_done
            integer need_clk;
            begin
                need_clk = (1280 / dut.u_traj_ramp.SLEW) * 160 * 16 / 10;  // 1.6 倍余量
                if (need_clk < 40000) need_clk = 40000;
                $display("  [INFO] SLEW=%0d -> 等 %0d 拍让 x_cmd 爬到 1280",
                         dut.u_traj_ramp.SLEW, need_clk);
                repeat (need_clk) @(posedge clk_50m);
                #1;
            end
        end
        $display("  [INFO] 斜坡走完后 x_cmd = %0d  x_final = %0d",
                 $signed(dut.x_cmd), $signed(dut.x_final));
        chk("T4e ★ x_cmd 斜坡到位 = 1280", 1280, $signed(dut.x_cmd));
        chk("T4f ★ x_final 跟随 x_cmd（这是位置环真正用的目标）",
            $signed(dut.x_cmd), $signed(dut.x_final));

        //-----------------------------------------------------------------
        // T5 再发一次不同数值，验证可重复
        //-----------------------------------------------------------------
        for (i = 0; i < 5; i = i + 1) uart_send_byte(8'h2B);   // '+' x5 = 320
        uart_send_byte(8'h21);                                  // '!'
        repeat (2000) @(posedge clk_50m); #1;
        $display("  [INFO] 第二次发送后 x_settle = %0d", $signed(dut.x_settle));
        chk("T5 ★ x_settle 累加到 1600", 1600, $signed(dut.x_settle));
        chk("T5b 发送脉冲累计 2 次", 2, send_seen);

        //-----------------------------------------------------------------
        // T6 负方向
        //-----------------------------------------------------------------
        uart_send_byte(8'h2D);   // '-' 取反 -> 待发送 = 320 -> -320? (已清0, 先加)
        //   清 0 状态下 '-' 仍是 0，所以先 + 再加
        for (i = 0; i < 5; i = i + 1) uart_send_byte(8'h2B);   // 320
        uart_send_byte(8'h2D);                                   // 取反 -> -320
        repeat (200) @(posedge clk_50m); #1;
        $display("  [INFO] 取反后 pos_pend = %0d", $signed(dut.pos_pend));
        chk("T6a 取反后待发送值 = -320", -320, $signed(dut.pos_pend));
        uart_send_byte(8'h21);                                   // '!'
        repeat (2000) @(posedge clk_50m); #1;
        $display("  [INFO] 负向发送后 x_settle = %0d", $signed(dut.x_settle));
        chk("T6b ★ x_settle 减到 1280", 1280, $signed(dut.x_settle));

        //-----------------------------------------------------------------
        // T7 行程限幅：连发大值应停在 TURN_LIMIT
        //   ★ v6.4：一圈由 1024 改为 1067，所以 TURN_LIMIT/POS_DELTA_MAX
        //     由 3072 改为 3201（仍是 3 圈）。这条断言跟着改。
        //     注意它断言的是【DUT 的实际限幅值】——本 tb 实例化的是真顶层
        //     pendulum_top，所以固件里改了这个数，这里必须同步，
        //     否则会以"测试失败"的形式暴露出来（这正是它该有的行为）。
        //     累计送 10×640 = 6400 > 3201，必定被夹住。
        //-----------------------------------------------------------------
        for (i = 0; i < 10; i = i + 1) begin
            uart_send_byte(8'h2B); uart_send_byte(8'h2B); uart_send_byte(8'h2B);
            uart_send_byte(8'h2B); uart_send_byte(8'h2B); uart_send_byte(8'h2B);
            uart_send_byte(8'h2B); uart_send_byte(8'h2B); uart_send_byte(8'h2B);
            uart_send_byte(8'h2B);   // 10 次 '+' = 640
            uart_send_byte(8'h21);   // 发送
        end
        repeat (4000) @(posedge clk_50m); #1;
        $display("  [INFO] 连续发送后 x_settle = %0d", $signed(dut.x_settle));
        chk("T7 ★ x_settle 被限幅在 +3201", 3201, $signed(dut.x_settle));

        //-----------------------------------------------------------------
        // T8 '1' 退出位置模式，编辑键恢复改系数
        //-----------------------------------------------------------------
        uart_send_byte(8'h31);   // '1' -> sel_loop = 0 (KP)
        repeat (200) @(posedge clk_50m); #1;
        chk("T8a '1' 退出位置模式", 0, dut.pos_cmd_active);

        $display("");
        $display("================================================================");
        $display(" checks = %0d, failures = %0d", checks, errors);
        if (errors == 0) $display("TEST_RESULT: tb_top_poscmd PASS");
        else             $display("TEST_RESULT: tb_top_poscmd FAIL  failures=%0d", errors);
        $display("================================================================");
        $finish;
    end

    initial begin
        #2_000_000_000;
        $display("TEST_RESULT: tb_top_poscmd FAIL  failures=999 (TIMEOUT)");
        $finish;
    end

endmodule
