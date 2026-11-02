//=============================================================================
// 文件名  : tb_pid_tuner.v
// 模块名  : tb_pid_tuner
// 项目    : J280 旋转倒立摆 FPGA 控制系统
// 功能    : 串口单字符在线调参器 自检 testbench（★ 有符号 Q10 运算是重点）
//-----------------------------------------------------------------------------
// 验证点
//   T1  复位值：sel_loop=0, run_en=0, telemetry_en=1（复位即上报），系数=INIT
//   T2  '1'..'5' 正确设置 sel_loop = 0..4
//   T3  正系数 '+'12.5%：1024 → 1152
//   T4  正系数 '-12.5%'：1152 → 1008
//   T5  '*' 翻倍 / '/' 减半
//   T6  ★★ 负系数 '+12.5%'：-1000 → -1125（**绝不能是 -875**）
//   T7  ★★ 负系数 '-12.5%'：-1000 → -875
//   T8  ★★ 负系数 '/' 减半用算术右移：-1125 → -563（floor，不是 -562）
//   T9  饱和：★ v5.0 系数上限改为 2147483647（32bit 有符号）
//   T10 'R' 切换 run_en（两个边沿都可切）
//   T11 'S' 切换 telemetry_en
//   T12 'U' / 'D' 输出恰好 1 拍脉冲
//   T13 无关字符不改变任何状态
//   T14 last_cmd 正确记录
//=============================================================================

`timescale 1ns/1ns

module tb_pid_tuner;

    reg         clk;
    reg         rst_n;
    reg  [7:0]  cmd_data;
    reg         cmd_valid;

    // ★ v5.0：系数位宽已加宽到 32bit，本行必须同步，
//   否则只取到低 16 位，饱和值看起来像 -2147483648/0（本轮踩过）
wire [31:0] kp_th, ki_th, kd_th, kp_x, kd_x;
    wire [2:0]  sel_loop;
    wire        run_en, telemetry_en;
    wire        up_ref_capture, down_ref_capture;
    wire        pump_pol_toggle;
    wire [31:0] swing_duty;        // ★ v5.0 同步为 32bit
    wire [15:0] last_cmd;

    integer fail_cnt, check_cnt;
    integer i;

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

    // 发一个命令字符（保证 cmd_valid 只有**一个**时钟沿被采到）
    //   ★ 末尾的 #1 是必需的：命令是在 posedge 以非阻塞方式写进寄存器的，
    //     若在同一个时间步的 active 区域立刻读输出，NBA 更新尚未提交，
    //     会读到**上一个命令**的结果，表现成"整整慢一条命令"。
    //     （首次运行 23 条断言全挂就是这个原因，不是 RTL 的问题。）
    task send_cmd;
        input [7:0] c;
        begin
            cmd_data  = c;
            cmd_valid = 1'b0;
            @(posedge clk);
            cmd_valid = 1'b1;
            @(posedge clk);      // 这个沿 RTL 采到 cmd_valid=1
            cmd_valid = 1'b0;
            #1;                  // 等 NBA 更新提交后再让调用者读输出
        end
    endtask

    // 打一次复位
    task do_reset;
        begin
            rst_n = 1'b0;
            repeat (4) @(posedge clk);
            rst_n = 1'b1;
            repeat (2) @(posedge clk);
        end
    endtask

    initial clk = 1'b0;
    always #10 clk = ~clk;

    // 初值：KP=1.0(1024)、KI=-1000(负系数测试用)、KD=0、位置P=0、位置D=0
    pid_tuner #(
        .Q_FMT      (10),
        .KP_TH_INIT (32'sd1024),
        .KI_TH_INIT (-32'sd1000), // = -1000（32bit 有符号位模式）
        .KD_TH_INIT (32'sd0),
        .KP_X_INIT (32'sd0),
        .KD_X_INIT (32'sd0)
    ) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .cmd_data         (cmd_data),
        .cmd_valid        (cmd_valid),
        .kp_th            (kp_th),
        .ki_th            (ki_th),
        .kd_th            (kd_th),
        .kp_x             (kp_x),
        .kd_x             (kd_x),
        .sel_loop         (sel_loop),
        .run_en           (run_en),
        .telemetry_en     (telemetry_en),
        .up_ref_capture   (up_ref_capture),
        .down_ref_capture (down_ref_capture),
        .pump_pol_toggle  (pump_pol_toggle),
        .swing_duty       (swing_duty),
        .last_cmd         (last_cmd)
    );

    initial begin
        $display("================================================================");
        $display(" tb_pid_tuner 开始（Q10 有符号定点）");
        $display("================================================================");

        fail_cnt = 0; check_cnt = 0;
        rst_n = 1'b0; cmd_valid = 1'b0; cmd_data = 8'h00;
        #100; rst_n = 1'b1; #100;

        //---------------------------------------------------------------------
        // T1 复位值
        //---------------------------------------------------------------------
        check_i("T1a 复位 sel_loop=0",       0,    sel_loop);
        check_i("T1b 复位 run_en=0",         0,    run_en);
        check_i("T1c 复位 telemetry_en=1（本版复位即上报）", 1, telemetry_en);
        check_i("T1d 复位 kp_th=1024(1.0)",  1024, $signed(kp_th));
        check_i("T1e 复位 ki_th=-1000",      -1000, $signed(ki_th));

        //---------------------------------------------------------------------
        // T2 '1'..'5' 选环
        //---------------------------------------------------------------------
        send_cmd(8'h32);   // '2'
        check_i("T2a '2' → sel_loop=1", 1, sel_loop);
        send_cmd(8'h35);   // '5'
        check_i("T2b '5' → sel_loop=4", 4, sel_loop);
        send_cmd(8'h31);   // '1' → KP
        check_i("T2c '1' → sel_loop=0", 0, sel_loop);

        //---------------------------------------------------------------------
        // T3/T4/T5 正系数运算
        //   kp_th = 1024
        //   '>' : 1024 + (1024>>>3=128) = 1152
        //   '<' : 1152 - (1152>>>3=144) = 1008
        //   '*' : 2016
        //   '/' : 1008
        //---------------------------------------------------------------------
        send_cmd(8'h3E);   // '>'
        check_i("T3  KP 1024 的 +12.5% = 1152", 1152, $signed(kp_th));

        send_cmd(8'h3C);   // '<'
        check_i("T4  KP 1152 的 -12.5% = 1008", 1008, $signed(kp_th));

        send_cmd(8'h2A);   // '*'
        check_i("T5a KP 1008 翻倍 = 2016", 2016, $signed(kp_th));

        send_cmd(8'h2F);   // '/'
        check_i("T5b KP 2016 减半 = 1008", 1008, $signed(kp_th));

        //---------------------------------------------------------------------
        // T6 ★★ 负系数 +12.5%：-1000 → -1000 + (-125) = -1125
        //   若误把系数当无符号处理，会得到 -875 之类完全错误的值
        //---------------------------------------------------------------------
        send_cmd(8'h32);   // '2' → sel_loop=1 (KI)
        check_i("T6a 选中 KI", 1, sel_loop);
        check_i("T6b KI 初始 = -1000", -1000, $signed(ki_th));

        send_cmd(8'h3E);   // '>'
        check_i("T6c ★负数 -1000 的 +12.5% = -1125（不是 -875）", -1125, $signed(ki_th));

        //---------------------------------------------------------------------
        // T7 ★★ 负系数 -12.5%：从 -1000 出发 → -1000 - (-125) = -875
        //---------------------------------------------------------------------
        do_reset();
        send_cmd(8'h32);   // '2' → KI
        send_cmd(8'h3C);   // '<'
        check_i("T7  ★负数 -1000 的 -12.5% = -875（不是 -1125）", -875, $signed(ki_th));

        //---------------------------------------------------------------------
        // T8 ★★ 负系数减半用算术右移：-1125 >>> 1 = -563（floor，不是 -562）
        //   -1125/2 = -562.5；'/' 向 0 取整得 -562，'>>>' 向负无穷得 -563
        //   本模块要求用 >>>，所以期望 -563
        //---------------------------------------------------------------------
        do_reset();
        send_cmd(8'h32);   // '2' → KI
        send_cmd(8'h3E);   // '>' → -1125
        check_i("T8a 准备值 = -1125", -1125, $signed(ki_th));
        send_cmd(8'h2F);   // '/'
        check_i("T8b ★-1125 减半 = -563（算术右移向负无穷）", -563, $signed(ki_th));

        //---------------------------------------------------------------------
        // T9 饱和：KP 从 1024 连续翻倍到 2^30，再翻倍必须钳在 2147483647
        //---------------------------------------------------------------------
        do_reset();
        send_cmd(8'h31);   // '1' → KP（复位值 1024 = 2^10）
        for (i = 0; i < 20; i = i + 1) send_cmd(8'h2A);   // → 2^30 = 1073741824
        check_i("T9a KP 翻倍到 2^30", 1073741824, $signed(kp_th));
        send_cmd(8'h2A);   // 2^31 超出 32bit 有符号上限 → 饱和
        check_i("T9b ★2^30 再翻倍饱和为 2147483647（不回绕）", 2147483647, $signed(kp_th));
        check_i("T9c 饱和后仍为正数（符号未翻转）", 1, ($signed(kp_th) > 0) ? 1 : 0);
        send_cmd(8'h2A);   // 再翻倍仍 2147483647
        check_i("T9d 持续翻倍保持 2147483647", 2147483647, $signed(kp_th));

        //---------------------------------------------------------------------
        // T10 'R' 切换 run_en
        //---------------------------------------------------------------------
        do_reset();
        check_i("T10a 复位 run_en=0", 0, run_en);
        send_cmd(8'h52);   // 'R'
        check_i("T10b 'R' → run_en=1", 1, run_en);
        send_cmd(8'h52);   // 'R'
        check_i("T10c 'R' 再按 → run_en=0", 0, run_en);

        //---------------------------------------------------------------------
        // T11 'S' 切换 telemetry_en
        //---------------------------------------------------------------------
        send_cmd(8'h53);   // 'S'
        check_i("T11a 'S' → telemetry_en=0", 0, telemetry_en);
        send_cmd(8'h53);   // 'S'
        check_i("T11b 'S' 再按 → telemetry_en=1", 1, telemetry_en);

        //---------------------------------------------------------------------
        // T12 'U' / 'D' 恰好 1 拍脉冲
        //---------------------------------------------------------------------
        send_cmd(8'h55);   // 'U'
        check_i("T12a 'U' 当拍 up_ref_capture=1", 1, up_ref_capture);
        @(posedge clk);
        #1;                // ★ 等 NBA 提交后再读，否则读到的是上一拍的值
        check_i("T12b 'U' 仅 1 拍（下一拍归零）", 0, up_ref_capture);

        send_cmd(8'h44);   // 'D'
        check_i("T12c 'D' 当拍 down_ref_capture=1", 1, down_ref_capture);
        @(posedge clk);
        #1;                // ★ 同上
        check_i("T12d 'D' 仅 1 拍（下一拍归零）", 0, down_ref_capture);

        //---------------------------------------------------------------------
        // T13 无关字符不改变任何状态
        //---------------------------------------------------------------------
        do_reset();
        send_cmd(8'h31);   // '1' → KP=1024
        begin : t13
            integer kp_before, sel_before, run_before, tel_before;
            kp_before  = $signed(kp_th);
            sel_before = sel_loop;
            run_before = run_en;
            tel_before = telemetry_en;
            send_cmd(8'h38);   // '8'  非法
            send_cmd(8'h78);   // 'x'  非法
            send_cmd(8'h00);   // 0x00 非法
            send_cmd(8'h20);   // 空格 非法
            check_i("T13a 非法字符不改系数", kp_before,  $signed(kp_th));
            check_i("T13b 非法字符不改 sel_loop", sel_before, sel_loop);
            check_i("T13c 非法字符不改 run_en", run_before, run_en);
            check_i("T13d 非法字符不改 telemetry_en", tel_before, telemetry_en);
        end

        //---------------------------------------------------------------------
        // T15 ★ 新增命令 '+'：加 64（修复"0 是乘法不动点"的死锁）
        //---------------------------------------------------------------------
        do_reset();
        send_cmd(8'h31);   // '1' → KP (=1024)
        send_cmd(8'h2F); send_cmd(8'h2F); send_cmd(8'h2F); send_cmd(8'h2F);
        send_cmd(8'h2F); send_cmd(8'h2F); send_cmd(8'h2F); send_cmd(8'h2F);
        send_cmd(8'h2F); send_cmd(8'h2F); send_cmd(8'h2F); send_cmd(8'h2F);
        check_i("T15a 连按 / 后 KP 归零（制造旧版的死锁状态）", 0, $signed(kp_th));
        send_cmd(8'h2B);   // '+' 加 64
        check_i("T15b ★ 从 0 按 '+' 得 64（旧版此处永久卡死）", 64, $signed(kp_th));
        send_cmd(8'h2B);
        check_i("T15c 再按 '+' 得 128", 128, $signed(kp_th));

        //---------------------------------------------------------------------
        // T16 ★ 新增命令 '-'：取反（修复"无法翻转极性"的缺口）
        //---------------------------------------------------------------------
        do_reset();
        send_cmd(8'h31);   // '1' → KP (=1024)
        send_cmd(8'h2D);   // '-' 取反
        check_i("T16a ★ KP 1024 取反 → -1024", -1024, $signed(kp_th));
        send_cmd(8'h2D);   // '-' 再取反
        check_i("T16b 再取反回到 +1024", 1024, $signed(kp_th));
        send_cmd(8'h32);   // '2' → KI (=-1000)
        send_cmd(8'h2D);   // '-'
        check_i("T16c KI -1000 取反 → +1000", 1000, $signed(ki_th));

        //---------------------------------------------------------------------
        // T17 ★ 新增命令 'P'：翻转起摆能量泵极性（1 拍脉冲）
        //---------------------------------------------------------------------
        send_cmd(8'h50);   // 'P'
        check_i("T17a 'P' 当拍 pump_pol_toggle=1", 1, pump_pol_toggle);
        @(posedge clk);
        #1;
        check_i("T17b 'P' 仅 1 拍", 0, pump_pol_toggle);

        //---------------------------------------------------------------------
        // T14 last_cmd 记录
        //---------------------------------------------------------------------
        send_cmd(8'h3E);   // '>'
        check_i("T14 last_cmd 记录为 '>'(0x3E)", 8'h3E, last_cmd[7:0]);

        //---------------------------------------------------------------------
        $display("================================================================");
        $display(" 检查点总数 = %0d，失败 = %0d", check_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("TEST_RESULT: tb_pid_tuner PASS");
        else
            $display("TEST_RESULT: tb_pid_tuner FAIL  失败项数=%0d", fail_cnt);
        $display("================================================================");
        $finish;
    end

    initial begin
        #500000;
        $display("TEST_RESULT: tb_pid_tuner FAIL  失败项数=999 (TIMEOUT)");
        $finish;
    end

endmodule
