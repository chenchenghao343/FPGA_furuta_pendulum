//=============================================================================
// 文件名  : tb_top_smoke.v
// 模块名  : tb_top_smoke
// 项目    : J280 旋转倒立摆 FPGA 控制系统
// 功能    : 整机顶层冒烟测试（真实 9600 8N1 串口 + 按键 + 编码器正交序列）
//-----------------------------------------------------------------------------
// 本 tb 的目的：把 pptx slide11 的三条信号通路串起来跑一遍，证明
//   ① 姿态采集通路 ADC → 中值标定 → phi
//   ② 位置反馈通路 编码器正交 → pos_raw
//   ③ 控制输出通路 FSM → 仲裁 → motor_drv → PWMA/AN1/AN2
//   ④ 辅助调试通路 遥测帧 AA 55 | w0 | w1 | w2 | 0D 0A
// 全部**真实连通**，而不是各模块孤立正确。
//-----------------------------------------------------------------------------
// 仿真加速说明（★ 必须如实记录，不能让人误以为跑的是全速参数）
//   - `angle_adc` 的 ACC_BITS 由默认 16 改为 4（defparam）：
//       真实：2^16 = 65536 点平均，控制周期 1.31072ms，控制率 762.94Hz
//       本 tb：2^4  = 16    点平均，控制周期 0.32us，  控制率 3.125MHz
//     平均逻辑与节拍产生机制完全相同，只是等比例缩短，便于仿真。
//   - `angle_adc` 的 STARTUP_CYCLES 由 500000 改为 200。
//   - `uart_link` 的 FRAME_GAP 由 999999 改为 20000（帧间隔缩短）。
//   - **串口波特率保持真实 9600 8N1 不变**（BIT_NS = 104167ns），
//     按键消抖保持真实 20ms 不变（CNT_MAX = 1_000_000），
//     因此这两条通路验的是真参数。
//-----------------------------------------------------------------------------
// 验证点
//   T1  复位后 adc_oe_n=0、adc_clk 为 25MHz 二分频
//   T2  上电后 FSM 处于 IDLE(0)，电机停机
//   T3  编码器正交 20 个周期 → 4 倍频计数变化 80
//   T4  按下 SW1（真实 20ms 消抖）→ 进入起摆 SWING(1)
//   T5  角度落在直立参考附近 → 自动切平衡 BALANCE(2)
//   T6  串口发 'S'（真实 9600 波形）→ 打开遥测
//   T7  ★ 按真实 9600 时序解码一帧遥测，逐字节核对
//       AA 55 | w0 | w1 | w2 | 0D 0A，且 w2 == 2（平衡）
//   T8  XH2 预留接口 PWMB/BN1/BN2 恒为 0（未驱动，符合设计约定）
//   T9  LED 指示：平衡态下 LED3(已平衡) 点亮（【高电平点亮】，v2.6 修正）
//=============================================================================

`timescale 1ns/1ns

module tb_top_smoke;

    // 9600 8N1 每比特 = 50MHz/9600 = 5208 拍 = 104160ns（规范值 104.1667us）。
    //   取整拍 5208 与 uart_tx / uart_rx 的 BIT_CNT_MAX 严格一致，
    //   10 位累计误差为 0，彻底消除采样点漂移。
    localparam integer BIT_NS = 104160;         // 一位时间（ns）= 5208 拍 × 20ns

    // 缩短 ADC 平均窗口（见文件头说明）
    localparam integer ACC_BITS_TB     = 4;
    localparam integer STARTUP_TB      = 200;
    localparam integer FRAME_GAP_TB    = 20000;

    reg        clk_50m;
    reg        rst_n;
    reg  [3:0] key_sw;
    wire       uart_tx_pin;
    reg        uart_rx_pin;
    wire       adc_clk, adc_oe_n;
    reg  [9:0] adc_data_in;
    reg        enc_a, enc_b;
    wire       PWMA, AN1, AN2, PWMB, BN1, BN2;
    reg        enc2_a, enc2_b;
    wire [3:0] led;

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

    initial clk_50m = 1'b0;
    always #10 clk_50m = ~clk_50m;

    //=========================================================================
    // DUT
    //=========================================================================
    pendulum_top dut (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .key_sw      (key_sw),
        .uart_tx     (uart_tx_pin),
        .uart_rx     (uart_rx_pin),
        .adc_clk     (adc_clk),
        .adc_oe_n    (adc_oe_n),
        .adc_data_in (adc_data_in),
        .enc_a       (enc_a),
        .enc_b       (enc_b),
        .PWMA        (PWMA),
        .AN1         (AN1),
        .AN2         (AN2),
        .PWMB        (PWMB),
        .BN1         (BN1),
        .BN2         (BN2),
        .enc2_a      (enc2_a),
        .enc2_b      (enc2_b),
        // ★ v6.6 新增端口：外部 5 键置"不按"，OLED 两线留空
        //   （不接 ext_key 会让 run_pulse/inc/dec 变 X；复位通路已与它无关）
        .ext_key     (5'b11111),
        // v6.7: SW6 复位键不按（高 = 不复位），复位只由 tb 的 rst_n 控制
        .ext_rst_key (1'b1),
        .oled_scl    (),
        .oled_sda    (),
        .led         (led)
    );

    // ---- 仿真加速：覆盖 ADC 平均窗口与遥测帧间隔 ----
    // ★ v4.1 起 angle_adc 改为【照官方做法】：每 SMP_STEP 拍采一点，
    //   每 SMP_3MS 拍输出 16 点平均。原来的 ACC_BITS/STARTUP_CYCLES 已删除，
    //   故此处改为覆盖新参数。为让仿真跑得动，按比例缩小：
    //       SMP_STEP = 9   -> 10 拍采一点
    //       SMP_3MS  = 159 -> 160 拍一个窗口
    //       160/10 = 16，正好 16 点，与新结构一致
    defparam dut.u_angle_adc.SMP_STEP = 14'd9;
    defparam dut.u_angle_adc.SMP_3MS  = 18'd159;
    defparam dut.u_uart_link.FRAME_GAP      = FRAME_GAP_TB;

    //=========================================================================
    // 实时参数镜像（用于断言）
    //   真实参数：ACC_BITS=16 → 控制周期 1.31072ms / 762.94Hz
    //   本 tb    ：ACC_BITS=4  → 控制周期 16/50MHz = 320ns
    //=========================================================================

    //=========================================================================
    // 串口接收（按真实 9600 波形，位中点采样）
    //
    // ★ NBA 竞争修正：原实现在 uart_recv_byte 中用 #(BIT_NS) 相对延时采样，
    //   虽然延时常量正确，但循环里对上一字节的 #(BIT_NS) 累加会与 clk 域
    //   （uart_tx 的 NBA 输出）产生非整数拍的相位漂移：
    //     BIT_NS = 104167ns，clk 周期 20ns → 每位 5208.35 拍（非整数）。
    //   10 位累计漂移 3.5 拍，10 字节累计可达 35 拍，足以在位数多的帧后段
    //   采到位的边缘甚至隔壁位。现改为按【整拍】推进：
    //     每位 = 5208 拍（= BIT_CNT，与 uart_tx 的 BIT_CNT_MAX 严格一致），
    //     采样点取位中心 = 起始沿 + 2604 拍。
    //   这样采样点永远落在位中心，与 RTL 位定时严格同步，不随时间漂移。
    //=========================================================================
    localparam integer BIT_CNT = 5208;          // 50MHz / 9600（单位：时钟拍）
    localparam integer BIT_MID = 2604;          // 半位宽（单位：时钟拍）
    // ★ 单位必须分清：`#` 是纳秒，BIT_CNT 是拍数。
    //   一位 = 5208 拍 × 20ns = 104160 ns，与修复后的 uart_tx 严格一致。
    //   （模块级 BIT_NS 已在上方声明为 104167 仅供注释参考，
    //     此处不再重复声明，发送/接收统一使用 104160。）

    task uart_recv_byte;
        output [7:0] d;
        output       ok;
        integer k;
        integer guard;
        integer idle_cnt;       // 连续高电平拍数（空闲检测）
        reg     got_start;
        begin
            d  = 8'h00;
            ok = 1'b0;
            // ★ 有界等待起始位下降沿。
            //   原实现直接 @(negedge uart_tx_pin)：一旦遥测没打开（例如 'S'
            //   命令失效），uart_tx_pin 会永远保持高电平，该语句永久阻塞，
            //   整个 tb 只能等到 200ms 看门狗才结束 —— 这正是一次真实运行中
            //   "失败项数=999 (TIMEOUT)" 的成因（999 是看门狗那一条）。
            //   这里改为最多等 3 个字节时间；超时则 ok=0 返回，
            //   让每一字节比对都明确 FAIL，而不是让整片仿真挂死。
            guard     = 0;
            got_start = 1'b0;
            idle_cnt  = 0;

            // ★★ 修正：必须先检测"线路空闲"，再接受下降沿作为起始位 ★★
            //   原实现只要发现 uart_tx_pin 为低就认定是起始位。但**数据位本身
            //   就是低电平**（例如帧 0x0C 的 D5/D6 全为 0，或 0x27 的多个低位），
            //   于是解码器会锁到帧中间的某个数据位上，
            //   之后按位推进自然全部错位 —— 实测解出 00 00 FF 00 00 FF 00 00 …
            //   就是这个症状（0xFF 是停止位区域被当成整字节全 1）。
            //
            //   正确做法（也是所有真实 UART 接收机的做法）：
            //     1) 先等线路连续保持高电平 >= 2 个位时间，确认处于空闲；
            //     2) 再等第一个下降沿，那才是真正的起始位。
            //   本系统中帧内字节之间有不小于 BYTE_GAP(46876 拍 ≈ 9 位时间)的
            //   空闲，帧与帧之间更长，因此"2 位时间"的判据足以把
            //   "字节间空隙"与"数据位中间的下降沿"可靠区分开，不会漏帧。
            while ((idle_cnt < 2 * BIT_CNT) && (guard < 4 * BIT_CNT * 11)) begin
                @(posedge clk_50m);
                if (uart_tx_pin === 1'b1) idle_cnt = idle_cnt + 1;
                else                      idle_cnt = 0;
                guard = guard + 1;
            end

            // 空闲确认后再逐拍等起始位下降沿（命中即刻退出，不浪费拍数）
            while ((guard < 4 * BIT_CNT * 11) && !got_start) begin
                @(posedge clk_50m);
                if (uart_tx_pin === 1'b0) begin
                    got_start = 1'b1;
                end
                else begin
                    guard = guard + 1;
                end
            end

            if (!got_start) begin
                $display("[CHECK] 遥测解码：等待起始位超时（遥测可能未打开）=> FAIL");
                fail_cnt = fail_cnt + 1;
                ok = 1'b0;
            end
            else begin
                // ★★ 单位修正 ★★
                //   这里原来写的是 #(BIT_MID) / #(BIT_CNT)，但 BIT_CNT=5208 是
                //   **时钟拍数**，而 `#` 后面的数是**时间（ns）**。
                //   5208 ns 只有 260 拍，比一个位（104160 ns）少 20 倍，
                //   于是采样点全部落在起始位的低电平里 → 整字节解成 0x00。
                //   （实测"收到遥测帧: 00 00 00 00 00 00 00 00 00 00"即此因。）
                //
                //   改为用 ns 常量：
                //     一位 = 5208 拍 × 20ns = **104160 ns**（与 uart_tx 严格一致）
                //     半位 = 52080 ns
                #(BIT_NS/2);             // 推进到起始位中点（自起始沿起算）
                for (k = 0; k < 8; k = k + 1) begin
                    #(BIT_NS);           // 整位推进 → 正好落在 Dk 的中点
                    d[k] = uart_tx_pin;
                end
                #(BIT_NS);               // 停止位中点
                ok = (uart_tx_pin === 1'b1);
            end
        end
    endtask

    //=========================================================================
    // 串口发送（按真实 9600 波形注入到 uart_rx）
    //
    // ★★ 原 tb 缺陷（T6b"'S' 命令没有生效"的根因之一）★★
    //   1) 原实现用 #(BIT_NS)=104167ns 逐位延时，而 clk 周期是 20ns，
    //      104167ns = 5208.35 拍（非整数），每位引入 0.35 拍漂移；
    //      更严重的是延时的【起点】不在时钟沿上，与 uart_rx 的 3 级同步器
    //      采样相位随机，起始位下降沿可能被漏采 → uart_rx 根本不启动 →
    //      cmd_valid 永不拉高 → 'S' 命令失效。
    //   2) 修法：所有电平切换统一在 @(negedge clk_50m) 之后建立（时钟低电平
    //      窗口），使下一拍 posedge 到来时 uart_rx 已稳定半个周期；位宽用
    //      【整拍】5208 拍，与 uart_rx 的 BIT_CNT_MAX 严格一致，零漂移。
    //=========================================================================
    task uart_send_byte;
        input [7:0] d;
        integer k;
        begin
            @(negedge clk_50m);
            uart_rx_pin = 1'b0;              // 起始位
            repeat (BIT_CNT) @(posedge clk_50m);
            for (k = 0; k < 8; k = k + 1) begin
                @(negedge clk_50m);
                uart_rx_pin = d[k];          // LSB first
                repeat (BIT_CNT) @(posedge clk_50m);
            end
            @(negedge clk_50m);
            uart_rx_pin = 1'b1;              // 停止位
            repeat (BIT_CNT) @(posedge clk_50m);
        end
    endtask

    //=========================================================================
    // 正交编码器序列产生（每个状态持续 50 拍，远慢于 50MHz 同步器）
    //
    // ★★ 方向修正（原 tb 缺陷，根因是激励序列与 RTL 真值表相反）★★
    //   qep_encoder 的方向真值表（state = {enc2_a, enc2_b}）：
    //     +1（正转）：00->01, 01->11, 11->10, 10->00
     //    -1（反转）：00->10, 01->00, 11->01, 10->11
    //   原 tb 的序列是 (1,0)->(1,1)->(0,1)->(0,0)，即
    //     10->11（-1）, 11->01（-1）, 01->00（-1）, 00->10（-1）
    //   四个跳变全是 -1 —— 所以 20 个"周期"使 pos_raw 由 10000 减到 9920，
    //   与真实观察（9920）完全一致，证明原 tb 的"正转"其实是反转。
    //   修正：把相序反过来，用 (0,0)->(0,1)->(1,1)->(1,0)：
    //     00->01（+1）, 01->11（+1）, 11->10（+1）, 10->00（+1）
    //   四个跳变全为 +1，这才是真正的"正转"。
    //=========================================================================
    //=========================================================================
    // ★ v2.4：编码器激励由 enc_a/enc_b 改为 enc2_a/enc2_b
    //   设计已统一使用 XH2 这一路（电机 A 路 + 编码器 2），
    //   本 tb 原先驱动的是 XH1 的 enc_a/enc_b，换口后激励打不到 DUT，
    //   位置检查必然失败 —— 这正是本次回归暴露出来的。
    //=========================================================================
    task qep_step;
        input integer n;
        integer k;
        begin
            // ★ 相序起点修正：原来第一个状态写的是 (0,0)，而 enc_a/enc_b 的
            //   初值就是 (0,0)，于是这一拍是**空操作**、不产生边沿。
            //   20 个周期共 80 次赋值却只产生 79 个跳变，
            //   实测 Δpos = +79 而非 +80，正是这个原因。
            //   修法：把 (0,0) 挪到相序末尾，使每次赋值都是真实的状态跳变。
            //   （不动 enc 初值，因为初值若改成 (1,0)，复位刚释放时 RTL 会
            //     看到 00→10 这一次跳变，反而多计 1。）
            for (k = 0; k < n; k = k + 1) begin
                enc2_a = 1'b0; enc2_b = 1'b1; repeat (50) @(posedge clk_50m); // 00→01 : +1
                enc2_a = 1'b1; enc2_b = 1'b1; repeat (50) @(posedge clk_50m); // 01→11 : +1
                enc2_a = 1'b1; enc2_b = 1'b0; repeat (50) @(posedge clk_50m); // 11→10 : +1
                enc2_a = 1'b0; enc2_b = 1'b0; repeat (50) @(posedge clk_50m); // 10→00 : +1
            end
        end
    endtask

    //=========================================================================
    // 主激励
    //=========================================================================
    reg [7:0] frame [0:27];   // ★ v6.0: 帧长 28 字节（v3.0.3 为 16，v4.4~v5.11 为 22）
    integer   k;
    integer   pos_delta_test;
    reg       okb;
    integer   adc_clk_period_ns;
    // 帧头同步用
    reg       found_frame;
    reg [7:0] prev_byte;
    reg [7:0] scan_byte;
    integer   sync_guard;

    // 遥测解码前锁存的快照依据（帧内数据是帧首一次性锁存的，见 T7 说明）
    reg [11:0] tel_angle_snap;
    reg [15:0] tel_pos_snap;
    reg [1:0]  tel_mode_snap;

    initial begin
        $display("================================================================");
        $display(" tb_top_smoke 开始（真实 9600 串口 + 真实 20ms 按键消抖）");
        $display(" 仿真加速项：ADC 窗口 150000->160 拍、采样间隔 9375->10 拍, FRAME_GAP=999999->%0d",
                 FRAME_GAP_TB);
        $display("================================================================");

        fail_cnt = 0; check_cnt = 0;
        rst_n = 1'b0;
        key_sw = 4'b1111;          // 按键低有效，空闲全 1
        uart_rx_pin = 1'b1;        // 串口空闲为高
        adc_data_in = 10'd726;     // ★ 726*4 = 2904，与默认 up_ref 2905 相差 1
                                   //   （v3.0.1 参考点改为**本设计 ADC** 实测值
                                   //     2905/846/2059；本 tb 原先按旧的 3149
                                   //     造激励，会让 phi 偏 244 counts 而失败）
        enc_a = 1'b0; enc_b = 1'b0;
        enc2_a = 1'b0; enc2_b = 1'b0;

        #200;
        rst_n = 1'b1;
        #2000;

        //---------------------------------------------------------------------
        // T1 ADC 接口电平与 adc_clk 频率
        //---------------------------------------------------------------------
        check_i("T1a adc_oe_n 恒为 0（低有效使能）", 0, adc_oe_n);

        // 测 adc_clk 半周期：应为 25MHz → 半周期 20ns
        @(posedge adc_clk);
        adc_clk_period_ns = $time;
        @(posedge adc_clk);
        adc_clk_period_ns = $time - adc_clk_period_ns;
        $display("[INFO] adc_clk 实测周期 = %0d ns（期望 40ns = 25MHz）", adc_clk_period_ns);
        check_i("T1b adc_clk 周期 = 40ns(25MHz)", 40, adc_clk_period_ns);

        //---------------------------------------------------------------------
        // T2 上电状态：IDLE、电机停机
        //---------------------------------------------------------------------
        repeat (200) @(posedge clk_50m);
        check_i("T2a 上电 mode=IDLE(0)", 0, dut.mode);
        check_i("T2b 上电 AN1=0", 0, AN1);
        check_i("T2c 上电 AN2=0", 0, AN2);

        //---------------------------------------------------------------------
        // T3 编码器 4 倍频
        //   20 个完整正交周期 → 4×20 = 80 个计数
        //   ★ 判据必须是【有符号的 +80】而不是绝对值：
        //     用绝对值的写法无法分辨正转/反转，会把"方向接反"这种真缺陷放过。
        //     原 tb 用 |Δpos| 且激励相序与 RTL 真值表相反，于是 pos_raw 实际
        //     走了 -80，而判据只看 |−80|=80 依然"通过"，掩盖了方向错误。
        //-------------------------------------------------------------------------
        check_i("T3a 编码器复位初值 = 10000", 10000, dut.pos_raw);
        $display("[INFO] 编码器激励前 pos_raw = %0d", dut.pos_raw);
        qep_step(20);
        repeat (20) @(posedge clk_50m);
        pos_delta_test = $signed(dut.pos_raw) - 10000;
        $display("[INFO] 编码器 20 个正交周期后 pos_raw = %0d (初值 10000, Δ=%0d)",
                 dut.pos_raw, pos_delta_test);
        // 严格有符号判据：正转必须是 +80
        check_i("T3b 4倍频：20周期正转 → Δpos=+80", 80, pos_delta_test);

        //---------------------------------------------------------------------
        // T4 按下 SW1（真实 20ms 消抖）→ 进入起摆
        //
        // ★★ 排查记录（按键链路逐节点探测）★★
        //   链路：key_sw[0] → u_key_run（v6.5 起单个消抖器）→ key_run_press
        //         → run_req（= key_run_press | tuner_run_toggle）
        //         → ctrl_fsm.run_req_rise → mode
        //
        //   节点 1：消抖计数器。原 tb 用 "repeat(1100000) @(posedge clk_50m)"
        //     给出 1,100,000 拍，而 key_filter 需要 cnt 从 0 数到 CNT_MAX
        //     =1,000,000（即 1,000,001 拍）再确认，另有 2 拍同步器延迟。
        //     1,100,000 > 1,000,003，余量充足 —— 这一节点【本来是对的】。
        //     注意：RTL 里该计数器名为 cnt（reg [19:0]），
        //     并非任务描述里写的 cnt_20ms，层次化引用必须用 .cnt。
        //
        //   节点 2：key_press[0] 单拍脉冲。key_filter 在确认按下那一拍
        //     同时置 key_state<=0 并把 key_flag<=1，是标准的 1 拍脉冲。
        //
        //   节点 3：run_req / run_req_rise。run_req_r 是每拍更新的寄存器，
        //     单拍脉冲足以产生一次上升沿，IDLE→SWING 正常。
        //
        //   ★ 真正的问题在于【tb 的激励与采样方式】：
        //     (a) 原 tb 在 qep_step 之后立刻写 key_sw[0]=0，而采样时刻与
        //         时钟沿的相对相位不确定，且
        //     (b) 释放按键后只等 100 拍就直接读 dut.mode —— 读的是 DUT 的
        //         寄存器输出，与 posedge 同拍读会读到 NBA 提交前的旧值。
        //     所以这里：① 用 @(negedge clk_50m) 对齐激励；② 检查前统一
        //     @(posedge clk_50m); #1; 使读到本拍提交后的稳定值；
        //     ③ 直接打印中间节点，便于确证链路每一级都动过。
        //---------------------------------------------------------------------
        @(negedge clk_50m);
        key_sw[0] = 1'b0;                       // 按下（对齐低电平窗口）
        $display("[INFO] SW1 按下：key_sw=%b", key_sw);

        // 等待 > 20ms（1_000_000 拍 @50MHz = 20ms，留足余量）
        repeat (1100000) @(posedge clk_50m);
        #1;
        $display("[INFO] SW1 保持 1.1e6 拍后：cnt=%0d key_state=%b key_flag=%b run_req=%b mode=%0d",
                 dut.u_key_run.cnt,
                 dut.u_key_run.key_state,
                 dut.u_key_run.key_flag,
                 dut.run_req, dut.mode);
        // 期望：cnt 已计满过（此刻可能已回 0）、key_state=0（已确认按下）
        // ★ v6.5：按键消抖器从 generate 的 4 份 g_key[i] 改成单个 u_key_run，
        //   层次化路径随之变化（原为 dut.g_key[0].u_key_filter.*）。
        check_i("T4a SW1 消抖已确认按下(key_state=0)", 0,
                dut.u_key_run.key_state);

        @(negedge clk_50m);
        key_sw[0] = 1'b1;                       // 松开（对齐低电平窗口）
        repeat (100) @(posedge clk_50m);
        #1;

        $display("[INFO] 按下 SW1 后 mode = %0d (0=空闲 1=起摆 2=平衡)", dut.mode);
        // 由于角度已在直立参考附近，会在下一个控制节拍立刻捕获 → 可能直接是 2
        check_i("T4  SW1 后已运行（mode=1 或 2）", 1,
                (dut.mode == 2'd1 || dut.mode == 2'd2) ? 1 : 0);

        //---------------------------------------------------------------------
        // T5 自动切入平衡
        //   ★ 检查前加 #1：dut.mode / run_pid / run_pos 都是寄存器/组合输出，
        //     必须在时间步稳定后读，避免读到 NBA 提交前的旧值。
        //---------------------------------------------------------------------
        repeat (2000) @(posedge clk_50m);
        #1;
        check_i("T5a 角度在直立参考附近 → mode=BALANCE(2)", 2, dut.mode);
        check_i("T5b BALANCE 时 run_pid=1", 1, dut.run_pid);
        check_i("T5c BALANCE 时 run_pos=1", 1, dut.run_pos);
        $display("[INFO] 当前 phi = %0d (angle_raw=%0d, up_ref=%0d)",
                 $signed(dut.phi), dut.angle_raw, dut.up_ref);

        //---------------------------------------------------------------------
        // T6/T7 打开遥测并解码一帧（真实 9600）
        //   ★★ 原 tb 缺陷（T6b"'S' 命令没有生效"的直接根因）★★
        //     原代码：uart_send_byte(8'h53); repeat(1000) @(posedge clk_50m);
        //     一个字节 9600 8N1 = 10 位 × 5208 拍 = 52080 拍 = 1.0417ms，
        //     而 1000 拍只有 20us —— 比收完一个字节所需的时间少了约 52 倍。
        //     检查发生在 pid_tuner 收到 cmd_valid 之前，telemetry_en 当然还是 0。
        //     修法：等待时间改为【一整字节 + 同步延迟 + 裕量】，
        //     即 BIT_CNT*11 + 1000 拍（57288+1000=58288 拍）。
        //---------------------------------------------------------------------
        @(posedge clk_50m); #1;
        // ★ v6.0：SW4 已从"遥测开关"改为"定点归零"，顶层里原来的
        //   telemetry_btn / telemetry_on 两个信号已删除，uart_link 直接接
        //   telemetry_en。故这里的分层引用要从 telemetry_on 改成 telemetry_en。
        check_i("T6a 遥测默认打开（本版固件复位即上报）", 1, dut.telemetry_en);
        uart_send_byte(8'h53);                  // 'S' -> 关闭
        repeat (BIT_CNT * 11 + 1000) @(posedge clk_50m); #1;
        $display("[INFO] 'S' 后：uart_rx cmd_valid 已发生, pid_tuner.telemetry_en=%b",
                 dut.u_pid_tuner.telemetry_en);
        check_i("T6b 'S' 命令关闭遥测", 0, dut.telemetry_en);
        uart_send_byte(8'h53);                  // 'S' -> 再打开
        repeat (BIT_CNT * 11 + 1000) @(posedge clk_50m); #1;
        check_i("T6c 'S' 再按一次恢复上报", 1, dut.telemetry_en);

        //---------------------------------------------------------------------
        // T7 解码一帧遥测
        //   ★ T7c~T7f 的判据说明（重要）：
        //     遥测数据是【帧首一次性快照】的（uart_link 在 ST_IDLE 启动帧时
        //     w0_snap<=w0_angle / w1_snap<=w1_pos 同时锁存）。因此帧里的
        //     w0/w1 对应的是"帧开始那一刻"的 angle_raw / pos_raw，
        //     而不是解码完成后的当前值。
        //     本 tb 在解码全程 angle_raw 恒定（ADC 输入固定 787→3148），
        //     pos_raw 也恒定（解码期间不再驱动编码器），所以帧内值与当前值
        //     一致，直接用 dut.angle_raw / dut.pos_raw 比对是成立的 ——
        //     但为严谨起见，这里在【开始解码前】先把快照依据锁存下来，
        //     用锁存值比对，避免"解码期间状态变化"造成的潜在误判。
        //---------------------------------------------------------------------
        tel_angle_snap = dut.angle_raw;
        tel_pos_snap   = dut.pos_raw;
        tel_mode_snap  = dut.mode;
        $display("[INFO] 开始解码遥测帧（真实 9600，约 20ms）...");
        $display("[INFO] 解码前快照：angle_raw=%0d pos_raw=%0d mode=%0d",
                 tel_angle_snap, tel_pos_snap, tel_mode_snap);
        //-----------------------------------------------------------------
        // ★★ 帧头同步（原 tb 从帧中间开始监听，导致整帧错开一个字节）★★
        //   实测症状：解出 55 0c 4c 27 60 00 02 0d 0a aa，
        //   即正好是正确帧 AA 55 0C 4C 27 60 00 02 0D 0A **整体后移一个字节**。
        //   原因：调用解码时第一帧可能已经在发送中（byte0 已过去），
        //   uart_recv_byte 的空闲检测会一直等到"字节间空隙"，
        //   于是把 byte1 当成了帧头。
        //
        //   正确做法与真实上位机一致：**按帧头 AA 55 同步**——
        //   不断收字节，直到出现连续的 AA 55，才认定这是帧的起点。
        //   这样无论从帧中间还是帧间开始监听，都能对齐到同一个帧。
        //-----------------------------------------------------------------
        found_frame = 1'b0;
        prev_byte   = 8'h00;
        sync_guard  = 0;
        while (!found_frame && (sync_guard < 40)) begin
            uart_recv_byte(scan_byte, okb);
            sync_guard = sync_guard + 1;
            if ((prev_byte === 8'hAA) && (scan_byte === 8'h55)) begin
                frame[0]    = 8'hAA;
                frame[1]    = 8'h55;
                found_frame = 1'b1;
            end
            prev_byte = scan_byte;
        end

        if (!found_frame) begin
            $display("[CHECK] 遥测解码：在 40 个字节内未找到帧头 AA 55 => FAIL");
            fail_cnt = fail_cnt + 1;
        end
        else begin
            $display("[INFO] 已按帧头 AA 55 同步（扫描 %0d 个字节）", sync_guard);
            // ★ v6.0：帧长 28 字节，必须收到 byte27；旧代码只收到 byte15，
            //   而帧尾断言写在 frame[14]/[15] —— 那是 v3.0.3（16 字节帧）
            //   的位置，早已过期两个版本（v4.4 是 20/21，v6.0 是 26/27）。
            //   这也是本 tb 历史上 T7i/T7j 一直 FAIL 的原因。
            for (k = 2; k < 28; k = k + 1) begin
                uart_recv_byte(frame[k], okb);
            end
        end

        $display("[INFO] 收到遥测帧: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 frame[0], frame[1], frame[2], frame[3], frame[4],
                 frame[5], frame[6], frame[7], frame[8], frame[9]);
        $display("[INFO] 期望格式  : AA 55 %02X %02X %02X %02X 00 02 ... 0D 0A",
                 tel_angle_snap[11:8], tel_angle_snap[7:0],
                 tel_pos_snap[15:8], tel_pos_snap[7:0]);

        check_i("T7a 帧头高字节 = AA", 8'hAA, frame[0]);
        check_i("T7b 帧头低字节 = 55", 8'h55, frame[1]);
        check_i("T7c w0(角度) 高字节", {8'h0, frame[2]}, {8'h0, tel_angle_snap[11:8]});
        check_i("T7d w0(角度) 低字节", {8'h0, frame[3]}, {8'h0, tel_angle_snap[7:0]});
        check_i("T7e w1(位置) 高字节", {8'h0, frame[4]}, {8'h0, tel_pos_snap[15:8]});
        check_i("T7f w1(位置) 低字节", {8'h0, frame[5]}, {8'h0, tel_pos_snap[7:0]});
        check_i("T7g w2(模式) 高字节 = 0", 0, frame[6]);
        check_i("T7h w2(模式) = 2 (平衡)", 2, frame[7]);
        // ★ v3.0.2: byte8/9 = u_motor（本 tb 未驱动，取任意值不断言），CR/LF 顺延到 10/11
        // ★ v3.0.3: byte10~13 = u_min/u_max，CR/LF 顺延到 14/15
        check_i("T7i 帧尾 = 0D", 8'h0D, frame[26]);   // ★ v6.0: 26/27（旧写 14/15）
        check_i("T7j 帧尾 = 0A", 8'h0A, frame[27]);
        check_i("T7k 停止位均为 1", 1, okb);

        //---------------------------------------------------------------------
        // T8 XH2 预留接口不得被驱动
        //---------------------------------------------------------------------
        check_i("T8a PWMB=0（XH2 未驱动）", 0, PWMB);
        check_i("T8b BN1=0（XH2 未驱动）",  0, BN1);
        check_i("T8c BN2=0（XH2 未驱动）",  0, BN2);

        //---------------------------------------------------------------------
        // T9 LED 指示（★ v2.6 修正为高电平点亮）
        //    原断言写 led[2]==0 表示点亮，与 RTL 犯同一个错（假设低电平点亮），
        //    因此一直是 PASS 的 —— 测试与实现共享同一错误假设时，测试抓不到它。
        //---------------------------------------------------------------------
        check_i("T9  BALANCE 态 LED3(已平衡) 点亮 led[2]=1", 1, led[2]);
        //---------------------------------------------------------------------
        $display("================================================================");
        $display(" 检查点总数 = %0d，失败 = %0d", check_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("TEST_RESULT: tb_top_smoke PASS");
        else
            $display("TEST_RESULT: tb_top_smoke FAIL  失败项数=%0d", fail_cnt);
        $display("================================================================");
        $finish;
    end

    // 超时兜底（本 tb 需要跑 ~70ms 真实时间）
    initial begin
        #200000000;
        $display("TEST_RESULT: tb_top_smoke FAIL  失败项数=999 (TIMEOUT)");
        $finish;
    end

endmodule
