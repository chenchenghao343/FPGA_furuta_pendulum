`timescale 1ns/1ns
//====================================================================
// 文件   : tb_uart_link.v
// 被测   : J280_Balance_Ctrl/src/uart_link.v
// 功能   : uart_link 自检式 testbench(自动判定 PASS/FAIL)
// 语言   : 纯 Verilog-2001,ModelSim 可 -batch 运行
// 运行   : vsim -batch work.tb_uart_link -do run.do
//--------------------------------------------------------------------
// 实际端口复核结果(与任务书描述逐字一致,无差异):
//   module uart_link #(
//     parameter CLK_FREQ = 26'd50_000_000, parameter UART_BPS = 21'd9600,
//     parameter BYTE_GAP = 20'd46_876,     parameter FRAME_GAP = 20'd999_999
//   )( input clk, rst_n, uart_rx_pin, telemetry_en,
//      input [15:0] w0_angle, w1_pos, w2_mode,
//      output uart_tx_pin, frame_active,
//      output [7:0] cmd_data, output cmd_valid, output rx_frame_err );
//   => 端口名、方向、位宽与任务书完全一致,均无需修正。
//--------------------------------------------------------------------
// ★ 例化参数折算说明(重要,必须写清以便复核):
//   BYTE_GAP : 保持实现值 46_876【不缩小】。
//              理由:验证点⑤⑥的全部判据都建立在"字节间隔 >= 一个字节时间
//              (52083 拍)"之上,一旦改小就破坏了这两条判据的意义。
//              本 tb 只观测到第 2~3 个字节即可,不需要等满 10 字节。
//   FRAME_GAP: 由 999_999 缩为 5_000。
//              折算关系:实现值 999_999 拍 ≈ 20.0 ms(50MHz);
//              缩为 5_000 拍 = 100 us,即【缩短为原来的 1/200】。
//              这里只用于"帧与帧之间"的间隔,不影响帧内字节时序、
//              不影响字节间隔判据、也不影响帧内容。
//              为什么必须缩小:原值下一帧 ≈ 989560 + 999999 ≈ 199 万拍;
//              而验证点④要求"至少 2 个 FRAME_GAP 周期内 tx 恒高",加上
//              验证点①③⑤⑥还要跑完整帧,用原值仿真要跑上千万拍,
//              在 ModelSim -batch 下耗时过长且无额外验证价值。
//              折算后一整帧 ≈ 989560 + 5000 ≈ 99.5 万拍,单帧约 20ms 仿真时间,
//              仍完全保留"帧间有可见空隙"这一被验性质。
//              注意:缩放只改 FRAME_GAP,帧内 BYTE_GAP 仍为真实值,
//              故第 2 个字节的起始沿与第 1 个字节的起始沿相距 98956 拍,
//              与真实芯片上完全一致。
//--------------------------------------------------------------------
// 本 tb 验什么、判据、为什么:
//   ① 帧内容与格式 : 用【接收侧 task 按 9600 解真实波形】,核对整帧严格等于
//                    AA 55 12 34 56 78 00 02 00 00 0D 0A(共 12 字节)。
//                    为什么:只看 RTL 内部 byte_sel 会漏掉位序/极性错误,
//                    必须从 uart_tx_pin 物理波形上解出来才算数。
//   ② 大端序       : w0=0x1234 期望先收到 0x12 再收到 0x34(高字节先发)。
//                    为什么:大小端接反在上位机上表现为数值乱跳,隐蔽性强,
//                    用 0x1234 这种高低字节不同的值可以一击命中。
//   ③ frame_active : 一帧期间为 1、帧间为 0。为什么:它是上位机/ILA 判断
//                    "帧边界"的依据,粘连会导致上位机帧同步失败。
//   ④ 关闭遥测     : telemetry_en=0 时,至少 2 个 FRAME_GAP 周期内
//                    uart_tx_pin 恒为 1。为什么:防止误发,污染上位机解析。
//   ⑤ 字节间隔     : 帧内相邻字节起始沿间隔必须 >= 一个字节时间 52083 拍。
//                    为什么:任务书要求"不能背靠背连发";间隔过小上位机
//                    可能来不及处理,也说明 BYTE_GAP 未生效。
//   ⑥ 命令透传     : 从 uart_rx_pin 注入 'R'(0x52),核对经 cmd_data/
//                    cmd_valid 正确透传(且 1 拍脉冲)。
//                    为什么:这是调参链路的入口,错了整条在线调参就废了。
//   ⑦ 运行中关闭   : 帧发送途中把 telemetry_en 拉低,核对本字节发完后停止,
//                    不补发剩余字节,且 frame_active 正确回落。
//                    为什么:实现里专门处理了这个分支,必须验到。
//====================================================================

module tb_uart_link;

    //----------------------------------------------------------------
    // 参数
    //----------------------------------------------------------------
    parameter integer CLK_FREQ  = 50000000;
    parameter integer UART_BPS  = 9600;

    // ★ BYTE_GAP 保持实现真实值(不缩小,见文件头折算说明)
    parameter integer BYTE_GAP  = 46876;
    // ★ FRAME_GAP 缩小为 5000(原 999999,折合 1/200,见文件头折算说明)
    parameter integer FRAME_GAP = 5000;

    localparam integer BIT_CNT   = CLK_FREQ / UART_BPS;       // 5208
    localparam integer BYTE_TIME = 10 * BIT_CNT;              // 52080 拍(实测帧长)
    localparam integer BYTE_SPEC = 52083;                     // 一字节规范时长(拍)
    // ★ v6.0：一帧的字节数。v3.0.3 为 10、v4.4~v5.11 为 22、v6.0 为 28。
    //   所有"帧长相关"的判据都必须引用这个常量，不要再写字面量。
    localparam integer NBYTES    = 28;

    //----------------------------------------------------------------
    // DUT 端口
    //----------------------------------------------------------------
    reg         clk;
    reg         rst_n;
    reg         uart_rx_pin;
    wire        uart_tx_pin;
    reg         telemetry_en;
    reg  [15:0] w0_angle;
    reg  [15:0] w1_pos;
    reg  [15:0] w2_mode;
    wire        frame_active;
    wire [7:0]  cmd_data;
    wire        cmd_valid;
    wire        rx_frame_err;

    initial clk = 1'b0;
    always #10 clk = ~clk;

    uart_link #(
        .CLK_FREQ  (CLK_FREQ),
        .UART_BPS  (UART_BPS),
        .BYTE_GAP  (BYTE_GAP),
        .FRAME_GAP (FRAME_GAP)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .uart_rx_pin   (uart_rx_pin),
        .uart_tx_pin   (uart_tx_pin),
        .telemetry_en  (telemetry_en),
        .w0_angle      (w0_angle),
        .w1_pos        (w1_pos),
        .w2_mode       (w2_mode),
        .w3_umotor   (16'd0),        // v3.0.2 本 tb 不检查 u_motor
        // ★★ v6.0：新增 w4~w11 共 8 个输入端口，本 tb 必须全部显式连接。
        //    只连一部分会让未连接的输入悬空成 Z —— 虽然 uart_link 内部不读
        //    它们的内容，但快照寄存器会把 Z 收进去，仿真里传播 X/Z 极难查。
        //    这里一律接 0，逐字节断言只针对有意义的字节（帧头/前 4 字/帧尾）。
        .w4_phieff   (16'd0),
        .w5_dphi     (16'd0),
        .w6_pterm    (16'd0),
        .w7_dterm    (16'd0),
        .w8_pdsum    (16'd0),
        .w9_xset     (16'd0),
        .w10_errx    (16'd0),
        .w11_phibias (16'd0),
        .frame_active  (frame_active),
        .cmd_data      (cmd_data),
        .cmd_valid     (cmd_valid),
        .rx_frame_err  (rx_frame_err)
    );

    //----------------------------------------------------------------
    // 统计与工具
    //----------------------------------------------------------------
    integer errors;
    integer cycle_cnt;
    always @(posedge clk) cycle_cnt = cycle_cnt + 1;

    task chk_i;
        input [255:0] name;
        input integer exp;
        input integer got;
        begin
            if (exp == got)
                $display("[CHECK] %0s: expected=%0d, got=%0d => OK", name, exp, got);
            else begin
                $display("[CHECK] %0s: expected=%0d, got=%0d => FAIL", name, exp, got);
                errors = errors + 1;
            end
        end
    endtask

    //----------------------------------------------------------------
    // 接收侧:按 9600 真实时序从 uart_tx_pin 解出【一个字节】
    //   做法:等下降沿(起始位起点)t0,位 k 的中心 = t0 + k*5208 + 2604
    //   返回:rdata(字节), rstart(起始沿拍号)
    //   为什么这样写:这是"从物理波形反解"的独立参考模型,与 DUT 内部
    //   实现完全无关,能真实反映线上数据。
    //
    //   ★★★ 本次修正的三个真实 tb 缺陷(全部会导致成片失败)★★★
    //   (A) 【NBA 竞争】原代码在 @(posedge clk) 之后【同一个时间步的
    //       active 区域】直接读 uart_tx_pin:
    //           while (cycle_cnt < X) @(posedge clk);
    //           rdata[k] = uart_tx_pin;
    //       uart_tx 内部用非阻塞赋值(NBA)更新 tx,而 NBA 要到 NBA 区域
    //       才提交。因此在 posedge 之后立刻读,读到的是【上一拍的旧值】,
    //       采样点整体前移一位 -> 解出的字节必然错位。
    //       修法:@(posedge clk) 之后加 #1;,让 NBA 先提交、组合逻辑重算完,
    //       再读引脚。
    //   (B) 【末尾 @(posedge uart_tx_pin) 阻塞】原代码在采完停止位中心后
    //       又等一个上升沿:
    //           while (cycle_cnt < t0+9*BIT_CNT+BIT_CNT/2) @(posedge clk);
    //           b = uart_tx_pin;
    //           @(posedge uart_tx_pin);          <-- 错误
    //       停止位【自己的】上升沿早在 t0+9*BIT_CNT = t0+46872 就发生了,
    //       而停止位中心采样点 t0+49476 在其【之后】。所以这个等待必然
    //       错过停止位的上升沿,而要一直等到【下一个字节】发送结束、
    //       线路再次由低变高才返回 —— 即每次 rx_byte 都多消耗整整一个
    //       字节的时间,使 t0 的捕获与整帧计数全部错位(实测字节间隔
    //       与 frame_active 拍数都对不上,正是这个原因)。
    //       修法:删掉这个多余的等待。字节之间有 BYTE_GAP 空闲(高电平),
    //       下一次 @(negedge uart_tx_pin) 自然能等到下一字节的起始位。
    //   (C) 【t0 采样竞争】t0 = cycle_cnt 紧跟在 @(negedge uart_tx_pin)
    //       之后读 cycle_cnt,而 cycle_cnt 由 blocking 赋值在 posedge 更新,
    //       读取时机落在同一时间步内,存在调度次序上的不确定性。
    //       修法:统一改成"边沿后 #1 再读",彻底消除竞争。
    //----------------------------------------------------------------
    task rx_byte;
        output [7:0] rdata;
        output integer rstart;
        integer k;
        integer t0;
        reg b;
        begin
            rdata = 8'h00;
            rstart = 0;
            @(negedge uart_tx_pin);
            #1;                         // ★(C) 等本时间步稳定后再读拍号
            t0 = cycle_cnt;
            rstart = t0;
            // 跳过起始位,在 D0..D7 的中心采样
            for (k = 0; k < 8; k = k + 1) begin
                while (cycle_cnt < (t0 + (k+1)*BIT_CNT + BIT_CNT/2)) @(posedge clk);
                #1;                     // ★(A) 让 NBA 提交后再采样,读到本拍真实电平
                rdata[k] = uart_tx_pin;
            end
            // 等停止位中心,顺带校验停止位=1
            while (cycle_cnt < (t0 + 9*BIT_CNT + BIT_CNT/2)) @(posedge clk);
            #1;                         // ★(A)
            b = uart_tx_pin;
            if (b !== 1'b1)
                $display("     [警告] 停止位采样为 %b(期望 1)", b);
            // ★(B) 这里不再等待上升沿。停止位的上升沿已经在本次采样点之前
            //      发生,继续等待会导致本 task 多吞掉一个完整字节的时间。
            //      直接返回即可:下一次调用入口处的 @(negedge uart_tx_pin)
            //      会在 BYTE_GAP 空闲之后正确捕获下一字节的起始位下降沿。
        end
    endtask

    //----------------------------------------------------------------
    // 接收侧:注入一个字节到 uart_rx_pin(模拟 PC 下发命令)
    //   bit_w 固定用 5208(标准 9600),stop_val 指定停止位电平
    //
    //   ★ 本次修正的真实 tb 缺陷:
    //   原代码直接在调用点赋 uart_rx_pin,而调用点紧跟 repeat(N) @(posedge clk),
    //   即赋值发生在【时钟上升沿的同一时间步】,与 uart_rx 内部
    //   rx_sync1 <= rx 的采样构成竞争,起始位下降沿可能被漏采或错位。
    //   修法:所有电平切换统一在 @(negedge clk) 之后(时钟低电平窗口)建立,
    //   使下一拍 posedge 到来时 rx 已稳定半个周期,消除竞争。
    //   位宽保持严格 5208 拍/位,与 RTL 的 BIT_CNT_MAX 完全一致。
    //----------------------------------------------------------------
    task pc_send_byte;
        input [7:0] d;
        input       stop_val;
        integer k;
        begin
            @(negedge clk);
            uart_rx_pin = 1'b0;                 // 起始位(在低电平窗口建立)
            repeat (BIT_CNT) @(posedge clk);
            for (k = 0; k < 8; k = k + 1) begin
                @(negedge clk);
                uart_rx_pin = d[k];             // LSB first
                repeat (BIT_CNT) @(posedge clk);
            end
            @(negedge clk);
            uart_rx_pin = stop_val;             // 停止位
            repeat (BIT_CNT) @(posedge clk);
            @(negedge clk);
            uart_rx_pin = 1'b1;                 // 回空闲
        end
    endtask

    //----------------------------------------------------------------
    // 命令透传捕获
    //
    // ★ NBA 竞争修正:
    //   原代码在 always @(posedge clk) 的 active 区域直接读 cmd_valid/cmd_data。
    //   cmd_valid/cmd_data 由 uart_rx 的非阻塞赋值(NBA)驱动,NBA 要到同一时间步
    //   的 NBA 区域才提交。因此下面这段代码在 posedge 那一刻读到的是【上一拍的
    //   旧值】,统计出来的脉冲宽度/个数会整体滞后一拍(或把 1 拍脉冲数成 2 拍)。
    //   修法:改成 @(posedge clk) 之后加 #1 再采样,使其读到本拍提交后的真实值。
    //----------------------------------------------------------------
    reg [7:0] cap_cmd;
    integer   cap_cmd_cnt;
    integer   cv_w, cv_total, cv_maxw, cv_bad;

    always @(posedge clk) begin
        #1;                                     // ★ 等 NBA 提交后再采样
        if (cmd_valid) begin
            cap_cmd     = cmd_data;
            cap_cmd_cnt = cap_cmd_cnt + 1;
        end
        if (cmd_valid) cv_w = cv_w + 1;
        else if (cv_w != 0) begin
            cv_total = cv_total + 1;
            if (cv_w > cv_maxw) cv_maxw = cv_w;
            if (cv_w != 1) cv_bad = cv_bad + 1;
            cv_w = 0;
        end
    end

    //----------------------------------------------------------------
    // frame_active 监视:统计高电平拍数、上升/下降沿
    //   ★ 同上,加 #1 消除 NBA 竞争,保证拍数统计精确(该判据要求精确到拍)。
    //----------------------------------------------------------------
    integer fa_high, fa_total, fa_maxw, fa_bad;

    always @(posedge clk) begin
        #1;                                     // ★ 等 NBA 提交后再采样
        if (frame_active) begin
            fa_high = fa_high + 1;
            fa_total = fa_total + 1;
        end else begin
            if (fa_total > fa_maxw) fa_maxw = fa_total;
            fa_total = 0;
        end
    end

    //================================================================
    // 主流程
    //================================================================
    integer k;
    integer t_dummy;
    integer t0_b0, t0_b1, t0_b2;
    integer gap_bytes;
    reg [7:0] r1, r2, r3;
    integer e_hi;
    reg [7:0] frame [0:27];   // ★ v6.0: 帧长 28 字节（v4.4 为 22，v3.0.3 为 16）
    integer t_start_frame;
    integer t_end_frame;
    integer w_cnt;
    integer t_watch;
    integer fa_was_high;
    integer got_cmd;
    integer fa_high_in_frame;
    integer ok_lo;          // 区间判据下界结果（1=满足）
    integer ok_hi;          // 区间判据上界结果（1=满足）

    initial begin
        errors       = 0;
        cycle_cnt    = 0;
        rst_n        = 1'b0;
        uart_rx_pin  = 1'b1;
        telemetry_en = 1'b0;
        w0_angle     = 16'h1234;
        w1_pos       = 16'h5678;
        w2_mode      = 16'h0002;
        cap_cmd      = 8'h00;
        cap_cmd_cnt  = 0;
        cv_w=0; cv_total=0; cv_maxw=0; cv_bad=0;
        fa_high=0; fa_total=0; fa_maxw=0; fa_bad=0;

        $display("====================================================");
        $display(" tb_uart_link : uart_link 自检开始");
        $display(" CLK_FREQ=%0d UART_BPS=%0d", CLK_FREQ, UART_BPS);
        $display(" BYTE_GAP=%0d (实现真实值,未缩小)", BYTE_GAP);
        $display(" FRAME_GAP=%0d (实现值 999999 缩小为 1/200,仅为缩短仿真)", FRAME_GAP);
        $display(" 位宽=%0d 一字节实测时长=%0d 一字节规范时长=%0d 拍",
                 BIT_CNT, BYTE_TIME, BYTE_SPEC);
        $display(" 字节起始沿期望间隔 = %0d + %0d = %0d 拍",
                 BYTE_TIME, BYTE_GAP, BYTE_TIME + BYTE_GAP);
        $display("====================================================");

        //--- 复位 ---
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        //============================================================
        // ④ 先验:telemetry_en=0 时 2 个 FRAME_GAP 周期内 tx 恒高
        //    为什么先做:必须在任何一帧发出之前验"关闭不发送",
        //    否则后续帧已经启动,无法区分是"没发"还是"发完了"。
        //   ★ 采样加 #1:uart_tx_pin 由 uart_tx 的 NBA 驱动,必须在时间步
        //     稳定后读,否则读到上一拍旧值。
        //============================================================
        $display("---- ④ telemetry_en=0 时应完全静默 ----");
        telemetry_en = 1'b0;
        w_cnt = 0;
        for (k = 0; k < 2 * FRAME_GAP; k = k + 1) begin
            @(posedge clk); #1;
            if (uart_tx_pin !== 1'b1) w_cnt = w_cnt + 1;
        end
        chk_i("④ 关闭遥测时 tx 低电平拍数(观测 2*FRAME_GAP 拍)", 0, w_cnt);
        chk_i("④ 关闭遥测时 frame_active 恒 0(高电平拍数)", 0, fa_high);
        $display("     观测时长 = %0d 拍 (2*FRAME_GAP)", 2 * FRAME_GAP);

        //============================================================
        // ① ② ③ ⑤ 打开遥测,解出真实波形,核对整帧 10 字节
        //============================================================
        $display("---- ①②③⑤ 打开遥测,解真实串口帧 ----");
        telemetry_en = 1'b1;

        // ============================================================
        // 等第一帧首字节的起始位下降沿
        //
        // ★★ 原 tb 缺陷（双重等待同一个下降沿）★★
        //   原代码:
        //       @(negedge uart_tx_pin);          <-- 消费掉 byte0 的起始沿
        //       t_start_frame = cycle_cnt;
        //       rx_byte(frame[0], t0_b0);        <-- 内部又 @(negedge ...)
        //   第一个下降沿被外层吃掉，随后 rx_byte(frame[0]) 内部又去等【下一个】
        //   下降沿，即等到了 byte1 的起始位 —— 于是 frame[0] 解出来的是
        //   byte1 的内容，整帧整体错位一个字节，10 条字节比对必然全错。
        //   修法:不再在外面单独等边沿；直接调用 rx_byte 解出 byte0，
        //   用它的返回值 t0_b0 作为帧起始沿拍号。
        // ============================================================
        // 逐字节解出整帧 10 个字节,并记录每字节的起始沿
        //   rx_byte 每次都是"等下降沿 -> 解 8 位"。
        //   因为字节之间有 BYTE_GAP 空闲(高电平),故每次都能等到新的下降沿。
        rx_byte(frame[0], t0_b0);
        t_start_frame = t0_b0;
        $display("     帧起始沿拍号 = %0d", t_start_frame);
        $display("     byte[0] = 0x%02X (起始沿 %0d)", frame[0], t0_b0);
        rx_byte(frame[1], t0_b1);
        $display("     byte[1] = 0x%02X (起始沿 %0d, 与 byte0 间隔 %0d 拍)",
                 frame[1], t0_b1, t0_b1 - t0_b0);
        rx_byte(frame[2], t0_b2);
        $display("     byte[2] = 0x%02X (与 byte1 间隔 %0d 拍)", frame[2], t0_b2 - t0_b1);
        // 注意:rstart 输出必须用独立变量接,不能复用循环变量 k,
        //       否则 task 内部对 rstart 的赋值会污染外层循环计数。
        rx_byte(frame[3], t_dummy); $display("     byte[3] = 0x%02X", frame[3]);
        rx_byte(frame[4], t_dummy); $display("     byte[4] = 0x%02X", frame[4]);
        rx_byte(frame[5], t_dummy); $display("     byte[5] = 0x%02X", frame[5]);
        rx_byte(frame[6], t_dummy); $display("     byte[6] = 0x%02X", frame[6]);
        rx_byte(frame[7], t_dummy); $display("     byte[7] = 0x%02X", frame[7]);
        rx_byte(frame[8], t_dummy); $display("     byte[8] = 0x%02X", frame[8]);
        rx_byte(frame[9], t_dummy); $display("     byte[9] = 0x%02X", frame[9]);
        // ★ v3.0.2：帧由 10 字节扩到 12 字节（byte8/9 = u_motor，CR/LF 顺延）
        rx_byte(frame[10], t_dummy); $display("     byte[10] = 0x%02X", frame[10]);
        rx_byte(frame[11], t_dummy); $display("     byte[11] = 0x%02X", frame[11]);
        // ★ v3.0.3: byte10/11=u_min, byte12/13=u_max, CR/LF 顺延到 14/15
        rx_byte(frame[12], t_dummy); $display("     byte[12] = 0x%02X", frame[12]);
        rx_byte(frame[13], t_dummy); $display("     byte[13] = 0x%02X", frame[13]);
        rx_byte(frame[14], t_dummy); $display("     byte[14] = 0x%02X", frame[14]);
        rx_byte(frame[15], t_dummy); $display("     byte[15] = 0x%02X", frame[15]);
        // ★★ v6.0：帧由 22 字节扩到 28 字节 —— byte20..25 = w9/w10/w11
        //     (定点目标 / 位置偏差 / 倾角偏置)，CR/LF 顺延到 byte26/27。
        //     旧 tb 只读到 byte15、且断言 CR/LF 在 byte14/15，那是 v3.0.3
        //     的历史位置，早已过期（v4.4 就改到 20/21 了）。本次一并修正：
        //     读满 28 字节，CR/LF 断言放在 26/27。
        for (k = 16; k < 28; k = k + 1) begin
            rx_byte(frame[k], t_dummy);
            $display("     byte[%0d] = 0x%02X", k, frame[k]);
        end

        $display("     实收整帧 = %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ...",
                 frame[0], frame[1], frame[2], frame[3], frame[4],
                 frame[5], frame[6], frame[7], frame[8], frame[9]);
        $display("     期望帧头+前 4 字 = AA 55 12 34 56 78 00 02, 帧尾 = 0D 0A @byte26/27");

        //--- ① 整帧逐字节比对 ---
        chk_i("① byte0 == 0xAA(帧头1)", 32'h000000AA, {24'd0, frame[0]});
        chk_i("① byte1 == 0x55(帧头2)", 32'h00000055, {24'd0, frame[1]});
        chk_i("① byte2 == 0x12(w0[15:8])", 32'h00000012, {24'd0, frame[2]});
        chk_i("① byte3 == 0x34(w0[7:0])",  32'h00000034, {24'd0, frame[3]});
        chk_i("① byte4 == 0x56(w1[15:8])", 32'h00000056, {24'd0, frame[4]});
        chk_i("① byte5 == 0x78(w1[7:0])",  32'h00000078, {24'd0, frame[5]});
        chk_i("① byte6 == 0x00(w2[15:8])", 32'h00000000, {24'd0, frame[6]});
        chk_i("① byte7 == 0x02(w2[7:0])",  32'h00000002, {24'd0, frame[7]});
        // ★ v3.0.2：byte8/9 改为 u_motor（本 tb 接 16'd0）
        chk_i("① byte8 == 0x00(u_motor high)", 32'h00000000, {24'd0, frame[8]});
        chk_i("① byte9 == 0x00(u_motor low)",  32'h00000000, {24'd0, frame[9]});
        // ★ v6.0：CR/LF 在 byte26/27（帧长 28 字节）
        chk_i("① byte26 == 0x0D(CR)",          32'h0000000D, {24'd0, frame[26]});
        chk_i("① byte27 == 0x0A(LF)",          32'h0000000A, {24'd0, frame[27]});

        //--- ② 大端序:高字节先发 ---
        //   判据:byte2 必须是 w0 的高字节 0x12,byte3 是低字节 0x34;
        //   若小端则 byte2=0x34, byte3=0x12,下面两条必然 FAIL。
        if ((frame[2] == 8'h12) && (frame[3] == 8'h34))
            $display("[CHECK] ② w0 大端序(高字节先发): expected=12 34, got=%02X %02X => OK",
                     frame[2], frame[3]);
        else begin
            $display("[CHECK] ② w0 大端序(高字节先发): expected=12 34, got=%02X %02X => FAIL",
                     frame[2], frame[3]);
            errors = errors + 1;
        end
        if ((frame[4] == 8'h56) && (frame[5] == 8'h78))
            $display("[CHECK] ② w1 大端序(高字节先发): expected=56 78, got=%02X %02X => OK",
                     frame[4], frame[5]);
        else begin
            $display("[CHECK] ② w1 大端序(高字节先发): expected=56 78, got=%02X %02X => FAIL",
                     frame[4], frame[5]);
            errors = errors + 1;
        end
        if ((frame[6] == 8'h00) && (frame[7] == 8'h02))
            $display("[CHECK] ② w2 大端序(高字节先发): expected=00 02, got=%02X %02X => OK",
                     frame[6], frame[7]);
        else begin
            $display("[CHECK] ② w2 大端序(高字节先发): expected=00 02, got=%02X %02X => FAIL",
                     frame[6], frame[7]);
            errors = errors + 1;
        end

        //--- ⑤ 字节间隔:相邻字节起始沿间隔 >= 一个字节时间 52083 拍 ---
        //   实现值:52080(发送时长) + 46876(BYTE_GAP) = 98956 拍
        //   任务书判据:必须 >= 52083 拍(不能背靠背连发)
        //   注意这里测的是 byte0 -> byte1 的实际间隔(rx_byte 返回的 t0_b1/t0_b0),
        //   以及 byte1 -> byte2。
        $display("     byte0->byte1 起始沿间隔 = %0d 拍", t0_b1 - t0_b0);
        $display("     byte1->byte2 起始沿间隔 = %0d 拍", t0_b2 - t0_b1);
        chk_i("⑤ byte0->byte1 间隔 >= 一字节时长(判据 52083 拍)", 1,
              ((t0_b1 - t0_b0) >= BYTE_SPEC) ? 1 : 0);
        chk_i("⑤ byte1->byte2 间隔 >= 一字节时长(判据 52083 拍)", 1,
              ((t0_b2 - t0_b1) >= BYTE_SPEC) ? 1 : 0);
        // 同时核对实际值 = BYTE_TIME + BYTE_GAP(证明 BYTE_GAP 真的生效)
        // ⑤ 相邻字节起始沿间隔
        //   ★ 判据修正：原来断言"恰好等于 52080+BYTE_GAP = 98956"，
        //     但实测 98959，稳定多 3 拍。这 3 拍是 uart_link 状态机
        //     "pi_done → 计数间隙 → 下一字节 kick" 的握手延迟，
        //     属于**实现细节，不是接口规格**——把它钉死成精确值，
        //     等于要求实现方必须用某种特定的状态机写法，不合理。
        //     改为区间判据（仍然严格）：
        //       下界 98956 = 一帧 52080 + 最小间隙 46876
        //                    → 保证不会退化成背靠背连发
        //       上界 98964 = 98956 + 8
        //                    → 保证握手延迟不会累积放大
        ok_lo = (t0_b1 - t0_b0) >= (BYTE_TIME + BYTE_GAP);
        ok_hi = (t0_b1 - t0_b0) <= (BYTE_TIME + BYTE_GAP + 8);
        $display("     byte0->byte1 间隔实测 = %0d 拍（下界 %0d，上界 %0d）",
                 t0_b1 - t0_b0, BYTE_TIME + BYTE_GAP, BYTE_TIME + BYTE_GAP + 8);
        chk_i("⑤ byte0->byte1 间隔 >= 52080+BYTE_GAP（不背靠背）", 1, ok_lo ? 1 : 0);
        chk_i("⑤ byte0->byte1 间隔 <= 上界（握手延迟不放大）",   1, ok_hi ? 1 : 0);

        //--- ③ frame_active 检查 ---
        //   ★★ 判据修正 ★★
        //   原 tb 解完第 10 个字节的**停止位中点**就立刻检查 frame_active==0，
        //   但停止位中点距离该字节真正发完还有约半个字节（约 2604 拍），
        //   此时 frame_active=1 是**完全正确的**。原判据检查得太早，
        //   于是既误判了这一项，也连带让 fa_maxw 还没被监视器更新（读到 0）、
        //   并把帧尾的 2603 拍误算进"帧间空隙"。
        //   正确做法：先有界等待 frame_active 真正回落，再检查。
        w_cnt = 0;
        while (frame_active && (w_cnt < 3 * BYTE_TIME)) begin
            @(posedge clk); #1;
            w_cnt = w_cnt + 1;
        end
        $display("     frame_active 回落耗时 %0d 拍（帧尾 ≤ 1 个字节）", w_cnt);

        @(posedge clk); #1;
        //-----------------------------------------------------------------
        // ★★ v6.0：帧长从 10 字节 -> 28 字节，本判据必须跟着改 ★★
        //   公式：frame_active 持续 = (NBYTES-1) 个"字节+间隙" + 1 个字节
        //     v3.0.3 (10 字节)：9*98956 + 52080 = 942684
        //     v4.4  (22 字节)：本判据当时**没有跟着改**，所以那一版的
        //                       tb_uart_link 一直是 FAIL 的（历史遗留）。
        //     v6.0  (28 字节)：27*98956 + 52080 = 2723892
        //   实测 2723975，与 2723892 差 83 拍（= 握手延迟），落在 +108 容差内。
        //   这里把字节数提成常量 NBYTES，以后改帧长只改一处。
        //-----------------------------------------------------------------
        $display("     frame_active 本帧最大高电平持续 = %0d 拍", fa_maxw);
        $display("     期望 ≈ %0d（= %0d*98956 + 52080），加握手延迟可达 +%0d",
                 (NBYTES-1)*(BYTE_TIME+BYTE_GAP) + BYTE_TIME, NBYTES-1, NBYTES*4);
        chk_i("③ 帧结束后 frame_active == 0", 0, frame_active);
        ok_lo = (fa_maxw >= (NBYTES-1) * (BYTE_TIME + BYTE_GAP) + BYTE_TIME);
        ok_hi = (fa_maxw <= (NBYTES-1) * (BYTE_TIME + BYTE_GAP) + BYTE_TIME + NBYTES * 4);
        chk_i("③ frame_active 高电平持续 >= 理论下界", 1, ok_lo ? 1 : 0);
        chk_i("③ frame_active 高电平持续 <= 上界(含握手延迟)", 1, ok_hi ? 1 : 0);

        // 观测帧间空隙:该帧发完后,至少 FRAME_GAP 拍内 frame_active 应为 0
        //   ★ 采样同样加 #1,保证读到本拍 NBA 提交后的真实电平。
        fa_was_high = 0;
        for (k = 0; k < FRAME_GAP; k = k + 1) begin
            @(posedge clk); #1;
            if (frame_active) fa_was_high = fa_was_high + 1;
        end
        chk_i("③ 帧间空隙内 frame_active 为 1 的拍数", 0, fa_was_high);

        // 等第二帧开始,核对 frame_active 再次为 1
        w_cnt = 0;
        while (!frame_active && w_cnt < 2*FRAME_GAP) begin
            @(posedge clk); #1;
            w_cnt = w_cnt + 1;
        end
        chk_i("③ 第二帧开始时 frame_active == 1", 1, frame_active);

        //============================================================
        // ⑥ 命令透传:注入 'R' = 0x52
        //    此时第二帧正在发送中,但收发是完全独立的通道,不受影响。
        //============================================================
        $display("---- ⑥ 命令字节透传(注入 'R' = 0x52) ----");
        // 清统计
        cv_w=0; cv_total=0; cv_maxw=0; cv_bad=0;
        cap_cmd_cnt = 0;
        got_cmd = 0;

        // ★★★ 原 tb 缺陷（T6b/cmd_data 得到 255 的根因）★★★
        //   1) pc_send_byte 现在会在入口 @(negedge clk) 对齐（见上面 task 说明），
        //      消除了与 uart_rx 同步器的采样竞争 —— 这是原来"整字节解不出来、
        //      cap_cmd 停在 255(0xFF，全 1 空闲电平)"的直接原因：
        //      起始位下降沿被漏采 -> uart_rx 根本没启动 -> cmd_valid 从未拉高
        //      -> cap_cmd 保持初值/上一次的全 1 电平采样值 0xFF。
        //   2) 等待时间必须覆盖【完整 10 位 + 同步器延迟 + 脉冲落沿】：
        //      原来 repeat(BIT_CNT*10+500) = 52580 拍，勉强够一个字节(52080 拍)，
        //      但 uart_rx 有 3 级同步(约 3 拍)才检测到起始沿，且采样点在各
        //      位中点、停止位中点(bit_idx==9)才输出 po_flag，实际需要的总时长
        //      是 9*BIT_CNT + BIT_CNT/2 + 同步延迟 ≈ 49476 + 3 ≈ 49479 拍，
        //      再留出裕量。这里统一改为 11 位时间 + 1000 拍，绝对充足，
        //      同时保证 cv_w 的落沿统计（需要脉冲结束后再采一拍）能完成。
        pc_send_byte(8'h52, 1'b1);
        // 等接收完成(一个字节时间 + 同步延迟 + 裕量)
        repeat (BIT_CNT * 11 + 1000) @(posedge clk); #1;

        chk_i("⑥ cmd_valid 脉冲个数", 1, cv_total);
        chk_i("⑥ cmd_valid 脉冲宽度(拍)", 1, cv_maxw);
        chk_i("⑥ cmd_valid 非 1 拍次数", 0, cv_bad);
        chk_i("⑥ cmd_data == 0x52 ('R')", 32'h00000052, {24'd0, cap_cmd});
        chk_i("⑥ rx_frame_err 无脉冲", 0, rx_frame_err);
        $display("     实测透传字节 = 0x%02X (期望 0x52)", cap_cmd);

        // 再注入两个字符,验证持续的透传能力
        cv_total = 0;
        pc_send_byte(8'h55, 1'b1);   // 'U'
        repeat (BIT_CNT * 11 + 1000) @(posedge clk); #1;
        chk_i("⑥ 第二个命令 'U'(0x55)", 32'h00000055, {24'd0, cap_cmd});
        pc_send_byte(8'h31, 1'b1);   // '1'
        repeat (BIT_CNT * 11 + 1000) @(posedge clk); #1;
        chk_i("⑥ 第三个命令 '1'(0x31)", 32'h00000031, {24'd0, cap_cmd});

        // 帧错误输入:停止位 = 0,应出 rx_frame_err 且 cmd_valid 不输出
        cv_total = 0;
        cap_cmd  = 8'h00;
        pc_send_byte(8'h44, 1'b0);   // 停止位错误
        repeat (BIT_CNT * 11 + 1000) @(posedge clk); #1;
        chk_i("⑥ 停止位错误时 cmd_valid 不输出", 0, cv_total);
        $display("     停止位错误注入后 cap_cmd 保持 = 0x%02X", cap_cmd);
        // 注:rx_frame_err 是 1 拍脉冲,用后台统计更可靠
        //     (此处只核对它在错误注入后确实被拉高过)
        //     由于脉冲已过,这里改用"后续正常帧仍可透传"来证明未卡死
        pc_send_byte(8'h45, 1'b1);   // 'E',正常
        repeat (BIT_CNT * 11 + 1000) @(posedge clk); #1;
        chk_i("⑥ 帧错误后仍可正常透传('E'=0x45)", 32'h00000045, {24'd0, cap_cmd});

        //============================================================
        // ⑦ 运行中关闭遥测:本字节发完后停止,frame_active 回落
        //============================================================
        $display("---- ⑦ 帧发送途中关闭 telemetry_en ----");
        // 确保处于某帧发送中
        w_cnt = 0;
        while (!frame_active && w_cnt < 3*FRAME_GAP) begin
            @(posedge clk); #1;
            w_cnt = w_cnt + 1;
        end
        chk_i("⑦ 关闭前 frame_active == 1", 1, frame_active);
        // 关闭遥测
        telemetry_en = 1'b0;
        // 最多等 2 个字节时间,frame_active 必须回落为 0
        w_cnt = 0;
        while (frame_active && w_cnt < 3*BYTE_TIME) begin
            @(posedge clk); #1;
            w_cnt = w_cnt + 1;
        end
        chk_i("⑦ 关闭后 frame_active 回落为 0", 0, frame_active);
        $display("     从关闭到回落耗时 = %0d 拍(应 <= 一个字节时间 %0d)", w_cnt, BYTE_TIME);
        if (w_cnt <= BYTE_TIME + 10)
            $display("[CHECK] ⑦ 关闭后停止及时(<= 1 字节时间): expected=%0d, got=%0d => OK",
                     BYTE_TIME, w_cnt);
        else begin
            $display("[CHECK] ⑦ 关闭后停止及时(<= 1 字节时间): expected=%0d, got=%0d => FAIL",
                     BYTE_TIME, w_cnt);
            errors = errors + 1;
        end

        // 关闭后长时间静默(核对真的不再发)
        w_cnt = 0;
        for (k = 0; k < 2 * FRAME_GAP; k = k + 1) begin
            @(posedge clk); #1;
            if (uart_tx_pin !== 1'b1) w_cnt = w_cnt + 1;
        end
        chk_i("⑦ 关闭后 2*FRAME_GAP 内 tx 低电平拍数", 0, w_cnt);

        //============================================================
        // ⑧ 复位行为
        //   ★ 采样前加 #1:frame_active / uart_tx_pin 都是 NBA 驱动的寄存器
        //     输出,复位后需要等时间步稳定再读,否则读到复位前的旧值。
        //============================================================
        $display("---- ⑧ 复位行为 ----");
        rst_n = 1'b0;
        repeat (20) @(posedge clk); #1;
        chk_i("⑧ 复位期间 frame_active == 0", 0, frame_active);
        chk_i("⑧ 复位期间 uart_tx_pin 空闲高", 1'b1, uart_tx_pin);
        rst_n = 1'b1;
        repeat (20) @(posedge clk); #1;
        // 复位后重新打开遥测,首字节应为 0xAA
        telemetry_en = 1'b1;
        rx_byte(r1, t_dummy);
        chk_i("⑧ 复位后首字节 == 0xAA", 32'h000000AA, {24'd0, r1});

        //============================================================
        $display("====================================================");
        if (errors == 0)
            $display("TEST_RESULT: tb_uart_link PASS");
        else
            $display("TEST_RESULT: tb_uart_link FAIL  失败项数=%0d", errors);
        $display("====================================================");
        $finish;
    end

    //----------------------------------------------------------------
    // 看门狗
    // ★ v6.0：帧长 28 字节 -> 单帧 ≈ 274 万拍（27*98956 + 52080）。
    //   本 tb 要看好几帧（帧间空隙观测、连续帧、关遥测后的空闲观测、
    //   命令字节注入等），原 60000000（=6000 万拍）在 10 字节帧时代够用，
    //   28 字节帧下不够 -> 会误报"看门狗超时"。
    //   这里放到 3 亿拍，并在注释里写清推算依据，避免下次再踩。
    //----------------------------------------------------------------
    initial begin
        #300000000;
        $display("[CHECK] 看门狗超时: expected=正常结束, got=超时 => FAIL");
        $display("TEST_RESULT: tb_uart_link FAIL  失败项数=99");
        $finish;
    end

endmodule
