// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: pendulum_top
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: J280 旋转倒立摆控制系统顶层模块
//           1. 例化互联全部子模块：传感采集、数字滤波、PID 运算、轨迹规划、驱动、人机接口
//           2. 控制节拍 3ms 内环角度环、30ms 外环位置环，构成串级闭环
//           3. 人机接口：板载 SW1 启动 / SW2 正弦 / SW6 复位 / 底板 5 键 / OLED / 9600bps 遥测
//           4. 系数由 pid_tuner 在运行期给出，可串口在线整定；遥测帧常开
// =============================================================================

`timescale 1ns/1ns

module pendulum_top (
    // ---- 系统 ----
    input  wire        clk_50m,       // M19  50MHz
    input  wire        rst_n,         // AB3  复位，低有效
    input  wire [3:0]  key_sw,        // T18 R18 U20 T20  按键，低有效

    // ---- 串口 ----
    output wire        uart_tx,       // K20  FPGA → PC
    input  wire        uart_rx,       // L20  PC → FPGA

    // ---- ADC 3PA1030 ----
    output wire        adc_clk,       // G18
    output wire        adc_oe_n,      // H20
    input  wire [9:0]  adc_data_in,   // L21 M21 J20 J19 D19 H18 H19 G19 G17 E19

    // ---- XH2 组的电机驱动：本设计使用这一路（对应插在 XH2 的电机）----
    output wire        PWMA,          // Y18
    output wire        AN1,           // Y17
    output wire        AN2,           // Y19

    // ---- XH2 组的编码器：本设计使用这一路（插在 XH2 的编码器）----
    input  wire        enc2_a,        // W18
    input  wire        enc2_b,        // W17

    // ---- XH1 组：本设计不使用（电机驱动脚恒置停机电平，编码器不使用）----
    output wire        PWMB,          // V14
    output wire        BN1,           // AA17
    output wire        BN2,           // V15
    input  wire        enc_a,         // Y20  未使用
    input  wire        enc_b,         // AA20 未使用

    // ---- LED ----
    output wire [3:0]  led,           // U18 U19 U17 T17  高电平点亮（不取反）

    // 底板 5 个命令键：ext_key[0]=SW1 启动  [1]=SW2 +100  [2]=SW3 -100
    //                 [3]=SW4 M（进入位置指令编辑）  [4]=SW5 !
    //   全部低有效、另一端接 GND，靠 FPGA 内部上拉；消抖 20ms 在 ext_keys 里做。
    input  wire [4:0]  ext_key,

    // 板上 SW6 是【复位】键，球号 K22（B2_33A_K22，H2 的 20 针）。
    input  wire        ext_rst_key,

    // OLED：SSD1306（128x64）I2C 接口，开漏
    output wire        oled_scl,
    inout  wire        oled_sda
);

    localparam [11:0] UP_REF_DEFAULT   = 12'd2855;   // 本设计 ADC 下竖直向上的读数
    localparam [11:0] DOWN_REF_DEFAULT = 12'd793;    // 本设计 ADC 下自然下垂的读数

    //=========================================================================
    // 1. 时钟与复位管理（clk_mgr：复位同步释放，不含 PLL）
    //=========================================================================
    wire clk_sys;
    wire rst_sys_n;

    // 板上 SW6 (K22) 复位键作为第二复位源。
    wire ext_rst_n;          // 显式声明，避免隐式网线

    key_filter #(
        .CNT_MAX (20'd1_000_000)     // 20ms @50MHz
    ) u_rst_key (
        .clk       (clk_50m),
        .rst_n     (rst_n),
        .key_in    (ext_rst_key),
        .key_flag  (),               // 复位用不到"按下脉冲"，用不到就空着
        .key_state (ext_rst_n)
    );

    clk_mgr #(
        .RST_SYNC_CYCLES (8)
    ) u_clk_mgr (
        .clk_50m   (clk_50m),
        .rst_n     (rst_n),
        // 外部"复位"键并入 rst_sys_n。
        .ext_rst_n (ext_rst_n),
        .clk_sys   (clk_sys),
        .rst_sys_n (rst_sys_n)
    );

    wire key_run_press;       // SW1 按下时的 1 拍脉冲（启动/停止）

    key_filter #(
        .CNT_MAX (20'd1_000_000)      // 20ms @50MHz
    ) u_key_run (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .key_in    (key_sw[0]),       // SW1 = T18 = 启动/停止
        .key_flag  (key_run_press),
        .key_state ()                 // 消抖电平本设计不用，留空
    );

    wire key_sine_press;      // SW2 按下时的 1 拍脉冲（正弦模式切换）

    key_filter #(
        .CNT_MAX (20'd1_000_000)      // 20ms @50MHz
    ) u_key_sine (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .key_in    (key_sw[1]),       // SW2 = R18 = 正弦开关
        .key_flag  (key_sine_press),
        .key_state ()
    );

    // 正弦模块的输出，显式声明避免隐式网线。
    wire signed [15:0] sine_mod;      // 正弦调制量（叠加到 traj_ramp 的目标）
    wire               sine_on;       // 1 = 正弦模式生效中

    //=========================================================================
    // 2. ADC 采集（3PA1030）—— 同时产生整个控制环的控制节拍
    //=========================================================================
    wire [11:0] angle_raw;
    wire        angle_vld;        // 控制节拍 tick

    angle_adc #(
        // 采样参数与厂商 adc.v 一致：3ms 窗口内累加 16 点后 >>4
        .SMP_3MS  (149_999),
        .SMP_STEP (  9_373)   // 每 187.5us 采一点：9374×16 = 149984 拍 ≈ 3ms
    ) u_angle_adc (
        .clk_50m      (clk_sys),
        .rst_n        (rst_sys_n),
        .adc_clk      (adc_clk),
        .adc_oe_n     (adc_oe_n),
        .adc_data_in  (adc_data_in),
        .angle_raw    (angle_raw),
        .angle_vld    (angle_vld)
    );

    wire ctrl_tick = angle_vld;

    //=========================================================================
    // 3. 正交编码器解码（XH2 / 编码器 2）
    //=========================================================================
    wire [15:0]        pos_raw;
    wire signed [15:0] pos_rel;
    wire signed [15:0] pos_delta;    // 本拍增量（保留，供扩展测速）

    qep_encoder #(
        .POS_INIT (16'd10000)            // 厂商上位机约定初值 10000
    ) u_qep_encoder (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        //=========================================================================
        // 编码器接 XH2（编码器 2）
        //=========================================================================
        .enc_a     (enc2_a),
        .enc_b     (enc2_b),
        .pos_raw   (pos_raw),
        .pos_rel   (pos_rel),
        .pos_delta (pos_delta)
    );

    //=========================================================================
    // 4. 串口（遥测上行 + 命令下行）
    //=========================================================================
    // cmd_data / cmd_valid 由 uart_link 输出，与底板按键合并后统一译码
    wire        rx_frame_err;
    wire        frame_active;

    //-------------------------------------------------------------------------
    // 串口命令总线 + 外部按键注入 的合并
    //-------------------------------------------------------------------------

    // ---- 参考点寄存器 ----
    reg  [11:0] up_ref;
    reg  [11:0] down_ref;

    // ---- 系数（来自 pid_tuner 的寄存器，可串口在线修改）----
    //   端口是无符号位模式，这里按有符号接出，PID 模块按 Q10 / Q12 解释
    wire signed [31:0] kp_th_w, ki_th_w, kd_th_w, kp_x_w, kd_x_w;
    wire [31:0]       swing_duty_w32;                   // 起摆占空比
    wire signed [15:0] swing_duty_w = swing_duty_w32[15:0];
    wire        telemetry_en;                          // 'S' 遥测开关
    wire [2:0]  sel_loop;                              // 0..5 选系数，6 位置指令编辑
    wire        tuner_run_en;                          // 'R' 运行 / 停止
    wire        tuner_up_capture;                      // 'U' 记录直立参考
    wire        tuner_down_capture;                    // 'D' 记录下垂参考
    wire        tuner_pump_pol;                        // 'P' 翻转起摆极性
    wire        tuner_pos_mode;                        // 'A' / 'M' / '!' -> 追随定点
    wire        tuner_zero_req;                        // '0' 定点归零
    wire [15:0] tuner_last_cmd;                        // 最近一条命令（观测用）

    // 遥测数据源
    wire [15:0] tel_w0 = {4'b0, angle_raw};   // 摆杆角度（原始 ADC 值，0~4090）
    wire [15:0] tel_w1 = pos_raw;             // 摆臂编码器位置（初值 10000）
    wire [15:0] tel_w2;                       // 模式（由 ctrl_fsm 驱动）
    // 外环观测量：x_settle 定点目标、err_x 位置偏差、phi_bias 倾角偏置
    wire signed [15:0] x_cmd;                 // 限速后的定点（traj_ramp 输出）
    wire signed [15:0] x_settle;              // 定点目标（送 OLED 的 SET 行）
    wire               traj_busy;             // 斜坡仍在逼近定点
    wire signed [15:0] err_x;                 // 位置偏差（pos_ctrl 输出）
    wire signed [15:0] phi_bias;              // 倾角偏置（pos_pd 输出，外环实际输出）
    wire signed [15:0] tel_w9  = x_settle;    // 定点目标
    wire signed [15:0] tel_w10 = err_x;       // 位置偏差
    wire signed [15:0] tel_w11 = phi_bias;    // 倾角偏置（外环实际输出）

    // u_motor 声明放在仲裁 always 块之前，避免隐式网线
    reg signed [15:0] u_motor;

    localparam [11:0] HALF_TURN = 12'd2059;   // 摆杆 180° 的计数差（标定值）

    wire [15:0]        phi_raw_u = {4'b0, angle_raw} - {4'b0, up_ref};
    wire signed [16:0] phi_raw   = {phi_raw_u[15], phi_raw_u};   // 符号扩展

    // 按传感器整圈 4096 折叠到 [-2048, +2047]
    wire signed [16:0] phi_fold =
        (phi_raw >  17'sd2048) ? (phi_raw - 17'sd4096) :
        (phi_raw < -17'sd2048) ? (phi_raw + 17'sd4096) :
                                 phi_raw;

    wire signed [15:0] phi = phi_fold[15:0];

    //=========================================================================
    // 5. 状态机输出的运行使能（必须在被 ang_deriv / limit_prot 引用前声明）
    //=========================================================================
    wire [1:0] mode;
    wire       run_pid;
    wire       run_swing;
    wire       run_pos;

    wire up_cap_now   = tuner_up_capture;
    wire down_cap_now = tuner_down_capture;

    // 注意：up_ref / down_ref 的寄存器更新逻辑统一放在文件后部

    //=========================================================================
    // 6. 位置指令通路（pos_cmd 编辑缓冲 + ext_keys 按键注入）
    //=========================================================================
    // OLED 的 SET（定点目标）/ PEND（待发送量）两行要显示真实数值，
    // 因此需要一条"编辑 -> 下发"通路：
    //   · ext_keys  —— 底板 SW1..SW5（启动 / +100 / -100 / M / !），消抖 + 合流
    //   · pos_cmd   —— 待发送量的编辑缓冲（±100 步进、双向饱和 ±2000）
    //   · pid_tuner —— 只需负责 'M'（进编辑模式）/ '!'（发送）的译码
    //   · 串口命令并入同一条译码链，上位机发 'M'/'+'/'!' 与按键等效
    // 定点目标的变化只来自两处：pos_cmd 的数值增量，以及 '0' 归零命令。
    //=========================================================================
    wire [7:0]  cmd_data;        // 串口命令字节（uart_link 输出）
    wire        cmd_valid;       // 1 拍有效

    wire ek_run, ek_inc, ek_dec;
    wire ek_cmd_valid_all;
    wire [7:0] ek_cmd_data_all;

    ext_keys #(
        .CNT_MAX (20'd1_000_000)      // 20ms @50MHz，与板载 SW1 一致
    ) u_ext_keys (
        .clk           (clk_sys),
        .rst_n         (rst_sys_n),
        .key_in        (ext_key),
        .cmd_valid     (cmd_valid),
        .cmd_data      (cmd_data),
        .run_pulse     (ek_run),
        .inc           (ek_inc),
        .dec           (ek_dec),
        .cmd_valid_all (ek_cmd_valid_all),
        .cmd_data_all  (ek_cmd_data_all)
    );

    // 串口命令与外部按键的合并（按键优先；合并动作在 ext_keys 内部完成）
    wire        cmd_valid_all = ek_cmd_valid_all;
    wire [7:0]  cmd_data_all  = ek_cmd_data_all;

    //=========================================================================
    // 7. 串口在线调参 + 动作脉冲（pid_tuner）
    //=========================================================================
    //   保存 6 个系数与若干动作脉冲，复位值来自下面的 *_INIT（无掉电保存）。
    //   命令：'1'..'6' 选通道，'>' '<' '*' '/' '-' '+' 修改；
    //        'R' 运行/停止、'S' 遥测、'U'/'D' 标定直立/下垂参考、
    //        'P' 翻转起摆极性、'A' 定点模式、'0' 定点归零、'M'/'!' 位置指令。
    pid_tuner #(
        .Q_FMT      (10),              // 系数 Q10（1.0 = 1024）
        .KP_TH_INIT (32'sd15507),      // 角度环 P（Q10）
        .KI_TH_INIT (32'sd256),        // 角度环 I（Q10）
        .KD_TH_INIT (32'sd2034),       // 角度环 D（Q10）
        .KP_X_INIT  (-32'sd845),       // 位置环 P（Q12）
        .KD_X_INIT  (-32'sd21730)      // 位置环 D（Q12）
    ) u_pid_tuner (
        .clk              (clk_sys),
        .rst_n            (rst_sys_n),
        .cmd_data         (cmd_data_all),
        .cmd_valid        (cmd_valid_all),
        .kp_th            (kp_th_w),
        .ki_th            (ki_th_w),
        .kd_th            (kd_th_w),
        .kp_x             (kp_x_w),
        .kd_x             (kd_x_w),
        .swing_duty       (swing_duty_w32),
        .sel_loop         (sel_loop),
        .run_en           (tuner_run_en),
        .telemetry_en     (telemetry_en),
        .up_ref_capture   (tuner_up_capture),
        .down_ref_capture (tuner_down_capture),
        .pump_pol_toggle  (tuner_pump_pol),
        .pos_mode         (tuner_pos_mode),
        .zero_req         (tuner_zero_req),
        .last_cmd         (tuner_last_cmd)
    );

    // 位置指令编辑模式：由 pid_tuner 的 sel_loop 表示（按 'M' 置 6）
    wire pos_cmd_active = (sel_loop == 3'd6);

    wire signed [15:0] pos_pend;         // 待发送增量（送 OLED 的 PEND 行）
    wire               pos_send_pulse;   // 1 拍：把增量加进定点
    wire signed [15:0] pos_cmd_delta;    // 本次要加的增量

    pos_cmd #(
        .W                (16),
        // 单次上限 3 圈 = 3 × 1067 = 3201 counts，与 traj_ramp 的
        // TURN_LIMIT 取成相等（输多少走多少）
        .DELTA_MAX        (16'sd3201)
    ) u_pos_cmd (
        .clk              (clk_sys),
        .rst_n            (rst_sys_n),
        .cmd_valid        (cmd_valid_all),
        .cmd_data         (cmd_data_all),
        .pos_mode_active  (pos_cmd_active),
        .key_inc          (ek_inc),     // 底板 +100
        .key_dec          (ek_dec),     // 底板 -100
        .pos_pend         (pos_pend),
        .pos_send_pulse   (pos_send_pulse),
        .pos_delta        (pos_cmd_delta)
    );

    //=========================================================================
    // 8. 运行请求
    //=========================================================================

    // 三个来源：板载 SW1、底板 ext_key[0]、串口 'R'（取 tuner_run_en 的两个边沿）
    reg  tuner_run_r;

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) tuner_run_r <= 1'b0;
        else            tuner_run_r <= tuner_run_en;
    end

    wire tuner_run_toggle;
    assign tuner_run_toggle = tuner_run_en ^ tuner_run_r;   // 两个边沿都产生脉冲
    wire run_req = key_run_press | tuner_run_toggle | ek_run;

    //=========================================================================
    // 9. 角速度微分 + X 阶数字滤波
    //=========================================================================
    wire signed [15:0] dtheta;
    wire signed [15:0] dx;
    wire signed [15:0] dtheta_hi;
    wire signed [15:0] dx_hi;

    ang_deriv #(
        .W          (16),
        .FILT_TAPS  (4),      // 4 阶滑动平均（2 的幂 → 用 >>>2 实现）
        .FILT_SHIFT (2),      // = log2(4)
        .SUM_W      (20)      // 8 x 最大差分 4095 = 32760 < 2^20，不溢出
    ) u_ang_deriv (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .tick     (ctrl_tick),
        .run_en   (run_pid | run_swing),   // 只在真正闭环时才微分，避免停机积累
        .theta_in (phi),
        .x_in     (pos_rel),
        .dtheta   (dtheta),
        .dx       (dx),
        .dtheta_hi(dtheta_hi),      // ×64 高分辨率差分
        .dx_hi    (dx_hi)           // 同上
    );

    //=========================================================================
    // 10. 限位保护
    //=========================================================================
    wire fault;
    wire arm_fault;
    wire ang_fault;

    limit_prot #(
        // 角度量程检查关闭：MIN=0 / MAX=4095，量程内恒通过
        .ANGLE_MIN (12'd0),
        .ANGLE_MAX (12'd4095),
        .POS_LIMIT (16'd6000)
    ) u_limit_prot (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .tick      (ctrl_tick),
        .run_en    (run_pid | run_swing),
        .pos_rel   (pos_rel),
        .angle_raw (angle_raw),
        .arm_fault (arm_fault),
        .ang_fault (ang_fault),
        .fault     (fault)
    );

    //=========================================================================
    // 11. 状态机（由它决定本轮谁输出控制量）
    //=========================================================================
    ctrl_fsm #(
        .W            (16),
        .CAPTURE_BAND (16'sd130),
        .CAPTURE_RATE (16'sd55),
        .LOST_BAND    (16'sd420),
        .LOST_HOLD    (12)
    ) u_ctrl_fsm (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .tick      (ctrl_tick),
        .run_req   (run_req),
        .fault     (fault),
        .phi       (phi),
        .dtheta    (dtheta),
        .mode      (mode),
        .run_pid   (run_pid),
        .run_swing (run_swing),
        .run_pos   (run_pos)
    );

    // mode 直接作为遥测 w2：0 = 空闲, 1 = 起摆, 2 = 平衡 (3 = 故障)
    assign tel_w2 = {14'b0, mode};

    wire pos_mode_eff = (mode == 2'd2) & (tuner_pos_mode | sine_on);

    wire signed [15:0] x_final = pos_mode_eff ? x_cmd : 16'sd0;

    //=========================================================================
    // 12. 正弦轨迹发生器（固定 A=200 counts / f=0.2Hz，SW2 开关）
    //=========================================================================
    traj_sine #(
        .W           (16),
        .AMP         (200),          // 幅值 counts（单边，≈67.5 度）
        .PHASE_INC   (24'd10066)     // f = 10066/(2^24*0.003) = 0.19999 Hz（周期 5.0 s）
    ) u_traj_sine (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .tick      (ctrl_tick),      // 3ms
        .run_en    (run_pos),        // 1 = BALANCE
        .key_pulse (key_sine_press), // 板载 SW2
        .x_mod     (sine_mod),
        .sine_on   (sine_on)
    );

    traj_ramp #(
        .W           (16),
        .SLEW        (16'sd2),          // 每 3ms 最多 2 counts（= 667 counts/s）
        .TURN_LIMIT  (16'sd3201)        // 累计上限 = pos_cmd 的 DELTA_MAX（3 圈）
    ) u_traj_ramp (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .tick      (ctrl_tick),
        .run_en    (run_pos),
        .zero_req  (tuner_zero_req),        // '0' 定点归零
        .delta_ext       (pos_cmd_delta),
        .delta_ext_valid (pos_send_pulse),
        .x_mod     (sine_mod),         // 正弦调制量（非正弦模式恒 0）
        .x_pos     (pos_rel),
        .x_cmd     (x_cmd),
        .x_settle  (x_settle),         // 送 OLED 的 SET 行
        .busy      (traj_busy)
    );

    pos_ctrl #(
        .W          (16),
        // 位置死区 3 counts（≈0.2 度），抑制过零附近的抖动
        .X_DEADBAND (16'sd3)
    ) u_pos_ctrl (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .tick     (ctrl_tick),
        .run_en   (run_pos),
        .x_target (x_final),      // 定点目标（追随闸关闭时为 0）
        .pos_rel  (pos_rel),
        // 正弦模式旁路位置死区，避免过零附近出现"平顶"造成相位失真
        .dead_en  (~sine_on),
        .err_x    (err_x)
    );

    //=========================================================================
    // 13. 【核心】流水线 PID
    //=========================================================================
    wire signed [15:0] u_pid;
    wire               u_pid_vld;
    wire               u_pid_sat;

    // 流水线各抽头（接在线逻辑分析仪观察，顶层不驱动任何外设）
    wire signed [47:0] dbg_s2_p_term;
    wire signed [47:0] dbg_s2_px_term;
    wire signed [47:0] dbg_s3_d_term;
    wire signed [47:0] dbg_s3_dx_term;
    wire signed [55:0] dbg_s3_i_acc;
    wire signed [55:0] dbg_s4_sum;

    localparam signed [15:0] PHI_BIAS_MAX  = 16'sd250;   // 幅值上限，约 ±21.6 度
    localparam signed [15:0] PHI_BIAS_RATE = 16'sd2;     // 每 3ms 周期最多变化 2 counts
                                                         // 0 -> 250 需 375ms，外环带宽约 2.7 rad/s

    pos_pd #(
        .W         (16),
        .GAIN_W    (32),
        .SUM_W     (56),
        .Q_FMT     (12),              // 与 kp_x/kd_x 的 Q12 标度一致（>>12）
        .DIV       (10),              // 外环 30ms（内环 3ms -> 1:10）
        .BIAS_MAX  (PHI_BIAS_MAX),
        .BIAS_RATE (PHI_BIAS_RATE)
        // 外环是纯 PD：只需 Kp / Kd 两个系数
    ) u_pos_pd (
        .clk       (clk_sys),
        .rst_n     (rst_sys_n),
        .ctrl_tick (ctrl_tick),
        .run_en    (run_pos),         // 与 pos_ctrl 共用同一个投入使能
        .err_x     (err_x),
        .kp_x      (kp_x_w),
        .kd_x      (kd_x_w),          // KD_X 走这里（角度偏置通道）
        .phi_bias  (phi_bias)
    );

    // 内环实际使用的偏差：位置环不动 u，只移动"目标角度"
    wire signed [15:0] phi_eff = phi - phi_bias;

    pid_pipeline #(
        // 这里的参数覆盖【优先于】pid_pipeline.v 里的默认值 
        .Q_FMT  (10),
        .ERR_W  (16),
        .GAIN_W (32),
        .PROD_W (48),
        .ACC_W  (56),
        .SUM_W  (56),
        .OUT_W  (16)
    ) u_pid_pipeline (
        .clk        (clk_sys),
        .rst_n      (rst_sys_n),
        .ctrl_tick  (ctrl_tick),
        .run_en     (run_pid),
        .err_th     (phi_eff),   // 减去位置偏置后的角度偏差
        .dth        (dtheta_hi),   // ×64 高分辨率差分
        .err_x      (16'sd0),    // 位置环不直接进控制量：它靠移动角度设定值调节内环，此处接 0
        .dx         (dx_hi),       // 同上，高分辨率差分
        .kp_th      (kp_th_w),
        .ki_th      (ki_th_w),
        .kd_th      (kd_th_w),
        // 常量位宽要与端口一致：kp_x 端口是 GAIN_W=32 位，故写成 32'sd0
        .kp_x       (32'sd0),    // 位置环 P 不参与控制量（由 pos_pd 产生 phi_bias）
        // 位置环 D 也从这里断开，改由 pos_pd 串联进角度偏置。
        .kd_x       (32'sd0),
        .u_ctrl     (u_pid),
        .u_vld      (u_pid_vld),
        .s5_sat     (u_pid_sat),
        .s2_p_term  (dbg_s2_p_term),
        .s2_px_term (dbg_s2_px_term),
        .s3_d_term  (dbg_s3_d_term),
        .s3_dx_term (dbg_s3_dx_term),
        .s3_i_acc   (dbg_s3_i_acc),
        .s4_sum     (dbg_s4_sum)
    );

    //=========================================================================
    // 14. 起摆能量泵
    //=========================================================================
    wire signed [15:0] u_swing;
    wire               captured;

    swing_pump #(
        .W             (16),
        // 起摆占空比运行时可调（走 .swing_duty 端口），此处参数只作默认值
        .CAPTURE_BAND  (16'sd130),
        .CAPTURE_RATE  (16'sd55),
        .KICK_MIN_DTH  (16'sd1),
        .KICK_DUTY     (16'sd1400),
        .KICK_DIR      (1),
        .PUMP_POLARITY (1)          // 极性反了摆杆会越摆越矮，此时取 -1
    ) u_swing_pump (
        .clk      (clk_sys),
        .rst_n    (rst_sys_n),
        .tick     (ctrl_tick),
        .run_en   (run_swing),
        .phi      (phi),
        .dtheta   (dtheta),
        .swing_duty (swing_duty_w),   // 起摆占空比，运行时可调
        .u_swing  (u_swing),
        .captured (captured)
    );

    //-------------------------------------------------------------------------
    // 起摆能量泵极性：运行时可翻转
    //-------------------------------------------------------------------------
    reg swing_pol;

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)          swing_pol <= 1'b1;   // 上电用 swing_pump 参数默认极性
        else if (tuner_pump_pol) swing_pol <= ~swing_pol;
    end

    //=========================================================================
    // 15. 控制量仲裁（状态机决定谁接管电机）
    //     优先级：故障停机 > 平衡环 > 起摆能量泵 > 空闲
    //=========================================================================

    always @(*) begin
        if (fault) begin
            u_motor = 16'sd0;                 // 故障：立即停机（最高优先级）
        end
        else if (run_pid) begin
            u_motor = u_pid;                  // 平衡环
        end
        else if (run_swing) begin
            // 起摆能量泵：按 swing_pol 决定是否取反（串口 'P' 可现场翻转）
            u_motor = swing_pol ? u_swing : -u_swing;
        end
        else begin
            u_motor = 16'sd0;                 // 空闲
        end
    end

    //=========================================================================
    // 16. 电机驱动（XH2 / A 路）
    //=========================================================================
    motor_drv #(
        .PWM_PERIOD (13'd5000),      // 50MHz/5000 = 10kHz 载波
        .DUTY_MAX   (13'd5000),
        .DEADBAND   (16'd20)
    ) u_motor_drv (
        .clk    (clk_sys),
        .rst_n  (rst_sys_n),
        .u_ctrl (u_motor),
        .pwm    (PWMA),
        .in1    (AN1),
        .in2    (AN2)
    );

    //=========================================================================
    // 17. 电机通道选择：驱动 A 路（PWMA/AN1/AN2）
    //=========================================================================
    assign PWMB = 1'b0;     // V14  B 路（本机未接电机）
    assign BN1  = 1'b0;     // AA17
    assign BN2  = 1'b0;     // V15

    reg up_ref_explicit;   // 1 = 用户已用串口 'U' 显式记录过直立参考

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            up_ref          <= UP_REF_DEFAULT;
            down_ref        <= DOWN_REF_DEFAULT;
            up_ref_explicit <= 1'b0;
        end
        else begin
            if (up_cap_now) begin
                up_ref          <= angle_raw;
                up_ref_explicit <= 1'b1;
            end
            else if (down_cap_now && !up_ref_explicit) begin
                // 只记录了下垂参考时，自动推算直立参考
                up_ref <= angle_raw + HALF_TURN;
            end

            if (down_cap_now)
                down_ref <= angle_raw;
        end
    end

    //-------------------------------------------------------------------------
    // 串口帧错误锁存：LED4 常亮提示接线/波特率问题（标定时顺带清除）
    //-------------------------------------------------------------------------
    reg rx_err_sticky;

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)
            rx_err_sticky <= 1'b0;
        else if (rx_frame_err)
            rx_err_sticky <= 1'b1;
        else if (up_cap_now || down_cap_now)
            rx_err_sticky <= 1'b0;
    end

    //=========================================================================
    // 18. LED 指示（高电平点亮：led[n] 直接接高有效信号）
    //=========================================================================
    reg [24:0] hb_cnt;

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) hb_cnt <= 25'd0;
        else            hb_cnt <= hb_cnt + 25'd1;
    end

    wire hb_led   = hb_cnt[24];                        // 50e6/2^25 ≈ 1.49Hz

    wire run_led  = (mode != 2'd0);                    // 运行中
    wire bal_led  = (mode == 2'd2);                    // 已平衡
    // 告警：限位/角度故障、PID 输出饱和、串口帧错误（锁存）、定点未到位
    wire warn_led = fault | arm_fault | ang_fault | u_pid_sat | rx_err_sticky | traj_busy;

    //=========================================================================
    // LED 输出分配
    //=========================================================================
    assign led[0] = hb_led;      // LED1 U18 心跳（约 1.49Hz 闪烁）
    assign led[1] = run_led;     // LED2 U19 运行中（mode != 0 时亮）
    assign led[2] = bal_led;     // LED3 U17 已平衡（mode == 2 时亮）
    assign led[3] = warn_led;    // LED4 T17 告警（限位/角度故障、PID饱和、串口帧错）

    wire tx_bin;                          // uart_link 的二进制帧输出（28 字节）

    uart_link #(
        .CLK_FREQ  (26'd50_000_000),
        .UART_BPS  (21'd9600)
    ) u_uart_link (
        .clk            (clk_sys),
        .rst_n          (rst_sys_n),
        .uart_rx_pin    (uart_rx),
        .uart_tx_pin    (tx_bin),      // 中间线，最终 assign 到 uart_tx
        .telemetry_en   (telemetry_en),// 'S' 命令开关遥测
        .w0_angle       (tel_w0),
        .w1_pos         (tel_w1),
        .w2_mode        (tel_w2),
        .w3_umotor   (u_motor),        // 最终 PWM 输出
        // 控制环内部量：P 项、D 项、PD 和，便于分段观察问题出在哪一环
        .w4_phieff   (phi_eff),                          // 内环角度偏差
        .w5_dphi     (dtheta),                           // 摆杆角速度（未放大）
        .w6_pterm    (dbg_s2_p_term[25:10]),             // P 项 >>10
        .w7_dterm    (dbg_s3_d_term[25:10]),             // D 项 >>10
        .w8_pdsum    (dbg_s4_sum[25:10]),                // PD 相加 >>10
        // 外环三个观测量：定点目标、位置偏差、倾角偏置
        .w9_xset     (x_settle),                         // 定点目标
        .w10_errx    (err_x),                            // 位置偏差
        .w11_phibias (phi_bias),                         // 倾角偏置（外环实际输出）
        .frame_active   (frame_active),
        // 串口命令与底板按键走同一条译码链
        .cmd_data       (cmd_data),
        .cmd_valid      (cmd_valid),
        .rx_frame_err   (rx_frame_err)
    );


    // UART 只发 28 字节二进制遥测帧
    assign uart_tx = tx_bin;

    //=========================================================================
    // 19. OLED 显示（SSD1306, I2C 128x64）
    //---------------------------------------------------------------------
    // 结构：
    // oled_disp      —— 把控制量排成 5 行文字，再渲染成 8 页 x 128 列位图
    //=========================================================================
    wire        oled_scl_lo;
    wire        oled_sda_lo;

    // 开漏：拉低 = 驱动 0，释放 = 高阻（由 OLED 模块板载上拉拉高）
    assign oled_scl = oled_scl_lo ? 1'b0 : 1'bz;
    assign oled_sda = oled_sda_lo ? 1'b0 : 1'bz;

    // 帧缓冲写口（oled_disp -> oled_ssd1306）
    wire [9:0]  oled_fb_addr;
    wire [7:0]  oled_fb_data;
    wire        oled_fb_we;

    // 握手
    wire        oled_frame_req;
    wire        oled_disp_done;

    oled_disp u_oled_disp (
        .clk        (clk_sys),
        .rst_n      (rst_sys_n),
        .mode       (mode),
        .cur_cnt    (pos_rel),          // 摆臂位置（相对零位）
        .set_cnt    (x_settle),         // 定点目标，送 SET 行
        .pend_cnt   (pos_pend),         // 待发送量，送 PEND 行
        .run_flag   (run_pid | run_swing),
        .fault_flag (fault),
        .edit_flag  (pos_cmd_active | sine_on),   // 编辑态或正弦模式
        .fb_addr    (oled_fb_addr),
        .fb_data    (oled_fb_data),
        .fb_we      (oled_fb_we),
        .frame_req  (oled_frame_req),
        .disp_done  (oled_disp_done)
    );

    oled_ssd1306 #(
        .QDIV (62)                  // 约 198kHz 的 SCL
    ) u_oled (
        .clk        (clk_sys),
        .rst_n      (rst_sys_n),
        .scl_lo     (oled_scl_lo),
        .sda_lo     (oled_sda_lo),
        .sda_i      (oled_sda),     // 直接读回引脚，用于采样 ACK
        .fb_we      (oled_fb_we),
        .fb_addr    (oled_fb_addr),
        .fb_data    (oled_fb_data),
        .frame_req  (oled_frame_req),
        .disp_done  (oled_disp_done),
        .ack_err    (),             // 未接：留空备用
        .busy       ()
    );

endmodule
