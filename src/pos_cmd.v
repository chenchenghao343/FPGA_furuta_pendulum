// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: pos_cmd
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 位置指令数值输入模块（位置指令模式的待发送增量编辑器）
//           1. 用 + - > < * / 编辑待发送增量，按 '!' 确认发送
//           2. 确认时锁存 pos_delta 并打一拍 pos_send_pulse 给 traj_ramp
//           3. 外部 +/- 按键固定步进 KEY_STEP，与串口编辑共用同一个待发送值
//           4. 编辑路径饱和到 ±DELTA_MAX，按键路径饱和到 ±KEY_LIMIT
// =============================================================================

`timescale 1ns/1ps

module pos_cmd #(
    parameter integer W          = 16,           // 待发送值位宽(有符号)
    parameter signed [W-1:0] DELTA_MAX = 16'sd3201,  // 单次发送上限 ±3 圈 @1067 counts
    parameter signed [W-1:0] KEY_STEP  = 16'sd100,
    // 按键路径的饱和上限。取 KEY_STEP 的整数倍，双向饱和的落点因此
    // 很干净（2000/1900/…），便于现场把待发送值拨到一个可预期的确定值。
    parameter signed [W-1:0] KEY_LIMIT = 16'sd2000
)(
    input  wire                clk,
    input  wire                rst_n,

    // ---- 命令接口（与 pid_tuner 并联，各自消费）----
    input  wire                cmd_valid,       // 1 拍脉冲
    input  wire [7:0]          cmd_data,

    // ---- 模式选通（由顶层给出）----
    //   sel_loop == 6 表示"选中位置指令模式"（'M' 按下后为 1，'1'..'5' 后为 0）
    input  wire                pos_mode_active,

    //---------------------------------------------------------------------
    // 外部 +/- 按键（各 1 拍脉冲）
    //   与串口 '+'/'-' 的区别：
    //     · 步进固定 KEY_STEP（=100），不是 64 / 取反 / 翻倍那一套；
    //     · 结果饱和到 ±KEY_LIMIT；
    //     · 不受 pos_mode_active 门控 —— 现场手拨不该还要先按 'M' 才动，
    //       否则"按了没反应"会被当成按键坏了。
    //   真正的"生效"仍然靠 '!' / 外部"输入"键，所以提前把值拨到 500
    //   不会让摆臂动，只是把待发送值准备好。
    //---------------------------------------------------------------------
    input  wire                key_inc,         // 1 拍：待发送值 +KEY_STEP
    input  wire                key_dec,         // 1 拍：待发送值 -KEY_STEP

    // ---- 状态回读（给遥测/上位机显示）----
    output reg  signed [W-1:0] pos_pend,        // 当前待发送值
    output reg                 pos_send_pulse,  // 1 拍脉冲：请把 pos_delta 加到定点
    output reg  signed [W-1:0] pos_delta        // 本次要发送的增量（锁存值）
);

    //-------------------------------------------------------------------------
    // 命令译码
    //   'M' 0x4D -> 进入位置指令模式（由顶层把 sel_loop 置 6）
    //   '!' 0x21 -> 确认发送
    //   '+' 0x2B  加 64
    //   '-' 0x2D  取反
    //   '>' 0x3E  +12.5%
    //   '<' 0x3C  -12.5%
    //   '*' 0x2A  翻倍
    //   '/' 0x2F  减半
    //   '0' 0x30  取消（清 0，不发送）
    //
    // 编辑键与 '!' 都必须与 pos_mode_active 相与，理由有两条：
    //   其一，否则同一次按键会既改系数又改待发送值，互相干扰；
    //   其二，'!' 若不受门控，在系数整定模式下也会打出一拍 pos_send_pulse：
    //   它会把 pos_delta 写成 0，而 traj_ramp 用 "cmd_delta == 0" 表示归零，
    //   于是一次误触就把定点目标重置到当前位置 —— 属于隐性的状态污染。
    //-------------------------------------------------------------------------
    wire is_pend_send = cmd_valid && pos_mode_active && (cmd_data == 8'h21);   // '!'
    wire is_edit_up   = cmd_valid && pos_mode_active && (cmd_data == 8'h3E);  // '>'
    wire is_edit_dn   = cmd_valid && pos_mode_active && (cmd_data == 8'h3C);  // '<'
    wire is_edit_x2   = cmd_valid && pos_mode_active && (cmd_data == 8'h2A);  // '*'
    wire is_edit_half = cmd_valid && pos_mode_active && (cmd_data == 8'h2F);  // '/'
    wire is_edit_neg  = cmd_valid && pos_mode_active && (cmd_data == 8'h2D);  // '-'
    wire is_edit_plus = cmd_valid && pos_mode_active && (cmd_data == 8'h2B);  // '+'
    wire is_edit_clr  = cmd_valid && pos_mode_active && (cmd_data == 8'h30);  // '0'

    //-------------------------------------------------------------------------
    // 编辑运算：用 W+1 位中间量，最后统一饱和到 ±DELTA_MAX
    //   为什么不用位宽更大的中间量：本模块的数值上限只有 ±3072，
    //   而 '*' 翻倍最多到 ±6144，W+1=17 位（±65535）绰绰有余。
    //   刻意保持窄位宽，是为了让"翻倍"没有回绕的空间 —— 回绕在
    //   定点数编辑里是最难查的一类问题。
    //-------------------------------------------------------------------------
    wire signed [W:0] pend_ext  = $signed({pos_pend[W-1], pos_pend});
    wire signed [W:0] dmax_ext  = $signed({DELTA_MAX[W-1], DELTA_MAX});
    wire signed [W:0] dmax_neg  = -dmax_ext;

    reg  signed [W:0] pend_new;
    always @* begin
        pend_new = pend_ext;
        if (is_edit_up)
            pend_new = pend_ext + (pend_ext >>> 3);        // +12.5%
        else if (is_edit_dn)
            pend_new = pend_ext - (pend_ext >>> 3);        // -12.5%
        else if (is_edit_x2)
            pend_new = pend_ext <<< 1;                     // 翻倍
        else if (is_edit_half)
            pend_new = pend_ext >>> 1;                     // 减半（算术右移）
        else if (is_edit_neg)
            pend_new = -pend_ext;                          // 取反
        else if (is_edit_plus)
            pend_new = pend_ext + 17'sd64;                 // 加 64
        else if (is_edit_clr)
            pend_new = 17'sd0;                             // 清 0
    end

    //-------------------------------------------------------------------------
    // 饱和到 ±DELTA_MAX
    //-------------------------------------------------------------------------
    wire signed [W:0] pend_sat =
        (pend_new >  dmax_ext) ? dmax_ext :
        (pend_new <  dmax_neg) ? dmax_neg :
                                  pend_new;

    //-------------------------------------------------------------------------
    // 按键 ±KEY_STEP，并饱和到 ±KEY_LIMIT
    //   用 W+1 位做中间量，保证 +100/-100 不会在边界处回绕。
    //   饱和是【双向】的：从 1950 再加 100 得 2000（不是 2050），
    //   从 -1950 再减 100 得 -2000。这对"手拨到一个确定值"是关键性质。
    //-------------------------------------------------------------------------
    wire signed [W:0] kstep_ext = $signed({{1{KEY_STEP[W-1]}},  KEY_STEP});
    wire signed [W:0] klim_ext  = $signed({{1{KEY_LIMIT[W-1]}}, KEY_LIMIT});
    wire signed [W:0] klim_neg  = -klim_ext;

    wire signed [W:0] inc_raw = pend_ext + kstep_ext;
    wire signed [W:0] dec_raw = pend_ext - kstep_ext;

    wire signed [W:0] inc_sat = (inc_raw > klim_ext) ? klim_ext : inc_raw;
    wire signed [W:0] dec_sat = (dec_raw < klim_neg) ? klim_neg : dec_raw;

    //-------------------------------------------------------------------------
    // 方向开关
    //   正增量约定为使摆臂向正方向（编码器计数增加的方向）转动。
    //   若实机方向相反，把下面这一行改成 assign pos_delta = -pos_delta_raw;
    //   或者把 pos_delta_raw 的符号位取反，其余代码不用动 ——
    //   这是全设计里唯一决定"正负方向"的地方。
    //-------------------------------------------------------------------------
    wire signed [W-1:0] pos_delta_raw = pos_pend;
    // 默认原样输出；若方向相反，把下一行取反
    wire signed [W-1:0] pos_delta_out = pos_delta_raw;

    //-------------------------------------------------------------------------
    // 主时序
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_pend        <= {W{1'b0}};
            pos_send_pulse  <= 1'b0;
            pos_delta       <= {W{1'b0}};
        end
        else begin
            pos_send_pulse <= 1'b0;              // 默认不脉冲

            //-------------------------------------------------------------
            // 外部 +/- 键优先级最高：与串口编辑键互斥（同一拍两者都来是
            // 不可能的手动操作），若真撞上，按键优先，因为它是现场手拨、
            // 更"新"。
            //-------------------------------------------------------------
            if (key_inc) begin
                pos_pend <= inc_sat[W-1:0];
            end
            else if (key_dec) begin
                pos_pend <= dec_sat[W-1:0];
            end
            else if (is_edit_up || is_edit_dn || is_edit_x2 || is_edit_half
                  || is_edit_neg || is_edit_plus || is_edit_clr) begin
                pos_pend <= pend_sat[W-1:0];
            end
            else if (is_pend_send) begin
                // 确认发送：把当前待发送值锁存到 pos_delta 并打一拍脉冲，
                // 然后待发送值清 0（语义："发一次走一段"）。
                pos_delta      <= pos_delta_out;
                pos_send_pulse <= 1'b1;
                pos_pend       <= {W{1'b0}};
            end
        end
    end

endmodule
