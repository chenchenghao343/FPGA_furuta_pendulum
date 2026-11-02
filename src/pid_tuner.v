// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: pid_tuner
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 串口在线调参器（单字符命令）
//           1. 保存 6 个参数：kp_th / ki_th / kd_th / kp_x / kd_x / swing_duty
//              复位值来自顶层传入的 *_INIT，没有掉电保存
//           2. '1'~'6' 选通道，'>' '<' '*' '/' '-' '+' 修改选中项
//           3. 输出运行/遥测开关与若干动作脉冲（标定参考点、翻转起摆极性、
//              定点归零、位置指令模式与发送）
// =============================================================================
//
// 【命令表】
//   '1'~'6'  选通道：0=kp_th 1=ki_th 2=kd_th 3=kp_x 4=kd_x 5=swing_duty
//   '>' +12.5%   '<' -12.5%   '*' 翻倍   '/' 减半   '-' 取反   '+' 加 64
//   'R' 运行/停止      'S' 遥测开关
//   'U' 记录直立参考   'D' 记录下垂参考
//   'P' 翻转起摆能量泵极性
//   'A' 切换定点模式   '0' 定点归零
//   'M' 进入位置指令编辑（sel_loop=6）   '!' 发送待发送量
//
// 【约定】
//   · 系数只存位模式（端口无符号），PID 模块按 Q10 / Q12 解释
//   · 所有运算中间量用 36bit，最后经 f_sat() 饱和到 32bit 有符号
//   · 本模块不保存定点数值（那是 traj_ramp 的状态），只输出"意图"
// =============================================================================

module pid_tuner #(
    parameter integer Q_FMT      = 10,
    parameter [31:0]  KP_TH_INIT = 32'd0,
    parameter [31:0]  KI_TH_INIT = 32'd0,
    parameter [31:0]  KD_TH_INIT = 32'd0,
    parameter [31:0]  KP_X_INIT = 32'd0,
    parameter [31:0]  KD_X_INIT = 32'd0,
    parameter [31:0]  SWING_INIT = 32'd4684
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  cmd_data,
    input  wire        cmd_valid,
    output reg  [31:0] kp_th,
    output reg  [31:0] ki_th,
    output reg  [31:0] kd_th,
    output reg  [31:0] kp_x,
    output reg  [31:0] kd_x,
    output reg  [31:0] swing_duty,
    output reg  [2:0]  sel_loop,
    output reg         run_en,
    output reg         telemetry_en,
    output reg         up_ref_capture,
    output reg         down_ref_capture,
    output reg         pump_pol_toggle,
    output reg         pos_mode,
    output reg         zero_req,
    output reg  [15:0] last_cmd
);
    //-------------------------------------------------------------------------
    // 饱和函数：36bit 有符号中间量夹到 32bit 有符号满量程
    //   入参取 36bit 是为了容纳 '<<<1' 翻倍后的中间量（32bit 系数翻倍后 33bit），
    //   保证判饱和之前不会先回绕。
    //-------------------------------------------------------------------------
    function signed [31:0] f_sat;
        input signed [35:0] v;
        begin
            if (v > 36'sd2147483647)
                f_sat = 32'sd2147483647;
            else if (v < -36'sd2147483648)
                f_sat = 32'sh80000000;
            else
                f_sat = v[31:0];
        end
    endfunction
    //-------------------------------------------------------------------------
    // 读出当前选中的系数：32bit 有符号 -> 36bit 有符号（符号位正确复制）
    //-------------------------------------------------------------------------
    function signed [35:0] f_get;
        input [2:0] a;
        begin
            case (a)
                3'd0:    f_get = $signed(kp_th);
                3'd1:    f_get = $signed(ki_th);
                3'd2:    f_get = $signed(kd_th);
                3'd3:    f_get = $signed(kp_x);
                3'd4:    f_get = $signed(kd_x);
                3'd5:    f_get = $signed(swing_duty);
                default: f_get = $signed(kd_x);
            endcase
        end
    endfunction
    //-------------------------------------------------------------------------
    // 命令译码（全部与 cmd_valid 相与，保证只在命令到达那一拍动作）
    //-------------------------------------------------------------------------
    wire is_sel  = cmd_valid && (cmd_data >= 8'h31) && (cmd_data <= 8'h36);// '1'..'6' 选通道
    wire is_up   = cmd_valid && (cmd_data == 8'h3E);                    // '>' +12.5%
    wire is_dn   = cmd_valid && (cmd_data == 8'h3C);                    // '<' -12.5%
    wire is_x2   = cmd_valid && (cmd_data == 8'h2A);                    // '*' 翻倍
    wire is_half = cmd_valid && (cmd_data == 8'h2F);                    // '/' 减半
    wire is_run  = cmd_valid && (cmd_data == 8'h52);                    // 'R' 运行/停止
    wire is_tel  = cmd_valid && (cmd_data == 8'h53);                    // 'S' 遥测开关
    wire is_up_r = cmd_valid && (cmd_data == 8'h55);                    // 'U' 记录直立参考
    wire is_dn_r = cmd_valid && (cmd_data == 8'h44);                    // 'D' 记录下垂参考
    wire is_neg  = cmd_valid && (cmd_data == 8'h2D);                    // '-' 取反
    wire is_plus = cmd_valid && (cmd_data == 8'h2B);                    // '+' 加 64（Q10 的 0.0625）
    wire is_pol  = cmd_valid && (cmd_data == 8'h50);                    // 'P' 翻转起摆极性
    wire is_pmode = cmd_valid && (cmd_data == 8'h41);                   // 'A' 切换定点模式
    wire is_posmode = cmd_valid && (cmd_data == 8'h4D);                 // 'M' 进入位置指令编辑
    wire pos_cmd_mode = (sel_loop == 3'd6);
    wire is_pend_send = cmd_valid && (cmd_data == 8'h21);               // '!' 发送待发送量
    wire is_zero  = cmd_valid && (cmd_data == 8'h30);                   // '0' 定点归零
    wire       coef_op = is_up | is_dn | is_x2 | is_half | is_neg | is_plus;
    wire [2:0] tgt     = is_sel ? (cmd_data[2:0] - 3'd1) : sel_loop;   // '1'..'6' -> 0..5
    //-------------------------------------------------------------------------
    // 系数运算：全部有符号 Q10 运算 + 饱和
    //   · '>>>3' / '+/-' 在 36bit 上做，绝不丢符号位
    //   · '<<<1' 翻倍后最多 33bit，36bit 可容纳
    //-------------------------------------------------------------------------
    reg signed [35:0] g_cur;
    reg signed [35:0] g_new;
    always @* begin
        g_cur = f_get(tgt);
        g_new = g_cur;
        // 以下 6 个运算全部经 f_sat() 饱和，36bit 中间量防溢出
        if (is_up) begin
            g_new = f_sat(g_cur + (g_cur >>> 3));
        end else if (is_dn) begin
            g_new = f_sat(g_cur - (g_cur >>> 3));
        end else if (is_x2) begin
            g_new = f_sat(g_cur <<< 1);
        end else if (is_half) begin
            g_new = f_sat(g_cur >>> 1);
        end else if (is_neg) begin
            g_new = f_sat(-g_cur);
        end else if (is_plus) begin
            g_new = f_sat(g_cur + 20'sd64);
        end
    end
    //-------------------------------------------------------------------------
    // 主时序：系数写回 + 动作脉冲（脉冲默认 0，只有命令到达那一拍抬高）
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kp_th           <= KP_TH_INIT;
            ki_th           <= KI_TH_INIT;
            kd_th           <= KD_TH_INIT;
            kp_x            <= KP_X_INIT;
            kd_x            <= KD_X_INIT;
            swing_duty      <= SWING_INIT;
            sel_loop        <= 3'd0;
            run_en          <= 1'b0;
            telemetry_en    <= 1'b1;      // 复位遥测常开：上电即上报，波形上位机直接可用（'S' 仍可关）
            up_ref_capture  <= 1'b0;
            down_ref_capture <= 1'b0;
            pump_pol_toggle <= 1'b0;
            pos_mode        <= 1'b0;
            zero_req        <= 1'b0;
            last_cmd        <= 16'd0;
        end else begin
            up_ref_capture   <= 1'b0;
            down_ref_capture <= 1'b0;
            pump_pol_toggle  <= 1'b0;
            zero_req         <= 1'b0;
            if (cmd_valid) begin
                last_cmd <= {8'd0, cmd_data};
                if (is_sel)
                    sel_loop <= cmd_data[2:0] - 3'd1;
                if (is_posmode)
                    sel_loop <= 3'd6;
                //--- 系数写回：位置指令编辑模式下不写系数，避免同一按键既改
                //    待发送值又改系数（面板上看不出来）---
                if (coef_op && !pos_cmd_mode) begin
                    case (tgt)
                        // 取低 32 位：f_sat() 已把结果饱和到 32bit 有符号范围
                        3'd0:    kp_th <= g_new[31:0];        // '1' 倾角 P
                        3'd1:    ki_th <= g_new[31:0];        // '2' 倾角 I
                        3'd2:    kd_th <= g_new[31:0];        // '3' 倾角 D
                        3'd3:    kp_x  <= g_new[31:0];        // '4' 位置 P
                        3'd4:    kd_x  <= g_new[31:0];        // '5' 位置 D
                        3'd5:    swing_duty <= g_new[31:0];   // '6' 起摆幅值
                        default: kd_x  <= g_new[31:0];
                    endcase
                end
                if (is_run)
                    run_en <= ~run_en;
                if (is_tel)
                    telemetry_en <= ~telemetry_en;
                if (is_up_r)
                    up_ref_capture <= 1'b1;
                if (is_dn_r)
                    down_ref_capture <= 1'b1;
                if (is_pol)
                    pump_pol_toggle <= 1'b1;
                if (is_pmode)
                    pos_mode <= ~pos_mode;
                if (is_zero)
                    pos_mode <= 1'b1;
                if (is_posmode | is_pend_send)
                    pos_mode <= 1'b1;
                //--- 动作脉冲：'0' 归零（1 拍）---
                if (is_zero)
                    zero_req <= 1'b1;
            end
        end
    end
endmodule