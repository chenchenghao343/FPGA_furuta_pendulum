// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: swing_pump
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 起摆能量泵模块
//           1. 按摆杆角速度方向注入同向能量，逐步增大摆幅
//           2. 角速度过低时改用固定幅值踢摆，突破静摩擦
//           3. 达到捕获条件后交棒给平衡环
// =============================================================================

`timescale 1ns/1ns

module swing_pump #(
    parameter integer W = 16,

    // ---- 起摆强度与判据 ----
    parameter signed [W-1:0] CAPTURE_BAND  = 16'sd130,   // 捕获带：角度偏差阈值
    parameter signed [W-1:0] CAPTURE_RATE  = 16'sd55,    // 捕获带：角速度阈值
    parameter signed [W-1:0] KICK_MIN_DTH  = 16'sd1,     // 角速度死区，低于此值改用踢摆
    parameter signed [W-1:0] KICK_DUTY     = 16'sd1400,  // 踢摆幅值
    parameter integer        KICK_DIR      = -1,          // 踢摆方向 (+1/-1)

    // ---- 极性 ----
    parameter integer PUMP_POLARITY = 1                  // +1 / -1，按电机实际转向取
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,       // 控制节拍
    input  wire               run_en,     // 1 = 起摆使能
    input  wire signed [W-1:0] phi,       // 相对直立的偏差角 φ = θ - θ_up_ref
    input  wire signed [W-1:0] dtheta,    // 角速度 θ̇
    //-------------------------------------------------------------------------
    // 起摆幅值在运行期由顶层给出，便于按摆幅大小改变泵浦强度
    //-------------------------------------------------------------------------
    input  wire signed [W-1:0] swing_duty,// 能量泵驱动幅值（由 pid_tuner 的 swing_duty 寄存器给出，可串口在线调）
    output reg  signed [W-1:0] u_swing,   // 起摆控制量
    output reg                captured    // 1 = 已进入捕获带，可切平衡环
);

    //-------------------------------------------------------------------------
    // |φ| ：取绝对值（W 位有符号 → 无符号幅值）
    //-------------------------------------------------------------------------
    wire signed [W-1:0] phi_abs = (phi >= 0) ? phi : -phi;

    //-------------------------------------------------------------------------
    // cos θ 分段线性近似： cos_approx = 1024 - |φ|，饱和到 ±1024
    //-------------------------------------------------------------------------
    reg signed [12:0] cos_approx;
    //-------------------------------------------------------------------------
    // 有符号运算域：phi_abs 先零扩展成 17bit 有符号，减法才按有符号解释，
    // 否则 1024 - |φ| 会被按无符号算成大正数
    //-------------------------------------------------------------------------
    wire signed [16:0] phi_abs_s = $signed({1'b0, phi_abs});   // 17bit 有符号
    wire signed [16:0] cos_raw   = 17'sd1024 - phi_abs_s;      // 有符号减法

    always @(*) begin
        if (phi_abs >= 16'sd2048)
            cos_approx = -13'sd1024;                    // φ ≥ 180°，cos = -1
        else if (cos_raw > 17'sd1024)
            cos_approx =  13'sd1024;                    // φ = 0°，cos = +1
        else if (cos_raw < -17'sd1024)
            cos_approx = -13'sd1024;
        else
            cos_approx = cos_raw[12:0];                 // 截位后仍在 ±1024 内
    end

    // θ̇ 与 cos θ 同号判断：只用符号位比较，避免负数被当成无符号大数；
    // 同号说明摆动方向与重力分量同向，驱动取反才能沿摆动方向注入能量
    wire pump_pos = ((dtheta >= 0) == (cos_approx >= 0));

    // 角速度死区判断
    wire signed [W-1:0] dth_abs = (dtheta >= 0) ? dtheta : -dtheta;
    wire dth_in_deadzone = (dth_abs < KICK_MIN_DTH);

    //-------------------------------------------------------------------------
    // 捕获判据：角度与角速度同时落在带内
    //-------------------------------------------------------------------------
    wire cap_now = (phi_abs < CAPTURE_BAND) && (dth_abs < CAPTURE_RATE);

    //-------------------------------------------------------------------------
    // 输出
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u_swing  <= {W{1'b0}};
            captured <= 1'b0;
        end
        else if (!run_en) begin
            u_swing  <= {W{1'b0}};
            captured <= 1'b0;
        end
        else if (tick) begin
            captured <= cap_now;

            if (cap_now) begin
                // 已进入捕获带：停止泵浦，交给平衡环
                u_swing <= {W{1'b0}};
            end
            else if (dth_in_deadzone) begin
                // ---- 踢摆：打破静止对称性 ----
                if (KICK_DIR >= 0)
                    u_swing <= (PUMP_POLARITY >= 0) ? KICK_DUTY : -KICK_DUTY;
                else
                    u_swing <= (PUMP_POLARITY >= 0) ? -KICK_DUTY : KICK_DUTY;
            end
            else begin
                // ---- 能量泵：a 与 sign(θ̇·cos θ) 反向 ----
                if (pump_pos)
                    u_swing <= (PUMP_POLARITY >= 0) ? -swing_duty : swing_duty;
                else
                    u_swing <= (PUMP_POLARITY >= 0) ? swing_duty : -swing_duty;
            end
        end
        // else：非控制节拍保持
    end

endmodule
