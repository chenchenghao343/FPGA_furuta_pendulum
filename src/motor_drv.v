// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: motor_drv
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: TB6612FNG 直流电机驱动接口模块
//           1. 把有符号控制量分解为方向（IN1/IN2）与幅值（PWM 占空比）
//           2. PWM 载波 10kHz，DUTY_MAX 对应满占空比
//           3. 控制量绝对值小于 DEADBAND 时占空比给 0，抑制零速嗡鸣
// =============================================================================

`timescale 1ns / 1ps

module motor_drv #(
    parameter PWM_PERIOD = 13'd5000,      // 50MHz/5000 = 10kHz
    parameter DUTY_MAX   = 13'd5000,      // 满占空比
    parameter DEADBAND   = 16'd20         // 控制量绝对值小于此值视为 0，防止零速抖动
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] u_ctrl,     // 有符号控制量，正=正转(CW), 负=反转(CCW)
    output wire               pwm,        // -> PWMA (Y18)
    output reg                in1,        // -> AN1  (Y17)
    output reg                in2         // -> AN2  (Y19)
);

    // -------------------------------------------------------------------------
    // PWM 载波计数器：0 ~ PWM_PERIOD-1 循环，周期 PWM_PERIOD 个时钟
    // -------------------------------------------------------------------------
    reg [12:0] pwm_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_cnt <= 13'd0;
        end else if (pwm_cnt >= PWM_PERIOD - 1'b1) begin
            pwm_cnt <= 13'd0;
        end else begin
            pwm_cnt <= pwm_cnt + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // 控制量处理：取绝对值 -> 限幅到 [0, DUTY_MAX]
    // -------------------------------------------------------------------------
    wire signed [16:0] u_ext;
    // 取绝对值要用 17bit：-32768 取反后是 +32768，16bit 装不下
    assign u_ext = {{1'b0}, u_ctrl};   // 符号扩展到 17bit

    wire signed [16:0] u_abs;
    assign u_abs = (u_ext < 0) ? (-u_ext) : u_ext;

    // 限幅到 DUTY_MAX（DUTY_MAX 为 13bit，扩展比较）
    reg [12:0] cmp;   // 实际占空比比较值，13bit，范围 0 ~ DUTY_MAX
    wire [16:0] u_abs_lim;
    assign u_abs_lim = (u_abs > DUTY_MAX) ? {4'b0, DUTY_MAX} : u_abs[16:0];

    // -------------------------------------------------------------------------
    // 死区判断与方向生成（时序逻辑，与 pwm 同拍）
    //   比较前把 DEADBAND 转成有符号数：若与无符号常量直接比较，负向控制量会被
    //   当成接近 65535 的大正数，反转分支与死区都不会被命中
    // -------------------------------------------------------------------------
    reg pwm_run;   // 是否允许输出 PWM

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in1     <= 1'b0;
            in2     <= 1'b0;
            pwm_run <= 1'b0;
            cmp     <= 13'd0;
        end else begin
            if (u_ctrl > $signed(DEADBAND[15:0])) begin
                // 正转：IN1=1, IN2=0
                in1     <= 1'b1;
                in2     <= 1'b0;
                pwm_run <= 1'b1;
                cmp     <= u_abs_lim[12:0];
            end else if (u_ctrl < -$signed(DEADBAND[15:0])) begin
                // 反转：IN1=0, IN2=1
                in1     <= 1'b0;
                in2     <= 1'b1;
                pwm_run <= 1'b1;
                cmp     <= u_abs_lim[12:0];
            end else begin
                // 死区：停机（IN1=IN2=0），PWM 强制为 0
                in1     <= 1'b0;
                in2     <= 1'b0;
                pwm_run <= 1'b0;
                cmp     <= 13'd0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // PWM 输出：计数小于比较值时为高，比较值为 0 时整周期全低
    // -------------------------------------------------------------------------
    assign pwm = (rst_n && pwm_run && (pwm_cnt < cmp)) ? 1'b1 : 1'b0;

endmodule
