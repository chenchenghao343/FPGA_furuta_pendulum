// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: pid_pipeline
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 五级流水线定点 PID 运算模块（倾角环核心）
//           1. P / I / D 与位置环两项拆成 5 级流水，每级一拍，吞吐率 1 结果/拍
//           2. 全定点实现（Q10），乘加用 DSP 硬核，单拍完成 32x16 有符号乘
//           3. 积分项带条件冻结抗饱和；输出限幅，留出电机驱动余量
// =============================================================================

`timescale 1ns/1ns

module pid_pipeline #(
    // ---- 定点与位宽参数 ----
    parameter integer Q_FMT  = 10,   // Q10 定点：1.0 == 1024
    parameter integer ERR_W  = 16,   // 误差/增量位宽(有符号)
parameter integer GAIN_W = 32,   // 系数位宽(有符号, Q10)，32bit 才有足够的增益分辨率
    parameter integer PROD_W = 48,   // 单项乘积位宽 = GAIN_W + ERR_W
    parameter integer ACC_W  = 56,   // 积分累加器位宽(有符号)
    parameter integer SUM_W  = 56,   // 级间求和位宽(5 项 48bit 相加 + 余量)
    parameter integer OUT_W  = 16,   // 控制量输出位宽(有符号)

    // ---- 饱和/限幅参数 ----
    parameter signed [ACC_W-1:0] I_MAX   = 56'sd4_194_304, // 积分限幅 ±4096(输出单位)
    parameter signed [OUT_W-1:0] OUT_MAX = 16'sd4095,      // 输出饱和上限 = 4095/5000 ≈ 82% 占空比(100% = motor_drv 的 DUTY_MAX)

    // ---- 级数参数化（为后续插入滤波级/前馈级预留）----
    parameter integer PIPE_STAGES = 5
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    ctrl_tick,   // 控制节拍：积分累加使能
    input  wire                    run_en,      // 总使能，0=复位积分并输出 0

    // ---- 反馈量 ----
    input  wire signed [ERR_W-1:0] err_th,      // 倾角偏差 e_th
    input  wire signed [ERR_W-1:0] dth,         // 倾角增量 Δθ
    input  wire signed [ERR_W-1:0] err_x,       // 位置偏差 e_x
    input  wire signed [ERR_W-1:0] dx,          // 位置增量 Δx

    // ---- 系数（Q10 有符号）----
    input  wire signed [GAIN_W-1:0] kp_th,
    input  wire signed [GAIN_W-1:0] ki_th,
    input  wire signed [GAIN_W-1:0] kd_th,
    input  wire signed [GAIN_W-1:0] kp_x,
    input  wire signed [GAIN_W-1:0] kd_x,

    // ---- 输出 ----
    output reg  signed [OUT_W-1:0] u_ctrl,      // 控制量
    output wire                    u_vld,       // 与 u_ctrl 同拍
    output reg                     s5_sat,      // 输出饱和标志

    // ---- 流水线抽头（仿真 / 在线逻辑分析仪）----
    output reg  signed [PROD_W-1:0] s2_p_term,
    output reg  signed [PROD_W-1:0] s2_px_term,
    output reg  signed [PROD_W-1:0] s3_d_term,
    output reg  signed [PROD_W-1:0] s3_dx_term,
    output reg  signed [ACC_W-1:0]  s3_i_acc,
    output reg  signed [SUM_W-1:0]  s4_sum
);

    // 正向饱和阈值（符号扩展版，用于有符号比较）
    localparam signed [SUM_W-1:0] OUT_MAX_EXT = OUT_MAX;

    //=========================================================================
    // S1 锁存级
    //=========================================================================
    reg signed [ERR_W-1:0]  s1_err_th, s1_dth, s1_err_x, s1_dx;
    reg signed [GAIN_W-1:0] s1_kp_th, s1_ki_th, s1_kd_th, s1_kp_x, s1_kd_x;
    reg                     s1_vld;

    //=========================================================================
    // S2 乘法级 A：P 项（两个独立 DSP）
    //=========================================================================
    reg signed [GAIN_W-1:0] s2_ki_th, s2_kd_th, s2_kd_x;
    reg signed [ERR_W-1:0]  s2_err_th, s2_dth, s2_dx;
    reg                     s2_vld;

    //=========================================================================
    // S3 乘法级 B + 积分累加级 + P 项打拍对齐
    //=========================================================================
    reg signed [PROD_W-1:0] s3_p_term, s3_px_term;   // 由 S2 延后一级得到
    reg                     s3_vld;

    //=========================================================================
    // S4 / S5
    //=========================================================================
    reg signed [SUM_W-1:0]  s5_u_full;
    reg                     s4_vld;      // S4 的有效标志，S5 的 valid 必须由它推进
    reg                     s5_vld;

    reg signed [1:0]        s5_sat_dir;   // 饱和方向：+1 / -1 / 0

    //-------------------------------------------------------------------------
    // 积分累加器下一拍值的组合计算
    //-------------------------------------------------------------------------
    wire signed [PROD_W-1:0] i_inc      = s2_ki_th * s2_err_th;
    wire signed [ACC_W-1:0]  i_acc_pre  = s3_i_acc + i_inc;
    wire signed [ACC_W-1:0]  i_acc_clmp = (i_acc_pre >  I_MAX) ?  I_MAX :
                                          (i_acc_pre < -I_MAX) ? -I_MAX : i_acc_pre;

    // 条件积分：输出已饱和，且积分增量方向与饱和方向一致 → 冻结
    wire i_freeze = ( (s5_sat_dir == 2'sd1)  && (i_inc > 0) ) ||
                    ( (s5_sat_dir == -2'sd1) && (i_inc < 0) );

    //-------------------------------------------------------------------------
    // 流水线主体
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // ---- S1 ----
            s1_err_th <= {ERR_W{1'b0}};  s1_dth <= {ERR_W{1'b0}};
            s1_err_x  <= {ERR_W{1'b0}};  s1_dx  <= {ERR_W{1'b0}};
            s1_kp_th  <= {GAIN_W{1'b0}}; s1_ki_th <= {GAIN_W{1'b0}};
            s1_kd_th  <= {GAIN_W{1'b0}}; s1_kp_x  <= {GAIN_W{1'b0}};
            s1_kd_x   <= {GAIN_W{1'b0}}; s1_vld   <= 1'b0;
            // ---- S2 ----
            s2_p_term <= {PROD_W{1'b0}}; s2_px_term <= {PROD_W{1'b0}};
            s2_ki_th  <= {GAIN_W{1'b0}}; s2_kd_th <= {GAIN_W{1'b0}};
            s2_kd_x   <= {GAIN_W{1'b0}}; s2_err_th <= {ERR_W{1'b0}};
            s2_dth    <= {ERR_W{1'b0}};  s2_dx <= {ERR_W{1'b0}};
            s2_vld    <= 1'b0;
            // ---- S3 ----
            s3_p_term <= {PROD_W{1'b0}}; s3_px_term <= {PROD_W{1'b0}};
            s3_d_term <= {PROD_W{1'b0}}; s3_dx_term <= {PROD_W{1'b0}};
            s3_i_acc  <= {ACC_W{1'b0}};  s3_vld <= 1'b0;
            // ---- S4 / S5 ----
            s4_sum    <= {SUM_W{1'b0}};
            s5_u_full <= {SUM_W{1'b0}};
            s4_vld    <= 1'b0;
            s5_vld    <= 1'b0;
        end
        else if (!run_en) begin
            //-------------------------------------------------------------
            // 停机：积分清零、输出清零、valid 清零（安全优先）
            //-------------------------------------------------------------
            s1_vld <= 1'b0; s2_vld <= 1'b0; s3_vld <= 1'b0; s5_vld <= 1'b0;
            s4_vld <= 1'b0;
            s1_err_th <= {ERR_W{1'b0}}; s1_dth <= {ERR_W{1'b0}};
            s1_err_x  <= {ERR_W{1'b0}}; s1_dx  <= {ERR_W{1'b0}};
            s1_kp_th  <= {GAIN_W{1'b0}}; s1_ki_th <= {GAIN_W{1'b0}};
            s1_kd_th  <= {GAIN_W{1'b0}}; s1_kp_x  <= {GAIN_W{1'b0}};
            s1_kd_x   <= {GAIN_W{1'b0}};
            s2_p_term <= {PROD_W{1'b0}}; s2_px_term <= {PROD_W{1'b0}};
            s2_ki_th  <= {GAIN_W{1'b0}}; s2_kd_th <= {GAIN_W{1'b0}};
            s2_kd_x   <= {GAIN_W{1'b0}}; s2_err_th <= {ERR_W{1'b0}};
            s2_dth    <= {ERR_W{1'b0}};  s2_dx <= {ERR_W{1'b0}};
            s3_p_term <= {PROD_W{1'b0}}; s3_px_term <= {PROD_W{1'b0}};
            s3_d_term <= {PROD_W{1'b0}}; s3_dx_term <= {PROD_W{1'b0}};
            s3_i_acc  <= {ACC_W{1'b0}};   // 积分清零，防止停机期间积分饱和
            s4_sum    <= {SUM_W{1'b0}};
            s5_u_full <= {SUM_W{1'b0}};
        end
        else begin
            //=============================================================
            // S1：无条件锁存 —— 数据每个时钟沿都在推进（"传送带"）
            //=============================================================
            s1_err_th <= err_th;
            s1_dth    <= dth;
            s1_err_x  <= err_x;
            s1_dx     <= dx;
            s1_kp_th  <= kp_th;
            s1_ki_th  <= ki_th;
            s1_kd_th  <= kd_th;
            s1_kp_x   <= kp_x;
            s1_kd_x   <= kd_x;
            s1_vld    <= 1'b1;

            //=============================================================
            // S2：P 项乘法（独立 DSP）+ D/I 操作数透传
            //=============================================================
            s2_p_term  <= s1_kp_th * s1_err_th;
            s2_px_term <= s1_kp_x  * s1_err_x;
            s2_ki_th   <= s1_ki_th;
            s2_kd_th   <= s1_kd_th;
            s2_kd_x    <= s1_kd_x;
            s2_err_th  <= s1_err_th;
            s2_dth     <= s1_dth;
            s2_dx      <= s1_dx;
            s2_vld     <= s1_vld;

            //=============================================================
            // S3：D 项乘法（独立 DSP） + 积分累加（独立加法器）
            //=============================================================
            s3_p_term  <= s2_p_term;
            s3_px_term <= s2_px_term;

            s3_d_term  <= s2_kd_th * s2_dth;
            s3_dx_term <= s2_kd_x  * s2_dx;

            if (ctrl_tick && !i_freeze)
                s3_i_acc <= i_acc_clmp;
            // else：条件积分冻结 或 非控制节拍 → 保持

            s3_vld <= s2_vld;

            //=============================================================
            // S4：加法器树
            //=============================================================
            s4_sum <= s3_p_term + s3_d_term + s3_px_term + s3_dx_term + s3_i_acc;
            s4_vld <= s3_vld;

            //=============================================================
            // S5：一次算术右移完成 Q10 反标定
            //=============================================================
            s5_u_full <= s4_sum >>> Q_FMT;
            s5_vld    <= s4_vld;
        end
    end

    //-------------------------------------------------------------------------
    // S5 输出组合：饱和限幅 + 饱和方向标志
    //-------------------------------------------------------------------------
    always @(*) begin
        if (s5_u_full > OUT_MAX_EXT) begin
            u_ctrl     = OUT_MAX;
            s5_sat     = 1'b1;
            s5_sat_dir = 2'sd1;
        end
        else if (s5_u_full < -OUT_MAX_EXT) begin
            u_ctrl     = -OUT_MAX;
            s5_sat     = 1'b1;
            s5_sat_dir = -2'sd1;
        end
        else begin
            u_ctrl     = s5_u_full[OUT_W-1:0];
            s5_sat     = 1'b0;
            s5_sat_dir = 2'sd0;
        end
    end

    //-------------------------------------------------------------------------
    // u_vld：与 u_ctrl 严格同拍
    //-------------------------------------------------------------------------
    assign u_vld = s5_vld;

endmodule
