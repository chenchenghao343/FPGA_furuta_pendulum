// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: pos_pd
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 位置环 PD 运算模块（外环，30ms，输出倾角偏置）
//           1. PD 运算：偏置 = (Kp_x * 位置偏差 + Kd_x * 摆臂速度) >> Q_FMT
//           2. 偏置幅值与变化速率双重限幅，防止指令突变
//           3. 本模块只有 P + D，不含积分项，稳态位置偏差由摩擦与装配决定
// =============================================================================
//
// 外环为什么只做 P + D：
//   外环 I 的唯一职责是消除【稳态位置偏差】。没有它，静止时 err_x 不会被
//   压到 0，剩下的偏差等于摩擦/装配造成的固定偏置（1 count ≈ 0.337°），
//   而且这个偏置会随摩擦变化，不是常数。
//   换来的是完全没有积分带来的慢摆/过冲风险：外环只剩 KP_X / KD_X 两个数，
//   参数含义单一、现场好调。这是取舍而不是优劣，做长时间平衡实验时应当
//   预期"摆臂会停在一个偏离定点的地方"。
// =============================================================================

`timescale 1ns/1ns

module pos_pd #(
    parameter integer W       = 16,      // err_x / phi_bias 位宽(有符号)
    parameter integer GAIN_W  = 32,      // 增益位宽(有符号, Q12)
    parameter integer SUM_W   = 56,      // 乘积求和的中间位宽
    parameter integer Q_FMT   = 12,      // 增益定点小数位(1.0 = 4096)
    //-------------------------------------------------------------------------
    // 外环分频比：相对 ctrl_tick 的倍数。
    //   DIV = 10  -> 内环 3ms / 外环 30ms = 1:10，与江协 STM32 工程一致。
    //   DIV = 1   -> 关闭降速，"两环同频"（用于对照实验）。
    //   计数器固定 8bit，故 DIV 必须 <= 256。
    //-------------------------------------------------------------------------
    parameter integer DIV     = 10,
    parameter signed [W-1:0] BIAS_MAX  = 16'sd250,  // 倾角偏置幅值上限
    parameter signed [W-1:0] BIAS_RATE = 16'sd2     // 每个 ctrl_tick 最大变化量
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     ctrl_tick,  // 控制节拍 3ms
    input  wire                     run_en,     // 1 = 外环投入（平衡模式）
    input  wire signed [W-1:0]      err_x,      // 位置偏差（编码器计数，已含死区）
    input  wire signed [GAIN_W-1:0] kp_x,       // 外环 P（Q12，串口 '4' 可调）
    input  wire signed [GAIN_W-1:0] kd_x,       // 外环 D（Q12，串口 '5' 可调）
    output reg  signed [W-1:0]      phi_bias    // 内环目标角度偏置（ADC计数）
);

    //-------------------------------------------------------------------------
    // ① 外环节拍分频：每 DIV 个 ctrl_tick 产生一个 pos_tick
    //   pos_tick 必须与 ctrl_tick 相与，这个写法不能省：
    //   div_cnt 只在 ctrl_tick 那一拍自增，所以 div_cnt == DIV-1 这个条件会
    //   持续整整一个控制周期（3ms = 20 个系统时钟），是一个"电平"而不是
    //   "脉冲"。而下面 err_x_prev / d_armed / bias_target_r 三个寄存器只在
    //   pos_tick 为 1 时更新 —— 若不与 ctrl_tick 相与，它们会在那 20 个时钟里
    //   每个时钟都更新一次：第 1 个时钟 err_x_prev <= err_x 并同时得到含 D 项
    //   的正确值，第 2 个时钟 err_x_prev 已经等于 err_x，差分归零，只剩 P 项，
    //   之后每个时钟重复第 2 步。结果是 D 项算出来立刻被覆盖，整个微分通道
    //   等于没接。（ctrl_tick 本身是 1 拍脉冲，相与后仍是 1 拍。）
    //-------------------------------------------------------------------------
    reg [7:0] div_cnt;
    wire      pos_tick = ctrl_tick && (div_cnt == (DIV - 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            div_cnt <= 8'd0;
        else if (ctrl_tick)
            div_cnt <= pos_tick ? 8'd0 : (div_cnt + 8'd1);
    end

    //-------------------------------------------------------------------------
    // ② 外环一阶差分（带"防投入冲击"武装标志）
    //-------------------------------------------------------------------------
    reg  signed [W-1:0] err_x_prev;
    reg                 d_armed;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            err_x_prev <= {W{1'b0}};
            d_armed    <= 1'b0;
        end
        else if (!run_en) begin
            // 外环未投入：历史清零，保证重新投入时第一拍微分为 0
            err_x_prev <= {W{1'b0}};
            d_armed    <= 1'b0;
        end
        else if (pos_tick) begin
            err_x_prev <= err_x;
            d_armed    <= 1'b1;
        end
    end

    // 17bit 有符号：两个 16bit 之差最大 ±65535，16bit 装不下
    //   未武装时输出 0，避免用陈旧的 err_x_prev 算出一个假跳变
    wire signed [W:0] derr_x = d_armed ? (err_x - err_x_prev) : {(W+1){1'b0}};

    //-------------------------------------------------------------------------
    // ③ PD 求和 + 定点反标定 + 饱和
    //    求和只有 P 与 D 两项。
    //    判正负一律用符号位，不要写 "err_x > 0" ——
    //    有符号与无符号混合比较时 Verilog 会把整个比较按无符号处理，
    //    负数会被当成很大的正数，必须用符号位比较。
    //-------------------------------------------------------------------------
    wire signed [SUM_W-1:0] p_term = kp_x * err_x;
    wire signed [SUM_W-1:0] d_term = kd_x * derr_x;
    wire signed [SUM_W-1:0] pd_sum = p_term + d_term;

    // 饱和阈值 = BIAS_MAX x 2^Q_FMT
    localparam signed [SUM_W-1:0] SUM_MAX = BIAS_MAX <<< Q_FMT;

    wire signed [W-1:0] bias_target =
        (pd_sum >  SUM_MAX) ?  BIAS_MAX :
        (pd_sum < -SUM_MAX) ? -BIAS_MAX :
                              pd_sum[Q_FMT+W-1 : Q_FMT];

    //-------------------------------------------------------------------------
    // ④ 外环节拍寄存（30ms 更新一次）
    //-------------------------------------------------------------------------
    reg signed [W-1:0] bias_target_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bias_target_r <= {W{1'b0}};
        else if (!run_en)
            bias_target_r <= {W{1'b0}};
        else if (pos_tick)
            bias_target_r <= bias_target;
    end

    //-------------------------------------------------------------------------
    // ⑤ 控制节拍速率限幅（3ms 更新一次）
    //    |Δ| <= BIAS_RATE 时直接跟随；否则每拍走一步。
    //    0 -> BIAS_MAX 需要 BIAS_MAX/BIAS_RATE 个 ctrl_tick。
    //-------------------------------------------------------------------------
    wire signed [W:0] bias_diff = bias_target_r - phi_bias;   // 17bit 防溢出

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            phi_bias <= {W{1'b0}};
        else if (!run_en)
            phi_bias <= {W{1'b0}};
        else if (ctrl_tick) begin
            if (bias_diff > BIAS_RATE)
                phi_bias <= phi_bias + BIAS_RATE;
            else if (bias_diff < -BIAS_RATE)
                phi_bias <= phi_bias - BIAS_RATE;
            else
                phi_bias <= bias_target_r;
        end
    end

endmodule
