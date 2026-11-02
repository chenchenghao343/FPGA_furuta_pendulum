// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: ang_deriv
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 摆杆角速度 / 摆臂速度数字微分模块
//           1. 在控制节拍上对倾角与位置做一阶差分
//           2. N 点滑动平均抑制微分噪声，除法用算术右移实现，不消耗除法器
//           3. 高分辨率输出恒等于 2^6 倍均值，供高位宽 PID 使用
//           4. run_en=0 时清空历史，重新投入闭环首拍微分为 0
// =============================================================================

`timescale 1ns/1ns

module ang_deriv #(
    parameter integer W          = 16,   // 数据位宽(有符号)
    parameter integer FILT_TAPS  = 4,    // 滑动平均点数，必须为 2 的幂；1=不滤波
    parameter integer FILT_SHIFT = 2,    // log2(FILT_TAPS) = log2(4)
    parameter integer SUM_W      = 20,   // 滑动平均求和位宽(有符号)
    parameter integer HI_SHIFT   = 6     // 高分辨率差分的小数位数(需 >= FILT_SHIFT)
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,        // 控制节拍（angle_vld）
    input  wire               run_en,      // 0 = 清零历史，避免重新投入时出现假微分
    input  wire signed [W-1:0] theta_in,   // 当前倾角（已减参考，可正可负）
    input  wire signed [W-1:0] x_in,       // 当前摆臂位置（相对中点）
    output reg  signed [W-1:0] dtheta,     // 倾角一阶差分（经 FILT_TAPS 点滑动平均）
    output reg  signed [W-1:0] dx,         // 位置一阶差分（经 FILT_TAPS 点滑动平均）
    // 高分辨率版本（= 均值 * 2^HI_SHIFT），仅供 pid_pipeline 使用
    output reg  signed [W-1:0] dtheta_hi,
    output reg  signed [W-1:0] dx_hi
);

    //-------------------------------------------------------------------------
    // 历史值寄存器
    //-------------------------------------------------------------------------
    reg signed [W-1:0] theta_prev;
    reg signed [W-1:0] x_prev;

    //-------------------------------------------------------------------------
    // 滑动平均移位链（深度 FILT_TAPS）
    //-------------------------------------------------------------------------
    reg signed [W-1:0] dth_hist [0:FILT_TAPS-1];
    reg signed [W-1:0] dx_hist  [0:FILT_TAPS-1];

    wire signed [W-1:0] dth_raw = theta_in - theta_prev;
    wire signed [W-1:0] dx_raw  = x_in     - x_prev;

    integer i;

    //-------------------------------------------------------------------------
    // 求和（组合）。FILT_TAPS=1 时和就是 dth_raw 本身。
    //-------------------------------------------------------------------------
    reg signed [SUM_W-1:0] dth_sum;
    reg signed [SUM_W-1:0] dx_sum;

    always @(*) begin
        dth_sum = {SUM_W{1'b0}};
        dx_sum  = {SUM_W{1'b0}};
        for (i = 0; i < FILT_TAPS; i = i + 1) begin
            // 符号扩展后累加，避免不同位宽相加被截断
            dth_sum = dth_sum + {{(SUM_W-W){dth_hist[i][W-1]}}, dth_hist[i]};
            dx_sum  = dx_sum  + {{(SUM_W-W){dx_hist[i][W-1]}},  dx_hist[i]};
        end
    end

    //-------------------------------------------------------------------------
    // 移位后的新窗口和（组合）：新和 = 旧和 − 最旧样本 + 新样本
    //-------------------------------------------------------------------------
    wire signed [SUM_W-1:0] dth_new_sum = dth_sum
        - {{(SUM_W-W){dth_hist[FILT_TAPS-1][W-1]}}, dth_hist[FILT_TAPS-1]}
        + {{(SUM_W-W){dth_raw[W-1]}}, dth_raw};
    wire signed [SUM_W-1:0] dx_new_sum  = dx_sum
        - {{(SUM_W-W){dx_hist[FILT_TAPS-1][W-1]}}, dx_hist[FILT_TAPS-1]}
        + {{(SUM_W-W){dx_raw[W-1]}}, dx_raw};

    //-------------------------------------------------------------------------
    // 高分辨率差分（组合）：把窗口和左移 (HI_SHIFT-FILT_SHIFT) 位，
    //-------------------------------------------------------------------------
    localparam integer HI_LSH = (HI_SHIFT > FILT_SHIFT) ? (HI_SHIFT - FILT_SHIFT) : 0;
    localparam integer EXT_W  = SUM_W + HI_LSH;

    wire signed [EXT_W-1:0] dth_hi_ext = dth_new_sum <<< HI_LSH;
    wire signed [EXT_W-1:0] dx_hi_ext  = dx_new_sum  <<< HI_LSH;

    function signed [W-1:0] fsat;
        input signed [EXT_W-1:0] v;
        begin
            if (v > 32767)        fsat = 16'sd32767;
            else if (v < -32768)  fsat = -16'sd32768;
            else                  fsat = v[W-1:0];
        end
    endfunction

    //-------------------------------------------------------------------------
    // 节拍更新
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            theta_prev <= {W{1'b0}};
            x_prev     <= {W{1'b0}};
            dtheta     <= {W{1'b0}};
            dx         <= {W{1'b0}};
            dtheta_hi  <= {W{1'b0}};
            dx_hi      <= {W{1'b0}};
            for (i = 0; i < FILT_TAPS; i = i + 1) begin
                dth_hist[i] <= {W{1'b0}};
                dx_hist[i]  <= {W{1'b0}};
            end
        end
        else if (!run_en) begin
            // 停机：历史全清，保证重新投入时第一拍微分为 0，不会产生冲击
            theta_prev <= {W{1'b0}};
            x_prev     <= {W{1'b0}};
            dtheta     <= {W{1'b0}};
            dx         <= {W{1'b0}};
            dtheta_hi  <= {W{1'b0}};
            dx_hi      <= {W{1'b0}};
            for (i = 0; i < FILT_TAPS; i = i + 1) begin
                dth_hist[i] <= {W{1'b0}};
                dx_hist[i]  <= {W{1'b0}};
            end
        end
        else if (tick) begin
            theta_prev <= theta_in;
            x_prev     <= x_in;

            // 移位链推进：整体右移一格，新值进 [0]
            for (i = FILT_TAPS - 1; i > 0; i = i - 1) begin
                dth_hist[i] <= dth_hist[i-1];
                dx_hist[i]  <= dx_hist[i-1];
            end
            dth_hist[0] <= dth_raw;
            dx_hist[0]  <= dx_raw;

            // 窗口和为 20 位，>>>FILT_SHIFT 后取低 16 位即输出值：
            // 用显式位选代替「20bit 表达式赋给 16bit」，高位本就无用。
            // 前提：FILT_SHIFT <= SUM_W-16（本例 2/3，恒成立）。
            dtheta <= dth_new_sum[FILT_SHIFT +: 16];
            dx     <= dx_new_sum [FILT_SHIFT +: 16];
            // 高分辨率版本（供 PID，分辨率提高 2^HI_SHIFT 倍）
            dtheta_hi <= fsat(dth_hi_ext);
            dx_hi     <= fsat(dx_hi_ext);
        end
        // else：非控制节拍，保持（控制环 333.3Hz，不必每拍更新）
    end

endmodule
