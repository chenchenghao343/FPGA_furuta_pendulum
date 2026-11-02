// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: pos_ctrl
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 位置环误差计算模块（外环，30ms）
//           1. 计算位置偏差 = 目标 - 实际位置
//           2. 死区处理：偏差小于 X_DEADBAND 时置 0，抑制中心附近的编码器抖动
//           3. 死区可由 dead_en 旁路（正弦等连续轨迹模式下必须旁路）
// =============================================================================

`timescale 1ns/1ns

module pos_ctrl #(
    parameter integer W = 16,
    //-------------------------------------------------------------------------
    // 位置死区取 3 个编码器计数：中心附近的正反抖动约 ±2 count，
    // 取 3 既能把抖动吃掉，又不会吃掉有效的小偏差
    //-------------------------------------------------------------------------
    parameter signed [W-1:0] X_DEADBAND = 16'sd3   // 位置死区（编码器计数）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,      // 控制节拍
    input  wire               run_en,    // 1 = 位置环投入（平衡/定点模式）
    input  wire signed [W-1:0] x_target, // 定点目标（已限速，来自 traj_ramp）
    input  wire signed [W-1:0] pos_rel,  // 摆臂相对中点的位置
    //-------------------------------------------------------------------------
    // 死区使能：0 = 旁路死区，1 = 正常启用
    //-------------------------------------------------------------------------
    input  wire               dead_en,
    output reg  signed [W-1:0] err_x     // 位置偏差 e_x = x_target - pos_rel（含死区）
);

    //-------------------------------------------------------------------------
    // 偏差形成 + 死区
    //-------------------------------------------------------------------------
    // 两侧都符号扩展到 W+1 位再相减，差值的范围留足一位，不会溢出
    wire signed [W:0] diff_ext =
        $signed({x_target[W-1], x_target}) - $signed({pos_rel[W-1], pos_rel});

    // dead_en=0 时把死区清零（正弦模式旁路，避免过零处平顶）
    wire signed [W:0] dead_ext = dead_en
                               ? $signed({X_DEADBAND[W-1], X_DEADBAND})
                               : {(W+1){1'b0}};

    // 死区内输出 0，带外则把偏差朝零方向收窄 dead_ext，形成连续过渡
    wire signed [W:0] err_ext =
        (diff_ext >  dead_ext) ? (diff_ext - dead_ext) :
        (diff_ext < -dead_ext) ? (diff_ext + dead_ext) :
                                  {(W+1){1'b0}};

    // 先饱和后取位片：直接截位会静默回绕（-40000 -> +25536），大偏差时符号会翻
    wire signed [W-1:0] err_sat =
        (err_ext >  17'sd32767) ?  16'sd32767 :
        (err_ext < -17'sd32768) ? -16'sd32768 :
                                   err_ext[W-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            err_x <= {W{1'b0}};
        else if (!tick) begin
            // 非控制节拍保持
        end
        else if (!run_en)
            // 位置环未投入：偏差恒为 0。
            err_x <= {W{1'b0}};
        else
            err_x <= err_sat;
    end

endmodule
