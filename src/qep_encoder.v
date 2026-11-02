// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: qep_encoder
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 增量式正交编码器解码模块
//           1. 四倍频解码 A/B 两相正交信号，识别正转/反转
//           2. 输出绝对计数与相对计数，二者严格同拍更新
//           3. 1 count = 360/1067 = 0.3374 度
// =============================================================================

`timescale 1ns / 1ps

module qep_encoder #(
    parameter POS_INIT = 16'd10000
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enc_a,        // Y20
    input  wire               enc_b,        // AA20
    output reg  [15:0]        pos_raw,      // 原始计数值，复位初值 POS_INIT
    output reg  signed [15:0] pos_rel,      // = pos_raw - POS_INIT，有符号，正=正转
    output wire signed [15:0] pos_delta     // 本拍位置增量（用于测速）
);

    // -------------------------------------------------------------------------
    // 第 1 步：A/B 相各 2 级同步打拍抗亚稳态
    // -------------------------------------------------------------------------
    reg enc_a_s0, enc_a_s1;
    reg enc_b_s0, enc_b_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_a_s0 <= 1'b0;
            enc_a_s1 <= 1'b0;
            enc_b_s0 <= 1'b0;
            enc_b_s1 <= 1'b0;
        end else begin
            enc_a_s0 <= enc_a;
            enc_a_s1 <= enc_a_s0;
            enc_b_s0 <= enc_b;
            enc_b_s1 <= enc_b_s0;
        end
    end

    // -------------------------------------------------------------------------
    // 第 2 步：当前状态与上一拍状态
    // -------------------------------------------------------------------------
    wire [1:0] state_now;
    assign state_now = {enc_a_s1, enc_b_s1};   // [1]=A, [0]=B

    reg [1:0] state_last;

    // -------------------------------------------------------------------------
    // 第 3 步：四相状态机判方向，8 种合法跳变 ±1
    // -------------------------------------------------------------------------
    reg signed [15:0] delta_next;

    // 以 {上一拍, 本拍} 的组合做译码：相邻状态迁移给 ±1，其余情况视为毛刺或
    // 两相同时变化，增量记 0，避免误计数
    always @(*) begin
        case ({state_last, state_now})
            // ---- 合法跳变：+1（正转）----
            4'b0001: delta_next = 16'sd1;    // 00 -> 01
            4'b0111: delta_next = 16'sd1;    // 01 -> 11
            4'b1110: delta_next = 16'sd1;    // 11 -> 10
            4'b1000: delta_next = 16'sd1;    // 10 -> 00
            // ---- 合法跳变：-1（反转）----
            4'b0010: delta_next = -16'sd1;   // 00 -> 10
            4'b0100: delta_next = -16'sd1;   // 01 -> 00
            4'b1101: delta_next = -16'sd1;   // 11 -> 01
            4'b1011: delta_next = -16'sd1;   // 10 -> 11
            // ---- 其余（同状态或两位同时变化的非法跳变）：保持不变 ----
            default: delta_next = 16'sd0;
        endcase
    end

    // pos_delta 每个时钟周期输出“本拍增量”
    assign pos_delta = delta_next;

    // -------------------------------------------------------------------------
    // 第 4 步：位置累加，pos_raw 与 pos_rel 同拍更新
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_last <= 2'b00;
            pos_raw    <= POS_INIT;
            pos_rel    <= 16'sd0;
        end else begin
            state_last <= state_now;
            pos_raw    <= pos_raw + delta_next;
            // 用 pos_raw 的当前值（旧值）参与计算，保证与 pos_raw 同拍一致。
            pos_rel    <= $signed(pos_raw - POS_INIT) + delta_next;
        end
    end

endmodule
