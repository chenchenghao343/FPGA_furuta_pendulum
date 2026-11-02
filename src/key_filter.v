// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: key_filter
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 通用按键消抖模块
//           1. 两级同步 + 计数消抖，消抖时间由 CNT_MAX 决定（20ms @50MHz）
//           2. key_flag 只在确认按下的那一拍输出 1 个脉冲，长按不重复触发
//           3. key_state 输出消抖后的稳定电平
// =============================================================================

`timescale 1ns / 1ps

module key_filter #(
    parameter CNT_MAX = 20'd1_000_000     // 20ms @50MHz
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key_in,      // 异步输入，低有效
    output reg  key_flag,    // 消抖确认按下时输出 1 拍脉冲
    output reg  key_state    // 消抖后的电平, 1=松开, 0=按下
);

    // -------------------------------------------------------------------------
    // 第 1 步：2 级同步打拍抗亚稳态
    // -------------------------------------------------------------------------
    reg key_sync0;
    reg key_sync1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sync0 <= 1'b1;   // 空闲电平为 1（松开）
            key_sync1 <= 1'b1;
        end else begin
            key_sync0 <= key_in;
            key_sync1 <= key_sync0;
        end
    end

    // -------------------------------------------------------------------------
    // 第 2 步：消抖计数 + 状态确认
    //   计数器只在同步后的电平与稳定状态不一致时前进，一旦回摆立即清零，
    //   因此要求电平连续保持 CNT_MAX 拍才承认翻转，短于该时间的抖动全部滤除
    // -------------------------------------------------------------------------
    reg [19:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt       <= 20'd0;
            key_flag  <= 1'b0;
            key_state <= 1'b1;   // 复位后视为松开
        end else begin
            key_flag <= 1'b0;    // 默认输出 0，仅在确认按下那一拍拉高

            if (key_sync1 != key_state) begin
                // 输入电平与当前稳定状态不一致 -> 疑似有动作，开始/继续消抖计时
                if (cnt == CNT_MAX) begin
                    // 持续不一致已达 CNT_MAX 拍，确认状态翻转
                    cnt       <= 20'd0;
                    key_state <= key_sync1;
                    if (key_sync1 == 1'b0) begin
                        key_flag <= 1'b1;   // 确认按下：输出 1 拍脉冲
                    end
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end else begin
                // 电平与稳定状态一致 -> 抖动回弹或稳定，计数器清零重来
                cnt <= 20'd0;
            end
        end
    end

endmodule
