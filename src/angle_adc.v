// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: angle_adc
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 倒立摆摆杆角度采样子模块（3PA1030 并行 ADC）
//           1. AD 时钟 25MHz（50MHz 二分频），ADC_OE_N 常使能
//           2. 每 187.5us 采一点，3ms 窗口累加 16 点后右移 4 位输出
//           3. 输出 angle_vld 作为全系统控制节拍：角度更新周期 = 控制周期 = 3ms
//           4. 复位后丢弃首个不完整窗口，避免上电首周期角度错误
// =============================================================================

`timescale 1ns / 1ps

module angle_adc #(
    parameter integer SMP_3MS  = 149_999,    // 3ms 窗口 @50MHz = 150000 拍
    parameter integer SMP_STEP =   9_373     // 累加间隔 - 1（187.5us @50MHz）
)(
    input  wire        clk_50m,      // 50MHz 系统时钟
    input  wire        rst_n,        // 异步复位，低有效

    output wire        adc_clk,      // G18, 输出给 3PA1030 的 CLK 引脚，25MHz
    output wire        adc_oe_n,     // H20, 低有效输出使能
    input  wire [9:0]  adc_data_in,  // 3PA1030 并行数据

    output reg  [11:0] angle_raw,    // 平均后的角度原始值，12bit，0~4095
    output reg         angle_vld     // 1 拍脉冲：本拍输出的角度值有效，
                                     // 同时作为整个控制环的控制节拍（3ms）
);

    //-------------------------------------------------------------------------
    // 1. adc_clk：50MHz 二分频 = 25MHz，纯计数器翻转，占空比 50%
    //-------------------------------------------------------------------------
    reg adc_clk_r;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) adc_clk_r <= 1'b0;
        else        adc_clk_r <= ~adc_clk_r;
    end
    assign adc_clk = adc_clk_r;

    assign adc_oe_n = 1'b0;      // 常低使能 3PA1030 输出

    //-------------------------------------------------------------------------
    // 2. 10bit -> 12bit：低位补两个 0，等效乘以 4，为后续定点运算保留小数位
    //-------------------------------------------------------------------------
    wire [11:0] theta_12bit = {adc_data_in, 2'b0};

    localparam integer STEP_P  = SMP_STEP + 1;    // 每步拍数
    localparam integer NPT     = 16;              // 每窗采样点数（定死）
    localparam integer WIN_END = NPT * STEP_P;    // 窗口拍数
    localparam integer WINW    = 18;              // WIN_END 最大 16*9374 = 149984

    reg [WINW-1:0] win_cnt;                       // 0 .. WIN_END-1

    // 窗口结束脉冲与采样点脉冲，均由计数器比较得到，无额外状态机
    wire out_pulse  = (win_cnt == (WIN_END - 1));
    wire step_pulse = ((win_cnt % STEP_P) == 0) && (win_cnt < (WIN_END - 1));

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) win_cnt <= {WINW{1'b0}};
        else        win_cnt <= out_pulse ? {WINW{1'b0}} : (win_cnt + 1'b1);
    end

    //-------------------------------------------------------------------------
    // 3. 累加平均
    //-------------------------------------------------------------------------
    // 累加：theta_sum 每个 step_pulse 加一个采样点。
    reg [15:0] theta_sum;
    reg        warmup;      // 上电首个窗口不完整，置 1 时丢弃

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            theta_sum <= 16'd0;
            angle_raw <= 12'd0;
            angle_vld <= 1'b0;
            warmup    <= 1'b1;
        end
        else begin
            angle_vld <= 1'b0;                  // 默认不脉冲

            if (warmup) begin
                // 复位后第一个窗口的点数不足 16，只清累加器、不输出
                if (out_pulse) begin
                    warmup    <= 1'b0;
                    theta_sum <= 16'd0;
                end
            end
            else if (out_pulse) begin
                // 输出：累加器里正好 16 个点（最后一个 step_pulse 已经加过）
                angle_raw <= theta_sum[15:4];   // >>4 == 除以 16
                angle_vld <= 1'b1;              // 控制节拍（3ms）
                theta_sum <= 16'd0;             // 清累加器，准备下一窗
            end
            else if (step_pulse) begin
                theta_sum <= theta_sum + {4'b0, theta_12bit};
            end
            // else: 保持
        end
    end

endmodule
