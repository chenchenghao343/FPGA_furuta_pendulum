// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: limit_prot
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 摆杆角度与摆臂位置越界保护模块
//           1. 摆杆角度超限或摆臂偏心超限时置位故障标志
//           2. 故障时强制控制量归零、电机停转，防止撞击或线缆拧断
//           3. 上下限可配，默认关闭角度检查，仅保留位置保护
// =============================================================================

`timescale 1ns / 1ps

module limit_prot #(
    //-------------------------------------------------------------------------
    // 角度量程检查默认已关闭
    //-------------------------------------------------------------------------
    parameter ANGLE_MIN  = 12'd0,       // 0 = 关闭下限检查（取量程端点即等效关闭）
    parameter ANGLE_MAX  = 12'd4095,    // 4095 = 关闭上限检查（12bit 满量程）
    parameter POS_LIMIT  = 16'd6000     // 摆臂偏心绝对限幅（编码器计数，相对中点）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,          // 控制节拍(angle_vld)
    input  wire               run_en,        // 总使能
    input  wire signed [15:0] pos_rel,       // 摆臂相对位置
    input  wire [11:0]        angle_raw,     // 摆杆角度
    output reg                arm_fault,     // 摆臂越界
    output reg                ang_fault,     // 角度越界(传感器异常/断线)
    output reg                fault          // 综合故障
);

    // 恢复确认所需的连续 tick 数（50 × 3ms = 150ms）
    localparam [15:0] RECOVER_TICKS = 16'd50;

    //-------------------------------------------------------------------------
    // 越界条件（组合判断，在 tick 时采样）
    //-------------------------------------------------------------------------
    // 摆臂：|pos_rel| > POS_LIMIT
    wire signed [16:0] pos_ext;
    assign pos_ext = $signed(pos_rel);

    wire signed [16:0] pos_abs;
    assign pos_abs = (pos_ext < 0) ? (-pos_ext) : pos_ext;

    wire arm_over;
    assign arm_over = (pos_abs > $signed({1'b0, POS_LIMIT}));

    // 角度：angle_raw < ANGLE_MIN 或 > ANGLE_MAX
    wire ang_over;
    assign ang_over = (angle_raw < ANGLE_MIN) || (angle_raw > ANGLE_MAX);

    //-------------------------------------------------------------------------
    // 故障锁存 + 恢复确认计数
    //-------------------------------------------------------------------------
    reg [15:0] arm_ok_cnt;   // 摆臂回到范围内的连续 tick 计数
    reg [15:0] ang_ok_cnt;   // 角度回到范围内的连续 tick 计数

    //-------------------------------------------------------------------------
    // 两路故障的"次态"（组合），供本拍寄存器更新与 fault 汇总共用同一份判据
    //-------------------------------------------------------------------------
    wire arm_recover_done = (arm_ok_cnt >= RECOVER_TICKS - 1'b1);
    wire ang_recover_done = (ang_ok_cnt >= RECOVER_TICKS - 1'b1);

    wire arm_fault_n = arm_over ? 1'b1 :
                       (arm_fault ? (arm_recover_done ? 1'b0 : 1'b1) : 1'b0);
    wire ang_fault_n = ang_over ? 1'b1 :
                       (ang_fault ? (ang_recover_done ? 1'b0 : 1'b1) : 1'b0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arm_fault  <= 1'b0;
            ang_fault  <= 1'b0;
            fault      <= 1'b0;
            arm_ok_cnt <= 16'd0;
            ang_ok_cnt <= 16'd0;
        end else if (!run_en) begin
            // 总使能关闭：故障无条件清零，便于重新投入闭环
            arm_fault  <= 1'b0;
            ang_fault  <= 1'b0;
            fault      <= 1'b0;
            arm_ok_cnt <= 16'd0;
            ang_ok_cnt <= 16'd0;
        end else if (tick) begin
            // ------------------- 摆臂故障 -------------------
            if (arm_over) begin
                arm_fault  <= 1'b1;      // 越界：置位并锁存
                arm_ok_cnt <= 16'd0;     // 恢复计数清零
            end else if (arm_fault) begin
                // 已回到范围内：需连续保持 RECOVER_TICKS 个 tick 才清除
                if (arm_ok_cnt >= RECOVER_TICKS - 1'b1) begin
                    arm_fault  <= 1'b0;
                    arm_ok_cnt <= 16'd0;
                end else begin
                    arm_ok_cnt <= arm_ok_cnt + 1'b1;
                end
            end else begin
                arm_ok_cnt <= 16'd0;
            end

            // ------------------- 角度故障 -------------------
            if (ang_over) begin
                ang_fault  <= 1'b1;      // 越界：置位并锁存
                ang_ok_cnt <= 16'd0;
            end else if (ang_fault) begin
                if (ang_ok_cnt >= RECOVER_TICKS - 1'b1) begin
                    ang_fault  <= 1'b0;
                    ang_ok_cnt <= 16'd0;
                end else begin
                    ang_ok_cnt <= ang_ok_cnt + 1'b1;
                end
            end else begin
                ang_ok_cnt <= 16'd0;
            end

            // ------------------- 综合故障 -------------------
            fault <= arm_fault_n | ang_fault_n;
        end
    end

endmodule
