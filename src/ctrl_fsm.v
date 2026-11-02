// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: ctrl_fsm
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 系统主控状态机（空闲 / 起摆 / 平衡 / 故障）
//           1. 四状态机：IDLE -> SWING -> BALANCE，异常进 FAULT 并锁存
//           2. 起摆->平衡用角度偏差 + 角速度双门限捕获，避免误捕获
//           3. 平衡->起摆用丢失判据 + 连续保持计数迟滞，抵抗瞬时抖动
//           4. 故障判据不经过 tick 采样，安全事件当拍生效
// =============================================================================

`timescale 1ns/1ns

module ctrl_fsm #(
    parameter integer W = 16,

    // ---- 状态编码（同时是遥测 w2 的取值）----
    parameter [1:0] MODE_IDLE    = 2'd0,
    parameter [1:0] MODE_SWING   = 2'd1,
    parameter [1:0] MODE_BALANCE = 2'd2,
    parameter [1:0] MODE_FAULT   = 2'd3,

    // ---- 判据阈值 ----
    parameter signed [W-1:0] CAPTURE_BAND = 16'sd130,  // 起摆→平衡：角度偏差阈值
    parameter signed [W-1:0] CAPTURE_RATE = 16'sd55,   // 起摆→平衡：角速度阈值
    parameter signed [W-1:0] LOST_BAND    = 16'sd420,  // 平衡→起摆：丢失阈值
    parameter integer        LOST_HOLD    = 12         // 丢失需连续保持的控制周期数
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,       // 控制节拍
    input  wire               run_req,    // 运行请求（按键或串口 'R'），电平信号
    input  wire               fault,      // 来自 limit_prot 的综合故障
    input  wire signed [W-1:0] phi,       // 相对直立的偏差角
    input  wire signed [W-1:0] dtheta,    // 角速度

    output reg  [1:0]         mode,       // 当前状态（遥测 w2）
    output reg                run_pid,    // 1 = PID 闭环投入
    output reg                run_swing,  // 1 = 起摆能量泵投入
    output reg                run_pos     // 1 = 位置环投入
);

    //-------------------------------------------------------------------------
    // run_req 上升沿检测（按键/串口都是"按一下切换一次"的语义）
    //-------------------------------------------------------------------------
    reg run_req_r;
    wire run_req_rise = run_req & ~run_req_r;

    //-------------------------------------------------------------------------
    // 判据
    //-------------------------------------------------------------------------
    wire signed [W-1:0] phi_abs = (phi >= 0) ? phi : -phi;
    wire signed [W-1:0] dth_abs = (dtheta >= 0) ? dtheta : -dtheta;

    wire cap_now  = (phi_abs < CAPTURE_BAND) && (dth_abs < CAPTURE_RATE);
    wire lost_now = (phi_abs > LOST_BAND);

    //-------------------------------------------------------------------------
    // 丢失保持计数
    //-------------------------------------------------------------------------
    reg [7:0] lost_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            lost_cnt <= 8'd0;
        else if (!tick)
            ; // 非控制节拍保持
        else if (lost_now) begin
            if (lost_cnt < LOST_HOLD)
                lost_cnt <= lost_cnt + 8'd1;
        end
        else
            lost_cnt <= 8'd0;
    end

    wire lost_confirmed = (lost_cnt >= LOST_HOLD[7:0]);

    //-------------------------------------------------------------------------
    // 状态寄存器
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode   <= MODE_IDLE;
            run_req_r <= 1'b0;
        end
        else begin
            run_req_r <= run_req;

            // 故障优先级最高，且不依赖 tick（安全相关，立即响应）
            if (fault) begin
                mode <= MODE_FAULT;
            end
            else begin
                case (mode)
                    //-------------------------------------------------------------------------
                    MODE_IDLE: begin
                        if (run_req_rise)
                            mode <= MODE_SWING;
                    end
                    //-------------------------------------------------------------------------
                    MODE_SWING: begin
                        if (run_req_rise)
                            mode <= MODE_IDLE;          // 手动停机
                        else if (tick && cap_now)
                            mode <= MODE_BALANCE;       // 捕获成功
                    end
                    //-------------------------------------------------------------------------
                    MODE_BALANCE: begin
                        if (run_req_rise)
                            mode <= MODE_IDLE;          // 手动停机
                        else if (tick && lost_confirmed)
                            mode <= MODE_SWING;         // 丢失，重新起摆
                    end
                    //-------------------------------------------------------------------------
                    MODE_FAULT: begin
                        // 故障消失后回空闲，必须重新按运行键才动作
                        mode <= MODE_IDLE;
                    end
                    //-------------------------------------------------------------------------
                    default: mode <= MODE_IDLE;
                endcase
            end
        end
    end

    //-------------------------------------------------------------------------
    // 输出译码（组合，与 mode 同拍生效）
    //-------------------------------------------------------------------------
    always @(*) begin
        case (mode)
            MODE_SWING: begin
                run_swing = 1'b1;
                run_pid   = 1'b0;
                run_pos   = 1'b0;    // 起摆期间位置环不投入，避免干扰能量泵
            end
            MODE_BALANCE: begin
                run_swing = 1'b0;
                run_pid   = 1'b1;
                run_pos   = 1'b1;    // 平衡期间位置环投入（权重已在 PID 里削弱）
            end
            default: begin
                run_swing = 1'b0;
                run_pid   = 1'b0;
                run_pos   = 1'b0;
            end
        endcase
    end

endmodule
