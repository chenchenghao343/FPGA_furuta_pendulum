// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: numfmt
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 有符号定点数转十进制字符模块（OLED 数字显示用）
//           1. 把 18 位有符号数拆成符号 + 5 位十进制数字，逐位输出
//           2. 不用除法器/取模：逐位试减实现，避免大规模组合逻辑拖长关键路径
// =============================================================================

`timescale 1ns/1ps

module numfmt (
    input  wire               clk,
    input  wire               rst_n,

    input  wire               start,      // 1 拍：启动一次转换
    input  wire signed [17:0] value,      // 待转换的数值

    output reg                busy,       // 转换中
    output reg                done,       // 1 拍：结果有效
    output reg                neg,        // 1 = 负数
    output reg  [3:0]         d4,         // 万位
    output reg  [3:0]         d3,         // 千位
    output reg  [3:0]         d2,         // 百位
    output reg  [3:0]         d1,         // 十位
    output reg  [3:0]         d0          // 个位
);

    //-------------------------------------------------------------------------
    // 状态编码：S_D4 ~ S_D0 依次表示"正在用 10^4 ~ 10^0 这个除数反复减"
    //-------------------------------------------------------------------------
    localparam [2:0] S_IDLE = 3'd0,
                     S_ABS  = 3'd1,
                     S_D4   = 3'd2,
                     S_D3   = 3'd3,
                     S_D2   = 3'd4,
                     S_D1   = 3'd5,
                     S_D0   = 3'd6,
                     S_DONE = 3'd7;

    reg  [2:0]         st;
    reg  signed [17:0] acc;        // 剩余值，边减边变小
    reg  [17:0]        div;        // 当前除数

    // 减法候选：acc - div
    wire signed [18:0] acc_ext = {acc[17], acc};
    wire signed [18:0] div_ext = {1'b0, div};
    wire signed [18:0] sub     = acc_ext - div_ext;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st   <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            neg  <= 1'b0;
            acc  <= 18'sd0;
            div  <= 18'd0;
            d4   <= 4'd0;
            d3   <= 4'd0;
            d2   <= 4'd0;
            d1   <= 4'd0;
            d0   <= 4'd0;
        end
        else begin
            done <= 1'b0;                       // 默认不脉冲

            case (st)
            //-------------------------------------------------------------------------
            S_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    busy <= 1'b1;
                    neg  <= value[17];          // 补码最高位即符号
                    acc  <= value[17] ? -value[17:0] : value[17:0];  // 取绝对值
                    d4 <= 4'd0; d3 <= 4'd0; d2 <= 4'd0; d1 <= 4'd0; d0 <= 4'd0;
                    st   <= S_D4;
                    div  <= 18'd10000;
                end
            end
            //-------------------------------------------------------------------------
            // 每一位：只要 acc >= div 就减一次、该位 +1。
            //-------------------------------------------------------------------------
            S_D4, S_D3, S_D2, S_D1, S_D0: begin
                if (sub >= 19'sd0) begin
                    acc <= sub[17:0];
                    case (st)
                    S_D4: d4 <= d4 + 4'd1;
                    S_D3: d3 <= d3 + 4'd1;
                    S_D2: d2 <= d2 + 4'd1;
                    S_D1: d1 <= d1 + 4'd1;
                    default: d0 <= d0 + 4'd1;
                    endcase
                end
                else begin
                    case (st)
                    S_D4: begin div <= 18'd1000;  st <= S_D3; end
                    S_D3: begin div <= 18'd100;   st <= S_D2; end
                    S_D2: begin div <= 18'd10;    st <= S_D1; end
                    S_D1: begin div <= 18'd1;     st <= S_D0; end
                    default: st <= S_DONE;
                    endcase
                end
            end
            //-------------------------------------------------------------------------
            S_DONE: begin
                busy <= 1'b0;
                done <= 1'b1;
                st   <= S_IDLE;
            end
            //-------------------------------------------------------------------------
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
