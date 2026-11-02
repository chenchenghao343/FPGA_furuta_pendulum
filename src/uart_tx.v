// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: uart_tx
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: UART 发送模块（9600bps 8N1）
//           1. 把 1 字节数据加起始位/停止位，按 9600bps 串行移出
//           2. 启动拍锁存整字节，发送期间输入变化不影响波形
//           3. 停止位结束时给出 1 拍 pi_done，调用方据此推进下一字节
// =============================================================================

module uart_tx #(
    parameter CLK_FREQ = 26'd50_000_000,
    parameter UART_BPS = 21'd9600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] pi_data,   // 待发送字节
    input  wire       pi_flag,   // 1 拍脉冲:开始发送
    output reg        tx,        // 串口输出线，空闲为高
    output reg        pi_done    // 发送完一个字节时输出 1 拍脉冲
);

    //-------------------------------------------------------------------------
    // 位宽常数:每位占多少个时钟
    //-------------------------------------------------------------------------
    localparam integer BIT_CNT_MAX = CLK_FREQ / UART_BPS;   // = 5208

    localparam [3:0] BIT_IDLE = 4'd0;   // 空闲
    localparam [3:0] BIT_LAST = 4'd9;   // 停止位索引

    reg        busy;         // 1 = 正在发送一帧
    reg [12:0] baud_cnt;     // 位内计数 0 ~ BIT_CNT_MAX-1
    reg [3:0]  bit_idx;      // 位索引 0=起始 1..8=数据 9=停止
    // bit_tick 由 baud_cnt 直接译码成组合 wire，不额外占用寄存器

    //-------------------------------------------------------------------------
    // 位计数器:仅在工作时计数,数满 BIT_CNT_MAX-1 归零
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            baud_cnt <= 13'd0;
        else if (!busy)
            baud_cnt <= 13'd0;
        else if (baud_cnt == BIT_CNT_MAX[12:0] - 13'd1)
            baud_cnt <= 13'd0;
        else
            baud_cnt <= baud_cnt + 13'd1;
    end

    //-------------------------------------------------------------------------
    // bit_tick:数满一整位后产生 1 拍
    //-------------------------------------------------------------------------
    wire bit_tick = busy && (baud_cnt == BIT_CNT_MAX[12:0] - 13'd1);

    //-------------------------------------------------------------------------
    // 数据锁存:启动拍取走 pi_data,后续移位只用锁存值
    //-------------------------------------------------------------------------
    reg [7:0] tx_shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tx_shift <= 8'd0;
        else if (pi_flag && !busy)
            tx_shift <= pi_data;      // 在启动那一刻锁存整字节
    end

    //-------------------------------------------------------------------------
    // busy:pi_flag 启动;停止位结束(bit_tick && bit_idx==9)时结束
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            busy <= 1'b0;
        else if (pi_flag && !busy)
            busy <= 1'b1;
        else if (bit_tick && (bit_idx == BIT_LAST))
            busy <= 1'b0;
    end

    //-------------------------------------------------------------------------
    // 位索引推进:启动时置 0(起始位),之后每位 +1
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bit_idx <= 4'd0;
        else if (pi_flag && !busy)
            bit_idx <= 4'd0;
        else if (bit_tick && busy)
            bit_idx <= bit_idx + 4'd1;
    end

    //-------------------------------------------------------------------------
    // tx 数据输出
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tx <= 1'b1;                                  // 空闲高
        else if (pi_flag && !busy)
            tx <= 1'b0;                                  // 起始位
        else if (bit_tick && busy) begin
            case (bit_idx + 4'd1)                        // 切到下一位
                4'd1: tx <= tx_shift[0];                 // LSB 先出，用锁存值
                4'd2: tx <= tx_shift[1];
                4'd3: tx <= tx_shift[2];
                4'd4: tx <= tx_shift[3];
                4'd5: tx <= tx_shift[4];
                4'd6: tx <= tx_shift[5];
                4'd7: tx <= tx_shift[6];
                4'd8: tx <= tx_shift[7];
                default: tx <= 1'b1;                     // 停止位 / 收尾回空闲高
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // pi_done:停止位结束时输出 1 拍脉冲
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pi_done <= 1'b0;
        else if (bit_tick && busy && (bit_idx == BIT_LAST))
            pi_done <= 1'b1;
        else
            pi_done <= 1'b0;
    end

endmodule
