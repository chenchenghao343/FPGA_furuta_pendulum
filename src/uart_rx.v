// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: uart_rx
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: UART 接收模块（9600bps 8N1）
//           1. 3 级同步打拍后检测起始位下降沿，每位内计数到中点才采样，提高抗噪能力
//           2. 接收到完整字节后输出 1 拍有效脉冲与数据
//           3. 停止位不为高时标记帧错误并重新同步
// =============================================================================

`timescale 1ns/1ns

module uart_rx #(
    parameter CLK_FREQ = 26'd50_000_000,
    parameter UART_BPS = 21'd9600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,         // 串口输入
    output reg  [7:0] po_data,    // 收到的字节
    output reg        po_flag,    // 收到一个字节时 1 拍脉冲
    output reg        frame_err   // 停止位错误时 1 拍脉冲
);

    //-------------------------------------------------------------------------
    // 位宽常数
    //-------------------------------------------------------------------------
    localparam integer BIT_CNT_MAX = CLK_FREQ / UART_BPS;   // = 5208
    localparam integer BIT_CNT_MID = BIT_CNT_MAX / 2;       // = 2604

    localparam [3:0] IDX_START = 4'd0;
    localparam [3:0] IDX_STOP  = 4'd9;

    //-------------------------------------------------------------------------
    // 3 级同步打拍
    //-------------------------------------------------------------------------
    reg rx_sync1, rx_sync2, rx_sync3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;      // 空闲为高，复位也置高
            rx_sync2 <= 1'b1;
            rx_sync3 <= 1'b1;
        end else begin
            rx_sync1 <= rx;
            rx_sync2 <= rx_sync1;
            rx_sync3 <= rx_sync2;
        end
    end

    // 下降沿检测（rx_sync3 比 rx_sync2 晚一拍）
    wire start_nedge = (~rx_sync2) & rx_sync3;

    //-------------------------------------------------------------------------
    // 接收使能
    //-------------------------------------------------------------------------
    reg work_en;

    //-------------------------------------------------------------------------
    // 起始位识别：下降沿必须叠加"当前空闲"条件，
    // 否则停止位期间出现的窄脉冲会让接收器提前重启
    //-------------------------------------------------------------------------
    wire start_bit_detect = start_nedge & ~work_en;

    //-------------------------------------------------------------------------
    // 位内计数器：仅在接收期间计数，满 BIT_CNT_MAX-1 归零
    //-------------------------------------------------------------------------
    reg [12:0] baud_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            baud_cnt <= 13'd0;
        else if (!work_en)
            baud_cnt <= 13'd0;
        else if (baud_cnt == BIT_CNT_MAX[12:0] - 13'd1)
            baud_cnt <= 13'd0;
        else
            baud_cnt <= baud_cnt + 13'd1;
    end

    //-------------------------------------------------------------------------
    // 采样脉冲：位中点
    //-------------------------------------------------------------------------
    reg samp_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            samp_tick <= 1'b0;
        else if (work_en && (baud_cnt == BIT_CNT_MID[12:0]))
            samp_tick <= 1'b1;
        else
            samp_tick <= 1'b0;
    end

    //-------------------------------------------------------------------------
    // 位索引：0=起始位, 1..8=D0..D7, 9=停止位
    //-------------------------------------------------------------------------
    reg [3:0] bit_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bit_idx <= 4'd0;
        else if (start_bit_detect)
            bit_idx <= 4'd0;
        else if (samp_tick && work_en)
            bit_idx <= bit_idx + 4'd1;
    end

    //-------------------------------------------------------------------------
    // 移位接收（LSB first）
    //-------------------------------------------------------------------------
    reg [7:0] rx_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_data <= 8'd0;
        else if (samp_tick && work_en && (bit_idx >= 4'd1) && (bit_idx <= 4'd8))
            rx_data <= {rx_sync3, rx_data[7:1]};
    end

    //-------------------------------------------------------------------------
    // 停止位采样与判定
    //-------------------------------------------------------------------------
    wire stop_sample = samp_tick & work_en & (bit_idx == IDX_STOP);
    wire stop_ok     = (rx_sync3 == 1'b1);

    //-------------------------------------------------------------------------
    // work_en 控制：真正的起始位置 1，停止位处理完归 0
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            work_en <= 1'b0;
        else if (start_bit_detect)
            work_en <= 1'b1;
        else if (stop_sample)
            work_en <= 1'b0;   // 无论对错，收完 10 位即回空闲，重新等起始沿
    end

    //-------------------------------------------------------------------------
    // 输出寄存器
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            po_data   <= 8'd0;
            po_flag   <= 1'b0;
            frame_err <= 1'b0;
        end else begin
            po_flag   <= 1'b0;
            frame_err <= 1'b0;

            if (stop_sample) begin
                if (stop_ok) begin
                    po_data <= rx_data;     // 与 po_flag 同拍输出
                    po_flag <= 1'b1;
                end else begin
                    frame_err <= 1'b1;      // 停止位不为高：丢弃本字节，不输出 po_flag
                end
            end
        end
    end

endmodule
