// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: oled_ssd1306
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: SSD1306 OLED 屏驱动模块（I2C 时序 + 显存写入）
//           1. 产生 I2C 时序，完成 SSD1306 上电初始化序列
//           2. 按页写入 128x64 显存，整帧 1024 字节串行送出
//           3. SCL 频率由 QDIV 分频决定，保证 400kHz 以内可靠通信
//           4. 传输按四阶段循环：初始化 → 设显存窗口 → 等渲染 → 写显存
// =============================================================================

`timescale 1ns/1ps

module oled_ssd1306 #(
    // 1/4 SCL 周期 = (QDIV+1) 个 clk。62 -> 198kHz（@50MHz）
    parameter integer QDIV = 62
)(
    input  wire        clk,
    input  wire        rst_n,

    //---------------------------------------------------------------------
    // I2C 引脚（开漏：1 = 拉低，0 = 释放）
    //---------------------------------------------------------------------
    output reg         scl_lo,
    output reg         sda_lo,
    input  wire        sda_i,       // 直接接回 oled_sda 引脚

    //---------------------------------------------------------------------
    // 帧缓冲写口（来自 oled_disp）
    //---------------------------------------------------------------------
    input  wire        fb_we,
    input  wire [9:0]  fb_addr,
    input  wire [7:0]  fb_data,

    //---------------------------------------------------------------------
    // 与 oled_disp 的握手
    //---------------------------------------------------------------------
    output reg         frame_req,   // 1 拍：请渲染一帧
    input  wire        disp_done,   // 1 拍：渲染完成

    //---------------------------------------------------------------------
    // 状态输出（顶层未接，可接逻辑分析仪观察）
    //---------------------------------------------------------------------
    output reg         ack_err,     // 收到 NACK 时置 1 并保持，直到复位
    output wire        busy         // 正在传输
);

    //=========================================================================
    // 帧缓冲：8 页 x 128 列 = 1024 字节
    //=========================================================================
    reg [7:0]  fb [0:1023];
    reg [9:0]  raddr;
    reg [7:0]  rdata;

    always @(posedge clk) begin
        if (fb_we) fb[fb_addr] <= fb_data;
        rdata <= fb[raddr];
    end

    //=========================================================================
    // 初始化命令表（SSD1306, 128x64, 内部电荷泵）
    //=========================================================================
    localparam integer INIT_N = 25;

    function [7:0] init_rom;
        input [4:0] i;
        begin
            case (i)
            5'd0:  init_rom = 8'hAE;   // 关显示
            5'd1:  init_rom = 8'hD5;   // 时钟分频/振荡频率
            5'd2:  init_rom = 8'h80;
            5'd3:  init_rom = 8'hA8;   // 多路复用比
            5'd4:  init_rom = 8'h3F;   // 1/64 duty（64 行）
            5'd5:  init_rom = 8'hD3;   // 显示偏移
            5'd6:  init_rom = 8'h00;
            5'd7:  init_rom = 8'h40;   // 起始行 = 0
            5'd8:  init_rom = 8'h8D;   // 电荷泵
            5'd9:  init_rom = 8'h14;   // 0x14 = 开
            5'd10: init_rom = 8'h20;   // 寻址模式
            5'd11: init_rom = 8'h00;   // 0x00 = 水平寻址（自动换行/换页）
            5'd12: init_rom = 8'hA1;   // 段重映射（左右镜像）
            5'd13: init_rom = 8'hC8;   // COM 扫描方向（上下镜像）
            5'd14: init_rom = 8'hDA;   // COM 引脚配置
            5'd15: init_rom = 8'h12;   // 交替 + 关闭左右反转
            5'd16: init_rom = 8'h81;   // 对比度
            5'd17: init_rom = 8'hCF;
            5'd18: init_rom = 8'hD9;   // 预充电周期
            5'd19: init_rom = 8'hF1;
            5'd20: init_rom = 8'hDB;   // VCOMH 电压
            5'd21: init_rom = 8'h40;
            5'd22: init_rom = 8'hA4;   // 按显存显示（非全亮）
            5'd23: init_rom = 8'hA6;   // 正常显示（非反显）
            5'd24: init_rom = 8'hAF;   // 开显示
            default: init_rom = 8'h00;
            endcase
        end
    endfunction

    // 显存窗口：列 0..127，页 0..7
    localparam integer WIN_N = 6;
    function [7:0] win_rom;
        input [2:0] i;
        begin
            case (i)
            3'd0: win_rom = 8'h21;   // 设置列地址
            3'd1: win_rom = 8'h00;   // 起始列 0
            3'd2: win_rom = 8'h7F;   // 结束列 127
            3'd3: win_rom = 8'h22;   // 设置页地址
            3'd4: win_rom = 8'h00;   // 起始页 0
            3'd5: win_rom = 8'h07;   // 结束页 7
            default: win_rom = 8'h00;
            endcase
        end
    endfunction

    //=========================================================================
    // 阶段与字节索引
    //=========================================================================
    localparam [1:0] P_INIT = 2'd0,
                     P_WIN  = 2'd1,
                     P_DATA = 2'd2,
                     P_REND = 2'd3;   // 等 oled_disp 渲染完

    reg [1:0]  phase;
    reg [10:0] bi;                  // 阶段内字节索引
    reg [10:0] blen;                // 本阶段总字节数

    reg [7:0] tx_byte;              // 当前正在移出的字节

    // 下一字节的 payload 字节
    wire [10:0] nidx = bi - 11'd1;
    reg  [7:0]  pay_next;
    always @* begin
        case (phase)
        P_INIT:  pay_next = init_rom(nidx[4:0]);
        P_WIN:   pay_next = win_rom(nidx[2:0]);
        default: pay_next = rdata;                  // P_DATA（已预取，见 I_NEXT）
        endcase
    end

    // 控制字节：命令流 0x00，数据流 0x40
    wire [7:0] ctrl_byte = (phase == P_DATA) ? 8'h40 : 8'h00;

    //=========================================================================
    // I2C 位引擎
    //=========================================================================
    reg  [6:0]  qcnt;
    reg  [3:0]  istate;
    reg  [2:0]  bitcnt;

    localparam [3:0] I_IDLE   = 4'd0,
                     I_START0 = 4'd1,   // SCL 高、SDA 高
                     I_START1 = 4'd2,   // SDA 拉低（START 条件）
                     I_START2 = 4'd3,   // SCL 拉低
                     I_BIT0   = 4'd4,   // 设置 SDA
                     I_BIT1   = 4'd5,   // 释放 SCL（上升沿）
                     I_BIT2   = 4'd6,   // SCL 保持高
                     I_BIT3   = 4'd7,   // SCL 拉低
                     I_ACK0   = 4'd8,   // 释放 SDA
                     I_ACK1   = 4'd9,   // SCL 高，采样 ACK
                     I_ACK2   = 4'd10,
                     I_ACK3   = 4'd11,  // SCL 拉低
                     I_NEXT   = 4'd12,  // 取下一个字节 / 结束本次传输
                     I_STOP0  = 4'd13,  // SDA 低
                     I_STOP1  = 4'd14,  // SCL 高
                     I_STOP2  = 4'd15;  // 释放 SDA（STOP 条件）

    localparam [6:0] QDIV_Q = QDIV[6:0];
    wire qtick = (qcnt == QDIV_Q);
    wire go    = (phase != P_REND);

    //---------------------------------------------------------------------
    // 字节流长度
    //---------------------------------------------------------------------
    always @* begin
        case (phase)
        P_INIT:  blen = 11'd2 + INIT_N;
        P_WIN:   blen = 11'd2 + WIN_N;
        default: blen = 11'd2 + 11'd1024;
        endcase
    end

    //---------------------------------------------------------------------
    // 1/4 周期计数器
    //---------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          qcnt <= 7'd0;
        else if (qtick)      qcnt <= 7'd0;
        else                 qcnt <= qcnt + 7'd1;
    end

    //---------------------------------------------------------------------
    // 状态机
    //---------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            istate    <= I_IDLE;
            scl_lo    <= 1'b0;          // 释放 = 由上拉拉高
            sda_lo    <= 1'b0;
            bitcnt    <= 3'd7;
            tx_byte   <= 8'h00;
            bi        <= 11'd0;
            phase     <= P_INIT;
            raddr     <= 10'd0;
            frame_req <= 1'b0;
            ack_err   <= 1'b0;
        end
        else begin
            frame_req <= 1'b0;

            //-----------------------------------------------------------------
            // P_REND 必须放在 qtick 判断【外面】：等渲染与 1/4 周期计数无关，
            // 若套在 qtick 里会漏掉只持续 1 拍的 disp_done。
            //-----------------------------------------------------------------
            if (phase == P_REND) begin
                if (disp_done) begin
                    phase <= P_DATA;
                    bi    <= 11'd0;
                    raddr <= 10'd0;
                end
            end
            else if (qtick) begin
                case (istate)
                //---------------------------------------------------------
                I_IDLE: begin
                    scl_lo <= 1'b0;
                    sda_lo <= 1'b0;
                    if (go) begin
                        istate <= I_START0;
                    end
                end
                //---------------------------------------------------------
                // START：SCL 高电平期间 SDA 由高变低
                //---------------------------------------------------------
                I_START0: begin
                    scl_lo <= 1'b0;
                    sda_lo <= 1'b0;
                    istate <= I_START1;
                end
                I_START1: begin
                    sda_lo <= 1'b1;                    // SDA 下降沿
                    istate <= I_START2;
                end
                I_START2: begin
                    scl_lo <= 1'b1;                    // 拉低 SCL，准备移位
                    // 装载本阶段第 0 个字节：从机地址 0x78（SA0=0）
                    tx_byte <= 8'h78;
                    bitcnt  <= 3'd7;
                    bi      <= 11'd0;
                    istate  <= I_BIT0;
                end
                //---------------------------------------------------------
                // 数据位：SCL 低时改 SDA，SCL 高时从机采样
                //---------------------------------------------------------
                I_BIT0: begin
                    sda_lo <= ~tx_byte[bitcnt];        // 0 -> 拉低，1 -> 释放
                    istate <= I_BIT1;
                end
                I_BIT1: begin
                    scl_lo <= 1'b0;                    // 释放 SCL = 上升沿
                    istate <= I_BIT2;
                end
                I_BIT2: begin
                    scl_lo <= 1'b0;
                    istate <= I_BIT3;
                end
                I_BIT3: begin
                    scl_lo <= 1'b1;                    // 下降沿
                    if (bitcnt == 3'd0) begin
                        istate <= I_ACK0;
                    end
                    else begin
                        bitcnt <= bitcnt - 3'd1;
                        istate <= I_BIT0;
                    end
                end
                //---------------------------------------------------------
                // ACK：主机释放 SDA，从机在第 9 个时钟把它拉低表示"收到"
                //---------------------------------------------------------
                I_ACK0: begin
                    sda_lo <= 1'b0;                    // 释放 SDA，让从机应答
                    istate <= I_ACK1;
                end
                I_ACK1: begin
                    scl_lo <= 1'b0;                    // SCL 上升沿
                    istate <= I_ACK2;
                end
                I_ACK2: begin
                    scl_lo <= 1'b0;
                    if (sda_i) ack_err <= 1'b1;        // 高 = NACK
                    istate <= I_ACK3;
                end
                I_ACK3: begin
                    scl_lo <= 1'b1;                    // SCL 下降沿
                    istate <= I_NEXT;
                end
                //---------------------------------------------------------
                // 取下个字节；到末尾就发 STOP
                //---------------------------------------------------------
                I_NEXT: begin
                    if (bi + 11'd1 >= blen) begin
                        istate <= I_STOP0;
                    end
                    else begin
                        bi     <= bi + 11'd1;
                        bitcnt <= 3'd7;
                        //-------------------------------------------------
                        // 为**下一个**字节（索引 bi+1）预装 tx_byte
                        //-------------------------------------------------
                        if (bi == 11'd0)
                            tx_byte <= ctrl_byte;
                        else
                            tx_byte <= pay_next;

                        //-------------------------------------------------
                        // raddr 预取：把"下下一字节"的地址放进去，
                        //-------------------------------------------------
                        if (phase == P_DATA)
                            raddr <= bi[9:0];

                        istate <= I_BIT0;
                    end
                end
                //---------------------------------------------------------
                // STOP：SCL 高电平期间 SDA 由低变高
                //---------------------------------------------------------
                I_STOP0: begin
                    scl_lo <= 1'b1;
                    sda_lo <= 1'b1;
                    istate <= I_STOP1;
                end
                I_STOP1: begin
                    scl_lo <= 1'b0;
                    istate <= I_STOP2;
                end
                I_STOP2: begin
                    sda_lo <= 1'b0;                    // SDA 上升沿 = STOP
                    //-----------------------------------------------------
                    // 一个阶段结束，切下一个阶段
                    //-----------------------------------------------------
                    case (phase)
                    P_INIT: begin
                        phase  <= P_WIN;
                        bi     <= 11'd0;
                    end
                    P_WIN: begin
                        // 窗口设好，请显示模块重画一帧，然后送显存
                        phase     <= P_REND;
                        frame_req <= 1'b1;
                        raddr     <= 10'd0;
                        bi        <= 11'd0;
                    end
                    default: begin                      // P_DATA
                        phase <= P_WIN;                 // 每帧重设窗口
                        bi    <= 11'd0;
                    end
                    endcase
                    istate <= I_IDLE;
                end
                //---------------------------------------------------------
                default: istate <= I_IDLE;
                endcase
            end
        end
    end

    assign busy = (istate != I_IDLE) || (phase == P_REND);

endmodule
