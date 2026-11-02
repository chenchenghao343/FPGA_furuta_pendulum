// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: uart_link
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 遥测数据帧打包与发送模块
//           1. 把 12 个内部观测量打包成固定 28 字节帧：AA 55 + 24 字节数据 + 0D 0A
//           2. 帧头帧尾双校验，接收端可滑动同步，容忍丢字节与错位
//           3. 发送节流由 BYTE_GAP / FRAME_GAP 控制，保证 9600bps 下不溢出
//           4. 帧首一次性快照，发帧期间输入变化不会撕裂同一帧内的数据
// =============================================================================

module uart_link #(
    parameter CLK_FREQ   = 26'd50_000_000,
    parameter UART_BPS   = 21'd9600,
    parameter BYTE_GAP   = 20'd46_876,     // 字节间最小间隔，约 9 个位时间(≈0.94ms)
    parameter FRAME_GAP  = 20'd999_999     // 两帧之间最小间隔(~20ms)
)(
    input  wire        clk,
    input  wire        rst_n,
    // 物理串口
    input  wire        uart_rx_pin,     // L20
    output wire        uart_tx_pin,     // K20
    // 遥测数据源
    input  wire        telemetry_en,    // 1 = 持续上报
    input  wire [15:0] w0_angle,
    input  wire [15:0] w1_pos,
    input  wire [15:0] w2_mode,
    // 送进电机的控制量（原始 16bit，客户端按有符号解析）
    input  wire [15:0] w3_umotor,
    // 控制环内部量：让上位机区分 P 项、D 项与位置环各自的贡献
    input  wire [15:0] w4_phieff,    // 内环角度偏差 = phi - phi_bias
    input  wire [15:0] w5_dphi,      // 摆杆角速度（本周期角度增量，未放大）
    input  wire [15:0] w6_pterm,     // P 项（输出单位）
    input  wire [15:0] w7_dterm,     // D 项（输出单位）
    input  wire [15:0] w8_pdsum,     // PD 相加（饱和前）
    input  wire [15:0] w9_xset,      // 定点目标
    input  wire [15:0] w10_errx,     // 位置偏差
    input  wire [15:0] w11_phibias,  // 倾角偏置（外环输出）
    output wire        frame_active,    // 正在发送一帧
    // 给 顶层 的命令字节流
    output wire [7:0]  cmd_data,
    output wire        cmd_valid,
    // 状态
    output wire        rx_frame_err
);

    //-------------------------------------------------------------------------
    // 例化字节发送器
    //-------------------------------------------------------------------------
    wire [7:0] tx_byte;
    wire       tx_kick;      // 启动一字节发送的 1 拍脉冲
    wire       tx_done;      // 一字节发完的 1 拍脉冲
    wire       tx_busy;      // 1 = 正在发送

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .UART_BPS (UART_BPS)
    ) u_uart_tx (
        .clk     (clk),
        .rst_n   (rst_n),
        .pi_data (tx_byte),
        .pi_flag (tx_kick),
        .tx      (uart_tx_pin),
        .pi_done (tx_done)
    );

    //-------------------------------------------------------------------------
    // 例化字节接收器(收到的字节直接透传给 顶层)
    //-------------------------------------------------------------------------
    wire [7:0] rx_byte;
    wire       rx_valid;

    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .UART_BPS (UART_BPS)
    ) u_uart_rx (
        .clk       (clk),
        .rst_n     (rst_n),
        .rx        (uart_rx_pin),
        .po_data   (rx_byte),
        .po_flag   (rx_valid),
        .frame_err (rx_frame_err)
    );

    assign cmd_data  = rx_byte;
    assign cmd_valid = rx_valid;

    //-------------------------------------------------------------------------
    // 一帧 28 字节的快照寄存器：帧首统一锁存，发帧期间数据保持稳定
    //-------------------------------------------------------------------------
    reg [15:0] w0_snap;
    reg [15:0] w1_snap;
    reg [15:0] w2_snap;
    reg [15:0] w3_snap;   // 电机控制量
    reg [15:0] w4_snap, w5_snap, w6_snap, w7_snap, w8_snap;   // phi_eff / dphi / P / D / PD 和
    reg [15:0] w9_snap, w10_snap, w11_snap;                   // 定点目标 / 位置偏差 / 倾角偏置

    //-------------------------------------------------------------------------
    // 发送状态机
    //-------------------------------------------------------------------------
    localparam [1:0] ST_IDLE = 2'd0;   // 空闲:等待可发帧
    localparam [1:0] ST_BYTE = 2'd1;   // 等待当前字节发完
    localparam [1:0] ST_GAP  = 2'd2;   // 字节间空隙
    localparam [1:0] ST_FGAP = 2'd3;   // 帧间空隙

    //-------------------------------------------------------------------------
    // 帧尾字节下标，单独定义成一个常量
    //-------------------------------------------------------------------------
    localparam [4:0] FRAME_LAST = 5'd27;   // 帧长 28 字节 -> 末下标 27

    reg [1:0]  state;
    reg [4:0]  byte_idx;    // 帧内字节序号 0..27（5bit 足够跨过 28 个字节）
    reg [19:0] gap_cnt;     // 空隙计数器

    // busy 由本地状态机自行跟踪(与 uart_tx 内部一致,避免额外端口)
    assign tx_busy = (state != ST_IDLE);

    reg frame_busy;         // 1 = 正在发送一帧(帧首..帧尾)

    assign frame_active = frame_busy;

    //-------------------------------------------------------------------------
    // 帧内字节选择(组合)
    //-------------------------------------------------------------------------
    reg [7:0] byte_sel;
    always @* begin
        case (byte_idx)
            5'd0:    byte_sel = 8'hAA;            // 帧头
            5'd1:    byte_sel = 8'h55;            // 帧头
            5'd2:    byte_sel = w0_snap[15:8];
            5'd3:    byte_sel = w0_snap[7:0];
            5'd4:    byte_sel = w1_snap[15:8];
            5'd5:    byte_sel = w1_snap[7:0];
            5'd6:    byte_sel = w2_snap[15:8];
            5'd7:    byte_sel = w2_snap[7:0];
            5'd8:    byte_sel = w3_snap[15:8];
            5'd9:    byte_sel = w3_snap[7:0];
            // byte8/9=u, 10/11=phi_eff, 12/13=dphi,
            5'd10:   byte_sel = w4_snap[15:8];
            5'd11:   byte_sel = w4_snap[7:0];
            5'd12:   byte_sel = w5_snap[15:8];
            5'd13:   byte_sel = w5_snap[7:0];
            5'd14:   byte_sel = w6_snap[15:8];
            5'd15:   byte_sel = w6_snap[7:0];
            5'd16:   byte_sel = w7_snap[15:8];
            5'd17:   byte_sel = w7_snap[7:0];
            5'd18:   byte_sel = w8_snap[15:8];
            5'd19:   byte_sel = w8_snap[7:0];
            // byte20..25 = x_settle / err_x / phi_bias，CR LF 顺延到 26/27
            5'd20:   byte_sel = w9_snap[15:8];
            5'd21:   byte_sel = w9_snap[7:0];
            5'd22:   byte_sel = w10_snap[15:8];
            5'd23:   byte_sel = w10_snap[7:0];
            5'd24:   byte_sel = w11_snap[15:8];
            5'd25:   byte_sel = w11_snap[7:0];
            5'd26:   byte_sel = 8'h0D;            // CR
            5'd27:   byte_sel = 8'h0A;            // LF
            default: byte_sel = 8'h00;
        endcase
    end

    // 待发送字节:始终驱动 uart_tx 的 pi_data;在 tx_kick 为高那一拍保持稳定
    assign tx_byte = byte_sel;

    //-------------------------------------------------------------------------
    // 主状态机
    //-------------------------------------------------------------------------
    reg kick_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            byte_idx   <= 5'd0;
            gap_cnt    <= 20'd0;
            frame_busy <= 1'b0;
            kick_r     <= 1'b0;
            w0_snap    <= 16'd0;
            w1_snap    <= 16'd0;
            w2_snap    <= 16'd0;
            w3_snap    <= 16'd0;
            w4_snap <= 16'd0; w5_snap <= 16'd0; w6_snap <= 16'd0;
            w7_snap <= 16'd0; w8_snap <= 16'd0;
            w9_snap <= 16'd0; w10_snap <= 16'd0; w11_snap <= 16'd0;   // 位置环三路
        end else begin
            kick_r <= 1'b0;      // 默认:kick 只维持 1 拍

            case (state)
                //-------------------------------------------------------------------------
                // 空闲:telemetry_en 置位则启动一帧(先快照)
                //-------------------------------------------------------------------------
                ST_IDLE: begin
                    frame_busy <= 1'b0;
                    if (telemetry_en) begin
                        w0_snap    <= w0_angle;   // 帧首一次性快照
                        w1_snap    <= w1_pos;
                        w2_snap    <= w2_mode;
            w3_snap    <= w3_umotor;   // 电机控制量
            w4_snap <= w4_phieff; w5_snap <= w5_dphi; w6_snap <= w6_pterm;
            w7_snap <= w7_dterm;  w8_snap <= w8_pdsum;
                        w9_snap  <= w9_xset;      // 定点目标
                        w10_snap <= w10_errx;
                        w11_snap <= w11_phibias;
                        byte_idx   <= 5'd0;
                        frame_busy <= 1'b1;
                        kick_r     <= 1'b1;       // 启动 byte0
                        state      <= ST_BYTE;
                    end
                end
                //-------------------------------------------------------------------------
                // 正在发一个字节:等 pi_done
                //-------------------------------------------------------------------------
                ST_BYTE: begin
                    if (tx_done) begin
                        if (!telemetry_en) begin
                            // 运行中被关掉:本字节发完立刻停,不补发剩余字节
                            frame_busy <= 1'b0;
                            byte_idx   <= 5'd0;
                            state      <= ST_IDLE;
                        end else if (byte_idx == FRAME_LAST) begin
                            // 帧尾(0x0A)发完 -> 进帧间空隙
                            frame_busy <= 1'b0;
                            byte_idx   <= 5'd0;
                            gap_cnt    <= FRAME_GAP[19:0];
                            state      <= ST_FGAP;
                        end else begin
                            byte_idx <= byte_idx + 4'd1;
                            gap_cnt  <= BYTE_GAP[19:0];
                            state    <= ST_GAP;
                        end
                    end
                end
                //-------------------------------------------------------------------------
                // 字节间空隙:数满后发下一个字节
                //-------------------------------------------------------------------------
                ST_GAP: begin
                    if (!telemetry_en) begin
                        frame_busy <= 1'b0;
                        byte_idx   <= 5'd0;
                        state      <= ST_IDLE;
                    end else if (gap_cnt == 20'd0) begin
                        kick_r <= 1'b1;            // 启动下一字节
                        state  <= ST_BYTE;
                    end else begin
                        gap_cnt <= gap_cnt - 20'd1;
                    end
                end
                //-------------------------------------------------------------------------
                // 帧间空隙
                //-------------------------------------------------------------------------
                ST_FGAP: begin
                    if (!telemetry_en) begin
                        state <= ST_IDLE;
                    end else if (gap_cnt == 20'd0) begin
                        state <= ST_IDLE;          // 回 IDLE 重新快照并开帧
                    end else begin
                        gap_cnt <= gap_cnt - 20'd1;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    assign tx_kick = kick_r;

endmodule
