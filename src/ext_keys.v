// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: ext_keys
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 5 路外部按键的消抖、"按键 -> 命令"译码与命令合流
//           1. 5 路按键各实例化一个 key_filter，只取确认按下时的单拍脉冲
//           2. run/inc/dec 直接引出为 1 拍脉冲，供顶层做启停与步进
//           3. mode/send 译成串口命令字节，与 uart_link 的命令总线按位或合并
//           4. 合流时按键优先：有按键命令就用按键字节，否则透传串口字节
// =============================================================================

`timescale 1ns/1ps

module ext_keys #(
    // 20ms @50MHz 的消抖计数上限，远大于机械按键的抖动时间
    parameter [19:0] CNT_MAX = 20'd1_000_000
)(
    input  wire        clk,
    input  wire        rst_n,        // 系统复位（低有效）

    input  wire [4:0]  key_in,       // 5 路按键，低有效

    // 串口命令总线（来自 uart_link）
    input  wire        cmd_valid,
    input  wire [7:0]  cmd_data,

    // 输出
    output wire        run_pulse,    // 启动/停止：1 拍
    output wire        inc,          // +100：1 拍
    output wire        dec,          // -100：1 拍
    output wire        cmd_valid_all,// 合并后的命令总线
    output wire [7:0]  cmd_data_all
);

    //=========================================================================
    // 5 路消抖
    //=========================================================================
    wire k_run, k_inc, k_dec, k_mode, k_send;

    key_filter #(.CNT_MAX (CNT_MAX)) u_k_run (
        .clk (clk), .rst_n (rst_n),
        .key_in (key_in[0]), .key_flag (k_run), .key_state ()
    );

    key_filter #(.CNT_MAX (CNT_MAX)) u_k_inc (
        .clk (clk), .rst_n (rst_n),
        .key_in (key_in[1]), .key_flag (k_inc), .key_state ()
    );

    key_filter #(.CNT_MAX (CNT_MAX)) u_k_dec (
        .clk (clk), .rst_n (rst_n),
        .key_in (key_in[2]), .key_flag (k_dec), .key_state ()
    );

    key_filter #(.CNT_MAX (CNT_MAX)) u_k_mode (
        .clk (clk), .rst_n (rst_n),
        .key_in (key_in[3]), .key_flag (k_mode), .key_state ()
    );

    key_filter #(.CNT_MAX (CNT_MAX)) u_k_send (
        .clk (clk), .rst_n (rst_n),
        .key_in (key_in[4]), .key_flag (k_send), .key_state ()
    );

    assign run_pulse = k_run;
    assign inc       = k_inc;
    assign dec       = k_dec;

    //=========================================================================
    // 命令注入与合流
    //   按键优先：ext_cmd_valid 为 1 时用按键字节，否则透传串口字节。
    //=========================================================================
    wire       ext_cmd_valid = k_mode | k_send;
    wire [7:0] ext_cmd_byte  = k_mode ? 8'h4D : 8'h21;   // 'M' : '!'

    assign cmd_valid_all = cmd_valid | ext_cmd_valid;
    assign cmd_data_all  = ext_cmd_valid ? ext_cmd_byte : cmd_data;

endmodule
