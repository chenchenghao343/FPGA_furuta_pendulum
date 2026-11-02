//=============================================================================
// 文件名  : Balance_Ctrl.sdc
// 项目    : J280 旋转倒立摆 FPGA 控制系统
// 器件    : GW2A-LV55PG484C8/I7
// 工具    : Gowin V1.9.12.x  （SUG940 Gowin 设计时序约束用户指南）
//-----------------------------------------------------------------------------
// 设计时序特征（决定了约束只需要很少几条）
//
//  1. 单时钟域：整个设计只有 clk_50m(M19) 一个时钟，50MHz。
//     ★ 不使用 PLL（赛题硬性要求"不用 PLL IP"），因此没有派生时钟、
//       没有时钟域交叉、不需要 set_false_path 处理 CDC。
//  2. 所有寄存器都由 posedge clk_50m 驱动，复位 rst_n(AB3) 为异步低有效，
//     在 clk_mgr 内部做同步释放。
//  3. 低速异步输入（按键、编码器 A/B、串口 rx、ADC 并行数据）都已在
//     各自模块内做了 2~3 级同步打拍，属于"有意跨时钟域"，无需额外约束。
//  4. 输出（PWM/方向/串口 TX/ADC 时钟/参考点）都是纯寄存器或简单组合，
//     无外部建立/保持要求，因此不加 set_output_delay。
//
// 结论：本 SDC 只需一条主时钟约束 + 少数几条异步输入例外。
//-----------------------------------------------------------------------------

//---------------------------------------------------------------------------
// 主时钟：clk_50m = 50MHz，周期 20ns，上升沿在 0ns，占空比 50%
//   ★ 这是整个设计唯一的时序基准。gw_sh / 综合器的所有路径分析都以它为起点。
//---------------------------------------------------------------------------
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]

//---------------------------------------------------------------------------
// 异步输入例外
//   按键、编码器 A/B、串口 RX 是相对 50MHz 完全异步的慢速信号，
//   模块内部已用多级触发器同步。对"第一级同步触发器之前"的路径不做
//   时序分析，否则工具会因无法知道外部到达时间而报无意义的违例。
//---------------------------------------------------------------------------
set_false_path -from [get_ports {key_sw[0]}] -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {key_sw[1]}] -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {rst_n}]     -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {uart_rx}]   -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {enc2_a}]    -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {enc2_b}]    -to [get_clocks {clk_50m}]

// ADC 并行数据：3PA1030 由 adc_clk(25MHz, 由 FPGA 输出) 驱动输出，
// 数据在 25MHz 边界变化，而 FPGA 在 50MHz 上采样。
// 模块内部对同一拍做了 16 点块平均（v4.1 起改为与官方 adc.v 一致的做法，
// 见 angle_adc.v；不再是旧的 65536 点累加平均），对单点采样时刻不敏感，
// 因此这条路径不作为关键时序路径分析，只保证建立时间由平均器吸收。
set_false_path -from [get_ports {adc_data_in[0]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[1]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[2]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[3]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[4]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[5]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[6]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[7]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[8]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[9]}]  -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {key_sw[2]}] -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {key_sw[3]}] -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {enc_a}]     -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {enc_b}]     -to [get_clocks {clk_50m}]

//---------------------------------------------------------------------------
// 说明：为什么不加 max_delay / multicycle
//   - 流水线 PID 的 5 级之间是纯寄存器到寄存器路径，工具按 20ns 正常分析。
//     实测 32bit×16bit 乘法（GAIN_W=32 × ERR_W=16，乘积 PROD_W=48）+ 56bit
//     加法/求和（ACC_W / SUM_W = 56，含积分累加器）在 GW2A-55 上远小于 20ns，
//     不需要 multicycle 放宽（放宽反而会掩盖真实的时序问题）。
//   - 串口 9600 的分频计数器是最慢的逻辑之一，但只是比较器链，无风险。
//---------------------------------------------------------------------------
