// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: traj_sine
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 固定正弦轨迹发生器（平衡状态下的往复摆动激励）
//           1. 24 位相位累加器 + 128 点 1/4 波查表 + 象限折叠，等效 512 点分辨率
//           2. 幅值与频率为编译期常量，频率不做现场可调
//           3. 输出只作为斜坡目标的叠加项，限速斜坡天然生效
//           4. 进入时相位清零，退出只允许在过零点，两侧都无阶跃
// =============================================================================

`timescale 1ns / 1ns

module traj_sine #(
    parameter integer W         = 16,
    //-------------------------------------------------------------------------
    // 波形参数（编译期常量，运行时不可调）
    //-------------------------------------------------------------------------
    parameter integer AMP       = 200,          // 正弦幅值（编码器 counts，单边）
    parameter [23:0]  PHASE_INC = 24'd10066     // 每 3ms 的相位增量（24bit = 一整周）
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     tick,       // 控制节拍 3ms
    input  wire                     run_en,     // 1 = BALANCE（只有这时才允许正弦）
    input  wire                     key_pulse,  // 板载 SW2 按下：切换正弦开关
    output reg  signed [W-1:0]      x_mod,      // 正弦调制量（叠加到 traj_ramp 的目标）
    output wire                     sine_on     // 1 = 正弦模式生效中
);

    //-------------------------------------------------------------------------
    // 相位累加器（24bit = 一整周）
    //-------------------------------------------------------------------------
    reg  [23:0] phase;
    wire [24:0] phase_sum  = {1'b0, phase} + {1'b0, PHASE_INC};
    wire        phase_wrap = phase_sum[24];          // 这一拍相位回绕（=过零点）

    //-------------------------------------------------------------------------
    // 开关状态机
    //-------------------------------------------------------------------------
    reg on_req;
    reg sine_act;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            on_req <= 1'b0;
        else if (!run_en)
            on_req <= 1'b0;                  // 离开 BALANCE 强制关闭
        else if (key_pulse)
            on_req <= ~on_req;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sine_act <= 1'b0;
        else if (!run_en)
            sine_act <= 1'b0;                // 非 BALANCE 立即关（此时 run_pos=0，参考跟随实际位置，无冲击）
        else if (on_req)
            sine_act <= 1'b1;
        else if (phase_wrap)
            sine_act <= 1'b0;                // 关闭只发生在过零点
    end

    assign sine_on = sine_act;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            phase <= 24'd0;
        else if (!run_en || !sine_act)
            phase <= 24'd0;                  // 关掉就归零，下次从 sin(0)=0 开始
        else if (tick)
            phase <= phase_sum[23:0];
    end

    //-------------------------------------------------------------------------
    // 象限折叠：取高 2 位定象限，次 7 位查 1/4 波表
    //-------------------------------------------------------------------------
    wire [1:0] quadrant = phase[23:22];
    wire [6:0] idx_raw  = phase[21:15];
    wire       mirror   = quadrant[0];
    wire       neg      = quadrant[1];
    wire [6:0] idx      = mirror ? (7'd127 - idx_raw) : idx_raw;

    //-------------------------------------------------------------------------
    // 1/4 波正弦表：128 点，Q10（1.0 = 1024），角度 = a/128 * 90 度
    //-------------------------------------------------------------------------
    reg signed [11:0] tbl_q;
    always @(*) begin
        case (idx)
                7'd0  : tbl_q = 12'sd0    ;  7'd1  : tbl_q = 12'sd13   ;  7'd2  : tbl_q = 12'sd25   ;  7'd3  : tbl_q = 12'sd38;
                7'd4  : tbl_q = 12'sd50   ;  7'd5  : tbl_q = 12'sd63   ;  7'd6  : tbl_q = 12'sd75   ;  7'd7  : tbl_q = 12'sd88;
                7'd8  : tbl_q = 12'sd100  ;  7'd9  : tbl_q = 12'sd113  ;  7'd10 : tbl_q = 12'sd125  ;  7'd11 : tbl_q = 12'sd138;
                7'd12 : tbl_q = 12'sd150  ;  7'd13 : tbl_q = 12'sd163  ;  7'd14 : tbl_q = 12'sd175  ;  7'd15 : tbl_q = 12'sd187;
                7'd16 : tbl_q = 12'sd200  ;  7'd17 : tbl_q = 12'sd212  ;  7'd18 : tbl_q = 12'sd224  ;  7'd19 : tbl_q = 12'sd237;
                7'd20 : tbl_q = 12'sd249  ;  7'd21 : tbl_q = 12'sd261  ;  7'd22 : tbl_q = 12'sd273  ;  7'd23 : tbl_q = 12'sd285;
                7'd24 : tbl_q = 12'sd297  ;  7'd25 : tbl_q = 12'sd309  ;  7'd26 : tbl_q = 12'sd321  ;  7'd27 : tbl_q = 12'sd333;
                7'd28 : tbl_q = 12'sd345  ;  7'd29 : tbl_q = 12'sd357  ;  7'd30 : tbl_q = 12'sd369  ;  7'd31 : tbl_q = 12'sd380;
                7'd32 : tbl_q = 12'sd392  ;  7'd33 : tbl_q = 12'sd403  ;  7'd34 : tbl_q = 12'sd415  ;  7'd35 : tbl_q = 12'sd426;
                7'd36 : tbl_q = 12'sd438  ;  7'd37 : tbl_q = 12'sd449  ;  7'd38 : tbl_q = 12'sd460  ;  7'd39 : tbl_q = 12'sd472;
                7'd40 : tbl_q = 12'sd483  ;  7'd41 : tbl_q = 12'sd494  ;  7'd42 : tbl_q = 12'sd505  ;  7'd43 : tbl_q = 12'sd516;
                7'd44 : tbl_q = 12'sd526  ;  7'd45 : tbl_q = 12'sd537  ;  7'd46 : tbl_q = 12'sd548  ;  7'd47 : tbl_q = 12'sd558;
                7'd48 : tbl_q = 12'sd569  ;  7'd49 : tbl_q = 12'sd579  ;  7'd50 : tbl_q = 12'sd590  ;  7'd51 : tbl_q = 12'sd600;
                7'd52 : tbl_q = 12'sd610  ;  7'd53 : tbl_q = 12'sd620  ;  7'd54 : tbl_q = 12'sd630  ;  7'd55 : tbl_q = 12'sd640;
                7'd56 : tbl_q = 12'sd650  ;  7'd57 : tbl_q = 12'sd659  ;  7'd58 : tbl_q = 12'sd669  ;  7'd59 : tbl_q = 12'sd678;
                7'd60 : tbl_q = 12'sd688  ;  7'd61 : tbl_q = 12'sd697  ;  7'd62 : tbl_q = 12'sd706  ;  7'd63 : tbl_q = 12'sd715;
                7'd64 : tbl_q = 12'sd724  ;  7'd65 : tbl_q = 12'sd733  ;  7'd66 : tbl_q = 12'sd742  ;  7'd67 : tbl_q = 12'sd750;
                7'd68 : tbl_q = 12'sd759  ;  7'd69 : tbl_q = 12'sd767  ;  7'd70 : tbl_q = 12'sd775  ;  7'd71 : tbl_q = 12'sd784;
                7'd72 : tbl_q = 12'sd792  ;  7'd73 : tbl_q = 12'sd799  ;  7'd74 : tbl_q = 12'sd807  ;  7'd75 : tbl_q = 12'sd815;
                7'd76 : tbl_q = 12'sd822  ;  7'd77 : tbl_q = 12'sd830  ;  7'd78 : tbl_q = 12'sd837  ;  7'd79 : tbl_q = 12'sd844;
                7'd80 : tbl_q = 12'sd851  ;  7'd81 : tbl_q = 12'sd858  ;  7'd82 : tbl_q = 12'sd865  ;  7'd83 : tbl_q = 12'sd872;
                7'd84 : tbl_q = 12'sd878  ;  7'd85 : tbl_q = 12'sd885  ;  7'd86 : tbl_q = 12'sd891  ;  7'd87 : tbl_q = 12'sd897;
                7'd88 : tbl_q = 12'sd903  ;  7'd89 : tbl_q = 12'sd909  ;  7'd90 : tbl_q = 12'sd915  ;  7'd91 : tbl_q = 12'sd920;
                7'd92 : tbl_q = 12'sd926  ;  7'd93 : tbl_q = 12'sd931  ;  7'd94 : tbl_q = 12'sd936  ;  7'd95 : tbl_q = 12'sd941;
                7'd96 : tbl_q = 12'sd946  ;  7'd97 : tbl_q = 12'sd951  ;  7'd98 : tbl_q = 12'sd955  ;  7'd99 : tbl_q = 12'sd960;
                7'd100: tbl_q = 12'sd964  ;  7'd101: tbl_q = 12'sd968  ;  7'd102: tbl_q = 12'sd972  ;  7'd103: tbl_q = 12'sd976;
                7'd104: tbl_q = 12'sd980  ;  7'd105: tbl_q = 12'sd983  ;  7'd106: tbl_q = 12'sd987  ;  7'd107: tbl_q = 12'sd990;
                7'd108: tbl_q = 12'sd993  ;  7'd109: tbl_q = 12'sd996  ;  7'd110: tbl_q = 12'sd999  ;  7'd111: tbl_q = 12'sd1002;
                7'd112: tbl_q = 12'sd1004 ;  7'd113: tbl_q = 12'sd1007 ;  7'd114: tbl_q = 12'sd1009 ;  7'd115: tbl_q = 12'sd1011;
                7'd116: tbl_q = 12'sd1013 ;  7'd117: tbl_q = 12'sd1015 ;  7'd118: tbl_q = 12'sd1016 ;  7'd119: tbl_q = 12'sd1018;
                7'd120: tbl_q = 12'sd1019 ;  7'd121: tbl_q = 12'sd1020 ;  7'd122: tbl_q = 12'sd1021 ;  7'd123: tbl_q = 12'sd1022;
                7'd124: tbl_q = 12'sd1023 ;  7'd125: tbl_q = 12'sd1023 ;  7'd126: tbl_q = 12'sd1024 ;  7'd127: tbl_q = 12'sd1024;
            default: tbl_q = 12'sd0;
        endcase
    end

    wire signed [11:0] val = neg ? (-tbl_q) : tbl_q;

    //-------------------------------------------------------------------------
    // 幅度缩放：Q10 表 x AMP / 1024（一次乘法，占 1 个 DSP）
    //-------------------------------------------------------------------------
    wire signed [27:0] prod = $signed(val) * $signed(AMP);
    // prod >>> 10 的低 16 位：显式取位，避免「28bit 赋给 16bit」的截断告警
    wire signed [15:0] x_mod_w = prod[25:10];      // 值域 |x_mod| <= AMP

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            x_mod <= {W{1'b0}};
        else if (!sine_act)
            x_mod <= {W{1'b0}};                    // 关闭立即归零（过零点处台阶 < 1 count）
        else if (tick)
            x_mod <= x_mod_w;
    end

endmodule
