// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: traj_ramp
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 定点（位置目标）累加器 + 限速斜坡发生器
//           1. 维护定点 x_settle，由归零命令或任意数值增量改变
//           2. 单拍命令脉冲先锁存、等 tick 消费，避免 3ms 采样窗口漏掉命令
//           3. x_cmd 每个 tick 最多走 SLEW 一步，逼近 "定点 + 正弦" 的目标
//           4. run_en=0 时 x_cmd 跟随 x_pos，重新投入时 err_x = 0
// =============================================================================

`timescale 1ns/1ns

module traj_ramp #(
    parameter integer W            = 16,        // x_pos / x_cmd / x_settle 位宽(有符号)
    parameter signed [W-1:0] SLEW  = 16'sd2,    // 每控制周期最大步进（限速）
    //-------------------------------------------------------------------------
    // TURN_LIMIT：定点相对"复位零点"的行程上限，单位 = 编码器 counts。
    //   32767 时该限幅永不生效（16bit 有符号满量程），即"不限行程"。
    //   取有限值而不是 32767 是出于机械安全：若摆臂上有导线或软限位，
    //   转太多圈有拉断/撞限位的风险；有滑环、能连续多圈转的机型
    //   改成 32767 即可放开。这个限幅是"安全阀"，不是工作范围。
    //-------------------------------------------------------------------------
    parameter signed [W-1:0] TURN_LIMIT = 16'sd3201,
    //-------------------------------------------------------------------------
    // SETTLE_INIT：复位时预置的定点值（编码器 counts）。
    //   默认 0 = "复位时摆臂所在位置就是定点"：pos_rel 在复位瞬间也是 0，
    //   所以 x_settle = 0 正好等于"停在复位时的位置"。
    //   只在 x_cmd 初值上体现：x_cmd 复位为 SETTLE_INIT，在 run_en=0 期间
    //   又会跟随 x_pos，所以只有"上电即投入外环"这一条路径会用到它。
    //-------------------------------------------------------------------------
    parameter signed [W-1:0] SETTLE_INIT = 16'sd0
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     tick,        // 控制节拍（3ms）
    input  wire                     run_en,      // 1 = 外环投入（平衡/定点模式）
    // ---- 目标操作（均为 1 拍脉冲）----
    input  wire                     zero_req,    // 把"当前位置"设为定点零点
    input  wire signed [W-1:0]      delta_ext,   // 任意数值增量（来自 pos_cmd）
    input  wire                     delta_ext_valid,
    input  wire signed [W-1:0]      x_mod,       // 正弦调制量
    // ---- 位置反馈 ----
    input  wire signed [W-1:0]      x_pos,       // 摆臂当前位置
    // ---- 输出 ----
    output reg  signed [W-1:0]      x_cmd,       // 限速后的参考（送 pos_ctrl）
    output reg  signed [W-1:0]      x_settle,    // 定点目标（送遥测）
    output wire                     busy         // 1 = 斜坡仍在逼近定点
);

    //-------------------------------------------------------------------------
    // ① 定点累加器
    //   步进量先扩到 W+1 位有符号，累加结果也在 W+1 位上算，判饱和后取
    //   低 W 位 —— 与 pid_tuner 的 f_sat 手法一致，保证"连按多次"不会回绕
    //   成反向的大目标。
    //-------------------------------------------------------------------------
    wire signed [W:0] delta_ext_ext = $signed({delta_ext[W-1], delta_ext});
    wire              any_cmd       = zero_req | delta_ext_valid;

    //-------------------------------------------------------------------------
    // ①-b 单拍命令必须先锁存，等下一个 tick 消费
    //   命令是 1 拍脉冲（50MHz 下只有 20ns），而本模块只在 tick（3ms）那一拍
    //   采样，直接采样的命中概率约 0.6%，等效于"按下去没反应"。
    //   因此命令到达的那一拍就把【步进量本身】锁存下来，等下一个 tick 消费。
    //
    //   关键：必须锁 delta，不能只锁"有没有命令"。归零命令要的是 x_pos
    //   （持续量，随取随有）；增量命令要的是 delta_ext（脉冲量，过期即无）。
    //
    //   优先级：zero_req > 增量（一次 tick 只做一件事，行为确定）。
    //   按键抖动/连按 20ms 一颗，远大于 3ms 一拍，不会漏、也不会叠加。
    //-------------------------------------------------------------------------
    reg signed [W:0] cmd_delta;    // 锁存下来的步进量（tick 那拍用它，不看输入脚）
    reg              cmd_pending;  // 有命令待执行

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_pending <= 1'b0;
            cmd_delta   <= {(W+1){1'b0}};
        end
        else if (tick) begin
            // tick 那拍把 pending 交给累加器，并清掉（每个 tick 最多执行一条）
            cmd_pending <= 1'b0;
            cmd_delta   <= {(W+1){1'b0}};
        end
        else if (!cmd_pending && any_cmd) begin
            cmd_pending <= 1'b1;
            // 这里用的是【本拍】的 delta_ext —— 也就是脉冲仍在的那一拍，
            // 能取到正确值。zero_req 优先于增量（归零语义最强）。
            cmd_delta   <= zero_req ? {(W+1){1'b0}} : delta_ext_ext;
        end
    end

    // 本拍要执行的步进量（tick 拍由锁存值给出，其余拍为 0）
    wire signed [W:0] delta_now = (tick && cmd_pending) ? cmd_delta : {(W+1){1'b0}};

    wire signed [W:0] settle_pre =
        $signed({x_settle[W-1], x_settle}) + delta_now;

    wire signed [W:0] lim_hi = $signed({TURN_LIMIT[W-1], TURN_LIMIT});
    wire signed [W:0] lim_lo = -lim_hi;

    wire signed [W-1:0] settle_next =
        (settle_pre >  lim_hi) ?  TURN_LIMIT :
        (settle_pre <  lim_lo) ? -TURN_LIMIT :
                                  settle_pre[W-1:0];

    //-------------------------------------------------------------------------
    // ①-c 定点累加器本体：每个 tick 最多执行一条命令
    //   cmd_delta == 0  -> 归零命令（把零点定义在当前物理位置）
    //   cmd_delta != 0  -> 增量命令（用锁存下来的步进量，不看输入脚）
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            x_settle <= SETTLE_INIT;
        else if (tick && cmd_pending) begin
            if (cmd_delta == {(W+1){1'b0}})
                // 把"定点零点"定义在摆臂当前物理位置上。
                //   注意不是把编码器清零 —— qep_encoder 的 pos_rel 在复位时
                //   已经锚定在 POS_INIT，这里只是移动**目标**的参考原点。
                //   两者效果等价（err_x = x_cmd - x_pos 只用到差值），
                //   但不动编码器状态更安全：不会打断角速度差分的历史。
                x_settle <= x_pos;
            else
                x_settle <= settle_next;
        end
    end

    //-------------------------------------------------------------------------
    // ①-d 命令执行观测寄存器（常驻 RTL，占用极小）
    //   把"命令是否到达 / 是否被锁存 / 是否被执行"三件事分别记录下来，
    //   在板级观测时用来区分"命令根本没到"、"到了但锁存丢失"、
    //   "锁存了但算式输入过期"三种情形。
    //-------------------------------------------------------------------------
    reg        dbg_hit;         // 至少执行过一次命令
    reg        dbg_was_zero;    // 最后一次执行的是归零命令
    reg        dbg_zr_seen;     // zero_req 至少出现过一次
    reg signed [W:0] dbg_delta; // 最后一次执行用的锁存步进量
    reg signed [W:0] dbg_pre;   // 最后一次算出的 settle_pre
    reg signed [W-1:0] dbg_nxt; // 最后一次算出的 settle_next
    // 计数：把"命令事件"与"命令执行"量化出来
    reg [15:0] cnt_latch;       // 锁存过几次命令
    reg [15:0] cnt_exec;        // 执行过几次
    reg [15:0] cnt_delta_nz;    // 其中锁存步进量【真不为 0】的次数

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_hit        <= 1'b0;
            dbg_was_zero   <= 1'b0;
            dbg_zr_seen    <= 1'b0;
            dbg_delta      <= {(W+1){1'b0}};
            dbg_pre        <= {(W+1){1'b0}};
            dbg_nxt        <= {W{1'b0}};
            cnt_latch      <= 16'd0;
            cnt_exec       <= 16'd0;
            cnt_delta_nz   <= 16'd0;
        end
        else begin
            if (zero_req) dbg_zr_seen <= 1'b1;
            if (!tick && !cmd_pending && any_cmd)
                cnt_latch <= cnt_latch + 16'd1;
            if (tick && cmd_pending) begin
                dbg_hit      <= 1'b1;
                dbg_was_zero <= (cmd_delta == {(W+1){1'b0}});
                dbg_delta    <= cmd_delta;
                dbg_pre      <= settle_pre;
                dbg_nxt      <= settle_next;
                cnt_exec     <= cnt_exec + 16'd1;
                if (cmd_delta != {(W+1){1'b0}})
                    cnt_delta_nz <= cnt_delta_nz + 16'd1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // ② 限速斜坡：目标 = 定点 + 正弦调制量，x_cmd 每个 tick 最多走 SLEW 一步
    //-------------------------------------------------------------------------
    wire signed [W:0] x_tgt_ext =
        $signed({x_settle[W-1], x_settle}) + $signed({x_mod[W-1], x_mod});

    wire signed [W:0] ramp_diff =
        x_tgt_ext - $signed({x_cmd[W-1], x_cmd});

    // 用 W+1 位做差再比较，避免两个 16bit 之差在 16bit 里回绕
    //   （例如 x_settle=+30000、x_cmd=-30000，差 60000 装不进 16bit 有符号）
    wire signed [W:0] slew_ext = $signed({SLEW[W-1], SLEW});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            x_cmd <= SETTLE_INIT;
        else if (!tick) begin
            // 保持
        end
        else if (!run_en)
            // 外环未投入：参考直接跟随实际位置，保证重新投入时 err_x = 0，
            // 不会因为"参考停在旧目标"而产生一个阶跃冲击。
            x_cmd <= x_pos;
        else if (ramp_diff > slew_ext)
            x_cmd <= x_cmd + SLEW;
        else if (ramp_diff < -slew_ext)
            x_cmd <= x_cmd - SLEW;
        else
            x_cmd <= x_tgt_ext[W-1:0];        // 目标 = 定点 + 正弦调制量
    end

    //-------------------------------------------------------------------------
    // ③ busy：斜坡是否还在逼近（供 LED / 遥测观察）
    //-------------------------------------------------------------------------
    assign busy = (ramp_diff > slew_ext) || (ramp_diff < -slew_ext);


endmodule
