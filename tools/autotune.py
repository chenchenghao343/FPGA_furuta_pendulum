# -*- coding: utf-8 -*-
"""
autotune.py -- J280 开发板【可被脚本驱动的】记录与调参工具

================================================================================
★ 为什么要有这个文件
================================================================================
用户 2026-10-15 提出："我能不能连接开发板标定下垂和竖直后让你自己调"。

技术上可行：本工具就运行在插着开发板的那台机器上，用 pyserial 直接开串口，
既能发命令（'R'/'1'..'6'/'&gt;' '&lt;' '*' '/' '+' '-' /'!' ...），
也能收 28 字节遥测帧并算指标。

但"让脚本自己调"意味着【真电机会按脚本决定去动】，所以本文件的第一原则是安全：

  1) 默认只读：不指定 --allow-writes 时，脚本【一个字节都不发】，
     只监听、只解码、只打印。先把链路确认通，再谈发命令。
  2) 看门狗：一旦出现下列任一情况立即发 'R' 停机并退出，并打印原因：
       · 遥测 mode == 3（FAULT）
       · u_motor 连续 U_SAT_ABORT_N 帧打满（|u| >= U_SAT）
       · 连续 NO_DATA_ABORT 秒收不到任何字节
       · 键盘 Ctrl-C
  3) 退出时默认发 'R'（--no-stop 可关），保证不把电机留在运行态。
  4) 单次只录有限时长（默认 60 s），且是【进入 BALANCE 之后】才开始计时。
  5) ★ 脚本【无法通过串口复位系数】（那是 SW5 的硬件复位）。
     所以每一步施加的操作序列都要记下来，值由 GainTracker 精确跟踪，
     随时可以用 --goto 走回目标值。这一条很重要：调坏了要能走回来。

★ 复用而不是重抄
  解码器 FrameDecoder、系数模型 GainTracker、常量 CNT_PER_REV/POS_BASE 等
  全部 import 自 serial_host.py —— 本工程已被"两份实现不一致"坑过多次
  （上位机两份、flow 脚本六份），所以这里不再抄第二份解码器。

================================================================================
★ 术语：为什么用 --goto 而不是"直接设成某个值"
================================================================================
固件里改系数只有 6 个按键级操作（+64 / ×2 / ÷2 / ±12.5% / 取反），
没有"直接写一个数"的命令。所以要把 PD 变成某个目标值，只能找一条
【操作序列】走过去。本工具用 BFS 在固件那套精确整数算术上搜这条序列
（见 find_ops），搜到后逐字节发出去，并用 GainTracker 复核。

  例：PD 从 -4000 走到 -18000 的序列是  **>   （×2 ×2 再 +12.5%）
      -4000 -> -8000 -> -16000 -> -18000        精确命中

  搜不到（或步数超过 --max-ops）时会明确报错，不会"差不多就发"。

================================================================================
用法
================================================================================
  # 0) 看有哪些串口
  python autotune.py --list

  # 1) 只读监视（最安全的第一步：确认 9600 8N1 能收到帧）
  python autotune.py --port COM5

  # 2) 启动 -> 等平衡 -> 录 60 秒 -> 出指标 + CSV（需要 --allow-writes 才能发 'R'）
  python autotune.py --port COM5 --allow-writes --run --measure 60

  # 3) 把 PD 走到 -14000 再测（BFS 搜操作序列）
  python autotune.py --port COM5 --allow-writes --run --goto PD=-14000 --measure 60

  # 4) 按给定操作序列走（'&gt;' 每次 +12.5%）：
  python autotune.py --port COM5 --allow-writes --run --step PD=&gt;&gt;&gt;&gt; --measure 60

  # 5) 无硬件自检（在校验指标算法与 BFS 时用）
  python autotune.py --selftest

================================================================================
"""

import argparse
import os
import sys
import time

# ---- 复用 serial_host.py 的实现（不重抄解码器）----
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import serial_host as SH          # noqa: E402

FRAME_LEN = SH.FRAME_LEN
BAUD = SH.BAUD
MODE_NAME = SH.MODE_NAME
CNT_PER_REV = SH.CNT_PER_REV
GainTracker = SH.GainTracker

# ---------------------------------------------------------------------------
# 看门狗阈值
# ---------------------------------------------------------------------------
U_SAT = 4090            # 与 serial_host 面板的"★饱和"判据一致
U_SAT_ABORT_N = 60      # 连续打满多少帧就停机（约 4.6 s @13Hz）
NO_DATA_ABORT = 3.0     # 连续多少秒收不到字节就停机
# ★ 起摆阶段防外漂：
#   实测（2026-10-16）起摆的能量泵会把摆臂【持续往外推】：
#   一次把 pos_rel 从 +4321 推到 +7520，另一次从 10085 一路到 17520，
#   都撞上 POS_LIMIT=6000 触发 arm_fault 而中止。
#   所以进 BALANCE 之前，只要 |pos_rel| 超过这个值就主动停机，
#   不让它一路撞到限幅。只在【非 BALANCE】阶段判，因为定位运动时
#   摆臂本来就要走远。
DRIFT_ABORT = 5200

# 串口能直接改的 6 个系数对应哪几个选择键
SEL_KEY = {'KP': '1', 'KI': '2', 'KD': '3', 'PP': '4', 'PD': '5', 'SW': '6'}


# ===========================================================================
# 一、操作序列搜索（在固件的精确整数算术上做 BFS）
# ===========================================================================
# ★ 必须用 GainTracker 的那套算术，不能自己写一套"差不多"的浮点近似 ——
#   否则算出来的序列到了板上结果不一样，就成了"面板骗人"的翻版。
#   GainTracker.apply 内部与 pid_tuner.v 的 f_sat / 各 op 逐条对应。
def op_apply(v, op):
    """把单个操作施加到值上，语义与固件完全一致（借 GainTracker 的算术）。

    ★ 这里刻意【不】用 GainTracker 实例来算，而是复刻它的表达式 ——
      因为实例版会把结果写回 self.g[sel]，不适合当纯函数用。
      为保持一致，下面每一条都与 GainTracker.apply 里的写法逐字对应。
    """
    sat = GainTracker._sat
    if op == '>':
        return sat(v + (v >> 3))
    if op == '<':
        return sat(v - (v >> 3))
    if op == '*':
        return sat(v << 1)
    if op == '/':
        return sat(v >> 1)
    if op == '+':
        return sat(v + 64)
    if op == '-':
        return sat(-v)
    raise ValueError("未知操作 %r" % op)


OPS = ('>', '<', '*', '/', '+', '-')


def find_ops(start, target, max_ops=12, max_nodes=300000):
    """BFS 搜一条从 start 到 target 的操作序列（对着固件的精确算术）。

    返回 (序列字符串, 结束值)；搜不到返回 (None, None)。
    ★ max_nodes 是防御性的：可达值集合可能很大，不能无限扩。
      超过上限就明确失败，不返回"近似的"序列。
    """
    if start == target:
        return "", start
    seen = {start: ""}
    frontier = [start]
    for _depth in range(max_ops):
        nxt = []
        for v in frontier:
            base = seen[v]
            for op in OPS:
                w = op_apply(v, op)
                if w in seen:
                    continue
                seq = base + op
                if w == target:
                    return seq, w
                seen[w] = seq
                nxt.append(w)
                if len(seen) > max_nodes:
                    return None, None
        frontier = nxt
        if not frontier:
            break
    return None, None


# ===========================================================================
# 二、指标计算（只用 BALANCE 段，避免起摆段污染统计）
# ===========================================================================
def _mean(xs):
    return sum(xs) / float(len(xs)) if xs else 0.0


def _mode_hist(frames):
    """各 mode 出现次数（监视 IDLE 段时用来说明'为什么没有指标'）。"""
    h = {}
    for _, f in frames:
        h[f[2]] = h.get(f[2], 0) + 1
    return h


def _std(xs):
    if len(xs) < 2:
        return 0.0
    m = _mean(xs)
    return (sum((x - m) ** 2 for x in xs) / float(len(xs) - 1)) ** 0.5


def _slope(ts, ys):
    """最小二乘斜率（单位/秒）。点数不足返回 0。"""
    n = len(ts)
    if n < 3:
        return 0.0
    mt, my = _mean(ts), _mean(ys)
    den = sum((t - mt) ** 2 for t in ts)
    if den == 0:
        return 0.0
    return sum((ts[i] - mt) * (ys[i] - my) for i in range(n)) / den


def metrics(frames, settle=15.0):
    """frames: [(t, (w0,w1,w2,u,phieff,dphi,pterm,dterm,pdsum,xset,errx,phib)), ...]

    只统计 mode==2（BALANCE）的帧 —— 起摆段 w1 本来就在大范围变化，
    混进来会把"抖动"算成一个毫无意义的巨大数字。

    ★ settle：进入 BALANCE 后的前 settle 秒算【建立过程】，单独另算一份
      "稳定段"指标。为什么必须分开：
        2026-10-16 那份 60 s 基线里，投平衡后的头 10 秒 w1 从 10361 掉到 9930，
        std 高达 130 counts、斜率 -22.8 counts/s；若和后面一起统计，
        "净位移 -399 counts" 会把这段一次性建立过程当成"漂移"报出去，
        而实际上第 10 秒之后斜率一直在 ±1.1 counts/s 以内。
        这两个数说的是完全不同的事，混在一起就是误导。
    """
    bal = [(t, f) for (t, f) in frames if f[2] == 2]
    if not bal:
        return None
    ts = [t for t, _ in bal]
    w1 = [f[1] for _, f in bal]
    u = [f[3] for _, f in bal]
    dphi = [f[5] for _, f in bal]
    phib = [f[11] for _, f in bal]
    xset = [f[9] for _, f in bal]
    m = {
        "n_bal": len(bal),
        "_settle_arg": settle,              # 给 fmt_metrics 显示用
        "t_bal": ts[-1] - ts[0] if len(ts) > 1 else 0.0,
        "jitter": _std(w1),                 # 位置抖动（counts）
        "drift": _slope(ts, w1),            # 慢漂（counts/秒）
        "u_mean": _mean(u),                 # 电机指令均值（±4095）
        "u_std": _std(u),
        "u_sat_frac": sum(1 for x in u if abs(x) >= U_SAT) / float(len(u)),
        "phi_bias_peak": max(abs(x) for x in phib),   # 限幅 ±250
        "dphi_std": _std(dphi),
        "xset_last": xset[-1],
        "w1_first": w1[0],
        "w1_last": w1[-1],
    }
    # ---- 稳定段（去掉前 settle 秒）----
    t_cut = ts[0] + settle
    idx = [i for i in range(len(ts)) if ts[i] >= t_cut]
    if len(idx) >= 10:
        s_ts = [ts[i] for i in idx]
        s_w1 = [w1[i] for i in idx]
        s_u = [u[i] for i in idx]
        s_pb = [phib[i] for i in idx]
        m["settled"] = True
        m["s_t0"] = s_ts[0] - ts[0]
        m["s_dur"] = s_ts[-1] - s_ts[0]
        m["s_jitter"] = _std(s_w1)
        m["s_drift"] = _slope(s_ts, s_w1)
        m["s_net"] = s_w1[-1] - s_w1[0]
        m["s_mean"] = _mean(s_w1)
        m["s_u_mean"] = _mean(s_u)
        m["s_phib_peak"] = max(abs(x) for x in s_pb)
    else:
        m["settled"] = False
    return m


def fmt_metrics(m, cpr=CNT_PER_REV):
    if m is None:
        return "  （没有 BALANCE 帧，无法统计）"
    s = ("  平衡时长      : %6.2f s   (%d 帧)\n"
         "  ── 全程（含建立过程，仅供参考）──\n"
         "  位置抖动 std  : %6.1f counts  (%.2f 圈)\n"
         "  慢漂          : %+7.2f counts/s\n"
         "  位置净位移    : %+6d counts  (%+.2f 圈)\n"
         "  u_motor 均值  : %+7.1f   (std %.1f, 饱和占比 %.1f%%)\n"
         "  phi_bias 峰值 : %6.0f    (限幅 ±250)%s\n"
         "  dphi std      : %6.1f"
         % (m["t_bal"], m["n_bal"],
            m["jitter"], m["jitter"] / float(cpr),
            m["drift"],
            m["w1_last"] - m["w1_first"], (m["w1_last"] - m["w1_first"]) / float(cpr),
            m["u_mean"], m["u_std"], m["u_sat_frac"] * 100.0,
            m["phi_bias_peak"],
            "  ★接近限幅" if m["phi_bias_peak"] > 200 else "",
            m["dphi_std"]))
    if m.get("settled"):
        s += ("\n  ── ★稳定段（去掉前 %.0f s 建立过程）──\n"
              "  起点/时长     : 第 %5.1f s 起，共 %.1f s\n"
              "  位置抖动 std  : %6.1f counts  (%.2f 圈)\n"
              "  慢漂          : %+7.2f counts/s   <- 这个数才代表'稳不稳'\n"
              "  段内净位移    : %+6d counts  (%+.2f 圈)\n"
              "  段内均值位置  : %6.1f  (x_pos = %+.0f counts)\n"
              "  u_motor 均值  : %+7.1f\n"
              "  phi_bias 峰值 : %6.0f    (限幅 ±250)"
              % (m.get("_settle_arg", 15.0),
                 m["s_t0"], m["s_dur"],
                 m["s_jitter"], m["s_jitter"] / float(cpr),
                 m["s_drift"],
                 m["s_net"], m["s_net"] / float(cpr),
                 m["s_mean"], m["s_mean"] - 10000.0,
                 m["s_u_mean"],
                 m["s_phib_peak"]))
    else:
        s += "\n  （时长短于建立过程，未单独统计稳定段）"
    return s


# ===========================================================================
# 三、串口与看门狗
# ===========================================================================
class Abort(Exception):
    pass


def open_port(port, baud):
    import serial
    return serial.Serial(port, baud, timeout=0.05)


def record(ser, dec, secs, wait_balance, label=""):
    """等进入 BALANCE 后录 secs 秒；返回 (frames, 说明)。看门狗随时可中止。"""
    frames = []
    t_wait0 = time.time()
    bal_t0 = None
    last_rx = time.time()
    u_sat_run = 0
    while True:
        data = ser.read(256)
        now = time.time()
        if data:
            t_prev = last_rx
            last_rx = now
            got = dec.feed(data)
            # ★ 一次读可能同时带回好几帧（串口缓冲）。若给它们同一个时间戳，
            #   最小二乘斜率（慢漂）就会被算歪 —— 时间轴被压扁了。
            #   这里把这一坨帧均摊到 [上次读到本次读] 的区间内，
            #   比"全用同一个 now"接近真实得多。
            n = len(got)
            for i, f in enumerate(got):
                t = t_prev + (now - t_prev) * (i + 1) / float(n) if n else now
                frames.append((t, f))
                if f[2] == 3:
                    raise Abort("遥测报 FAULT (mode=3)")
                if abs(f[3]) >= U_SAT:
                    u_sat_run += 1
                    if u_sat_run >= U_SAT_ABORT_N:
                        raise Abort("u_motor 连续 %d 帧打满（|u|>=%d）"
                                    % (u_sat_run, U_SAT))
                else:
                    u_sat_run = 0
                if bal_t0 is None and f[2] == 2:
                    bal_t0 = t
                # ★ 起摆阶段防外漂（见 DRIFT_ABORT 的说明）：
                #   还没进 BALANCE 时，摆臂偏心一旦超过阈值就主动停机，
                #   避免撞 POS_LIMIT 触发 arm_fault（那会把摆臂越推越远）。
                if bal_t0 is None and f[2] != 2:
                    _pr = f[1] - 10000
                    if abs(_pr) > DRIFT_ABORT:
                        raise Abort("起摆阶段摆臂外漂到 pos_rel=%+d（阈值 %d）——主动停机"
                                    % (_pr, DRIFT_ABORT))
        else:
            if now - last_rx > NO_DATA_ABORT:
                raise Abort("连续 %.1f 秒收不到任何字节（线松了/板子没在发？）"
                            % NO_DATA_ABORT)

        if bal_t0 is None:
            if wait_balance is not None and now - t_wait0 > wait_balance:
                raise Abort("等了 %.0f 秒仍未进入 BALANCE（mode 始终不是 2）"
                            % wait_balance)
        else:
            if now - bal_t0 >= secs:
                return frames, "记录完成（BALANCE %.1f s）" % (now - bal_t0)
        if secs <= 0 and bal_t0 is not None:
            return frames, "已进入 BALANCE，--measure 0 故立即返回"


def save_csv(path, frames, gains, note=""):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# J280 autotune 记录  %s\n" % note)
        f.write("# 系数: %s\n" % gains)
        f.write("# 一圈 = %d counts\n" % CNT_PER_REV)
        # ★★ 字段名坑：遥测里那个叫 pd_sum 的字，其实【不是】P+D ★★
        #   见 pid_pipeline.v:305
        #       s4_sum = s3_p_term + s3_d_term + s3_px_term + s3_dx_term + s3_i_acc
        #   它把【内环 P/D + 外环 P/D + 积分 I】全都加在一起了，
        #   所以正常情况下它就恒等于 u_motor（只有 u 饱和时两者才不同）。
        #   2026-10-16 我按字面理解去拆"u 里有多少是 P、多少是 D"，
        #   得出 P+D=-33 却 pd_sum=+804 的自相矛盾结果，白绕了一圈。
        #   这里把列名改成 sum_incl_I 并注明分解式，免得以后重犯。
        #   正确分解：u ≈ p_term + d_term + phi_bias + I
        f.write("# 分解式: u_motor = p_term + d_term + phi_bias + I(积分)\n")
        f.write("t_s,w0_angle,w0_deg,w1_pos,w2_mode,u_motor,phi_eff,dphi,"
                "p_term,d_term,sum_incl_I,x_settle,err_x,phi_bias\n")
        if not frames:
            return
        t0 = frames[0][0]
        for t, v in frames:
            f.write("%.3f,%d,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
                    % (t - t0, v[0], SH.adc_to_deg(v[0]), v[1], v[2], v[3],
                       v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11]))


# ===========================================================================
# 四、主流程
# ===========================================================================
def send(ser, s, allow, echo=True):
    if not allow:
        raise SystemExit("★ 这条命令需要写权限：%r\n"
                         "  默认只读。确认板子处于安全状态后加 --allow-writes。" % s)
    for ch in s:
        ser.write(ch.encode("ascii"))
        time.sleep(0.02)
    if echo:
        print("  >> 已发送 %r" % s)


def selftest():
    print("=" * 72)
    print(" autotune.py 自检（不需要硬件）")
    print("=" * 72)
    ok = True

    def chk(name, exp, got):
        nonlocal ok
        good = exp == got
        ok = ok and good
        print("  [%s] %-46s exp=%s got=%s" % ("OK" if good else "FAIL", name, exp, got))

    # ---- 1) 操作算术必须与 GainTracker.apply 逐条一致 ----
    for op in OPS:
        for v in (-18000, -4000, -1, 0, 1, 64, 1000, 30000):
            gt = GainTracker()
            gt.g['PD'] = v
            gt.sel = 'PD'
            gt.apply(op)
            chk("op %s 在 %d 上一致" % (op, v), gt.g['PD'], op_apply(v, op))

    # ---- 2) 已知序列：-4000 走到 -18000 应是 **> ----
    seq, end = find_ops(-4000, -18000, max_ops=12)
    chk("-4000 -> -18000 可达", True, seq is not None)
    chk("-4000 -> -18000 终点正确", -18000, end)
    print("       序列 = %r" % seq)
    # 逐字节复核这条序列真的能走到
    v = -4000
    for c in (seq or ""):
        v = op_apply(v, c)
    chk("逐字节复核序列结果", -18000, v)

    # ---- 3) 起点即终点 -> 空序列 ----
    seq2, end2 = find_ops(-18000, -18000)
    chk("起点=终点 返回空序列", "", seq2)

    # ---- 4) 不可达时明确返回 None（不返回近似值）----
    #     把步数限成 1，-4000 不可能一步到 -18000
    seq3, end3 = find_ops(-4000, -18000, max_ops=1)
    chk("步数不够时返回 None", None, seq3)

    # ---- 5) 指标算法：造一段已知数据 ----
    #     w1 从 10000 起每帧 +7 counts，共 20 帧，t 每帧 0.1s
    #     -> 斜率应为 70 counts/s
    #     -> 抖动 std 应为 7*std(0..19) = 7*sqrt(35) = 41.4126
    #   ★★ 这里我自己写错过一次，记录一下 ★★
    #     第一版断言"完美线性数据的 std = 0"，结果自检报 FAIL（实得 41.41）。
    #     错在概念：std 量的是【围绕均值的离散程度】，一条斜坡当然有离散度；
    #     描述线性趋势的是【斜率 drift】，不是 std。
    #     是测试写错了，不是代码错了 —— 保留这条注释，避免以后再犯。
    frames = []
    for i in range(20):
        f = (0, 10000 + 7 * i, 2, 100, 0, 0, 0, 0, 0, 0, 0, 0)
        frames.append((i * 0.1, f))
    m = metrics(frames)
    chk("线性数据漂移 = 70.0 counts/s", 70.0, round(m["drift"], 3))
    chk("线性数据 std = 7*sqrt(35) = 41.4126", 41.412558, round(m["jitter"], 6))
    chk("全部计入 BALANCE 帧", 20, m["n_bal"])

    # ---- 5b) 常数数据：std 与 drift 都该是 0 ----
    frames_c = [(i * 0.1, (0, 10000, 2, 100, 0, 0, 0, 0, 0, 0, 0, 0)) for i in range(20)]
    mc = metrics(frames_c)
    chk("常数数据 std = 0", 0.0, round(mc["jitter"], 6))
    chk("常数数据漂移 = 0", 0.0, round(mc["drift"], 6))

    # ---- 5c) 斜坡 + 【与斜坡正交的】噪声 ----
    #   ★★ 这里我又写错过一次，一并记录 ★★
    #     第一版用"每帧 ±30 交替"的噪声，然后天真地以为
    #     "std 就是噪声本身的 30.78、斜率仍是 70"。两个都错：
    #       · std 里还有斜坡本身的离散（之前那条已经踩过一次，又忘了）
    #       · 交替噪声与帧序号 i 【相关】，协方差不为 0，
    #         既会把 std 抬高，也会把最小二乘斜率带偏
    #     实得 std=49.41、斜率偏离 70，自检如实报 FAIL。
    #   正确做法：构造与 i 【正交】的噪声 —— 让正负样本关于均值位置对称。
    #     本组 20 帧，均值位置 9.5；令 i=0,19 取 +30，i=9,10 取 -30，
    #     则 sum((i-9.5)*n_i) = 30*(0) - 30*(0) = 0，协方差恒为 0。
    #     这样"斜率应严格不变"才是一个【能验证】的断言。
    noise = [0] * 20
    noise[0] = 30
    noise[19] = 30
    noise[9] = -30
    noise[10] = -30
    frames_n = []
    ys = []
    for i in range(20):
        y = 10000 + 7 * i + noise[i]
        ys.append(y)
        frames_n.append((i * 0.1, (0, y, 2, 100, 0, 0, 0, 0, 0, 0, 0, 0)))
    mn = metrics(frames_n)
    #   期望 std 用【独立的直接算法】算出来，而不是引用被测函数 ——
    #   这样才是在核对 _std 的实现（如 n 与 n-1、均值口径），不是在自证。
    _mu = sum(ys) / 20.0
    exp_std = (sum((y - _mu) ** 2 for y in ys) / 19.0) ** 0.5
    chk("含噪数据 std = %.4f（独立算法核对）" % exp_std,
        round(exp_std, 4), round(mn["jitter"], 4))
    chk("与斜坡正交的噪声不改变斜率（仍 70.0）", 70.0, round(mn["drift"], 3))
    chk("噪声确实抬高了 std（> 纯斜坡的 41.41）", True, mn["jitter"] > 41.412558)

    # ---- 6) 非 BALANCE 帧必须被排除 ----
    frames2 = list(frames)
    for i in range(10):
        f = (0, 99999, 1, 4000, 0, 0, 0, 0, 0, 0, 0, 0)   # mode=1 起摆
        frames2.append((2.0 + i * 0.1, f))
    m2 = metrics(frames2)
    chk("起摆帧不污染统计（仍 20 帧）", 20, m2["n_bal"])
    chk("起摆帧不影响漂移（仍 70）", 70.0, round(m2["drift"], 3))

    # ---- 7) phi_bias 峰值与饱和占比 ----
    f_sat = (0, 10000, 2, 4095, 0, 0, 0, 0, 0, 0, 0, 250)
    frames3 = [(0.0, f_sat), (0.1, f_sat)]
    m3 = metrics(frames3)
    chk("phi_bias 峰值 = 250", 250, m3["phi_bias_peak"])
    chk("u 饱和占比 = 100%", 1.0, round(m3["u_sat_frac"], 3))

    # ---- 8) 全非 BALANCE -> 返回 None（不能假装有结果）----
    chk("无 BALANCE 帧返回 None", None, metrics([(0.0, (0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0))]))

    # ---- 9) 发送门控：只读时不许发 ----
    class FakeSer:
        def __init__(self): self.buf = b""
        def write(self, b): self.buf += b
    fs = FakeSer()
    try:
        send(fs, "R", allow=False, echo=False)
        chk("只读模式拒绝发送", "拒绝", "竟然发出去了")
    except SystemExit:
        chk("只读模式拒绝发送", "拒绝", "拒绝")
    chk("拒绝时确实没写串口", b"", fs.buf)
    send(fs, "R", allow=True, echo=False)
    chk("允许后正常发送", b"R", fs.buf)

    print()
    print("=" * 72)
    print(" 自检结果：%s" % ("全部通过 PASS" if ok else "存在失败 FAIL"))
    print("=" * 72)
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description="J280 开发板自动记录/调参（默认只读，加 --allow-writes 才会发命令）")
    ap.add_argument("--port", help="串口号，如 COM5")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    ap.add_argument("--allow-writes", action="store_true",
                    help="★ 允许向板子发送命令（默认只读，一个字节都不发）")
    ap.add_argument("--run", action="store_true", help="先发 'R' 启动（等价按 SW1）")
    ap.add_argument("--measure", type=float, default=60.0,
                    help="进入 BALANCE 后录制多少秒（默认 60）")
    ap.add_argument("--wait", type=float, default=25.0,
                    help="等进入 BALANCE 的超时秒数（默认 25）")
    ap.add_argument("--goto", action="append", default=[],
                    metavar="PARAM=VAL",
                    help="把系数走到指定值，如 --goto PD=-14000（可重复）")
    ap.add_argument("--step", action="append", default=[],
                    metavar="PARAM=OPS",
                    help="对系数施加操作序列，如 --step PD=>>>>（可重复）")
    ap.add_argument("--max-ops", type=int, default=12, help="BFS 最大步数（默认 12）")
    ap.add_argument("--outdir", default=None, help="CSV 输出目录（默认 <包>/记录）")
    ap.add_argument("--no-stop", action="store_true", help="退出时不发 'R' 停机")
    ap.add_argument("--monitor", type=float, default=None, metavar="SECS",
                    help="只监视 SECS 秒后退出：不发 'R'、不等 BALANCE、不改任何东西")
    ap.add_argument("--live", type=float, default=None, metavar="SECS",
                    help="实时逐帧打印（给「手转摆臂回零」这类操作看数用）。"
                         "只读，不发任何命令")
    ap.add_argument("--telemetry-on", action="store_true",
                    help="先发一个 'S' 打开遥测（固件复位后遥测默认关闭）")
    ap.add_argument("--send", default=None, metavar="STR",
                    help="发送指定字符（如 --send U 记录直立参考、--send R 启停）。"
                         "只发这一个字符串，不带任何隐含动作")
    ap.add_argument("--assume", action="append", default=[], metavar="PARAM=VAL",
                    help="声明某个系数【当前】已知是多少，用于让本地模型与实机重新对齐。"
                         "例：--assume PP=-1800。"
                         "★ 为什么需要它：串口无法读回系数，模型一旦与实机不一致，"
                         "只能靠人告诉它真值才能恢复")
    ap.add_argument("--selftest", action="store_true", help="无硬件自检后退出")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.list:
        import serial.tools.list_ports as lp
        ps = list(lp.comports())
        print("发现 %d 个串口：" % len(ps))
        for p in ps:
            print("   %-8s %s" % (p.device, p.description))
        if not ps:
            print("   （一个都没有 —— 板子插了吗？驱动装了吗？）")
        return 0

    if not args.port:
        print("需要 --port COMx（先用 --list 看有哪些口）")
        return 2

    outdir = args.outdir
    if outdir is None:
        outdir = os.path.join(os.path.dirname(_HERE), "记录")
    os.makedirs(outdir, exist_ok=True)

    print("=" * 72)
    print(" J280 autotune   串口=%s 波特率=%d" % (args.port, args.baud))
    if not args.allow_writes:
        print(" ★ 只读模式：本次不会向板子发送任何字节")
    else:
        print(" ★ 写模式已开启：会发命令。请人在场、手边就是电源开关。")
    print("=" * 72)

    ser = open_port(args.port, args.baud)
    dec = SH.FrameDecoder()
    gt = GainTracker()
    stopped = False
    started = False          # ★ 是否由【本脚本】发过 'R' 把它启动起来
    try:
        # ---- 先把模型对齐到已知真值（--assume）----
        #   ★ 串口读不回系数，所以一旦模型和实机不一致（例如工具中途报错、
        #     或有人用上位机手动改过），只能靠这个显式声明来恢复。
        for spec in args.assume:
            if '=' not in spec:
                print("!! --assume 格式应为 PARAM=VAL，收到 %r" % spec)
                return 2
            p, v = spec.split('=', 1)
            p = p.upper()
            if p not in SEL_KEY:
                print("!! --assume 未知系数 %r" % p)
                return 2
            old = gt.g[p]
            gt.g[p] = int(v)
            print("   [assume] %s: 模型原值 %d -> %d" % (p, old, int(v)))

        # ---- 可选：打开遥测（固件复位后遥测默认关闭，不开发 'S' 一个字节都收不到）----
        if args.telemetry_on:
            send(ser, 'S', args.allow_writes)
            time.sleep(0.3)

        # ---- 可选：只发一个指定字符（如 'U' 记直立参考、'0' 定零点）----
        #   ★ 刻意做成"只发这一个字符串，不带任何隐含动作"：
        #     不像某些工具发完还会顺手补一个 'R'，那会意外启动电机。
        if args.send is not None:
            if args.send == 'R':
                started = True        # 明确记录：'R' 是启停切换，退出时要负责停
            send(ser, args.send, args.allow_writes)
            time.sleep(0.4)

        # ---- 实时逐帧模式：给"手转摆臂回零"这类操作一边转一边看数 ----
        if args.live is not None:
            print("\n实时显示 %.0f 秒。板子只读，不会发任何命令。" % args.live)
            print("  目标：把摆臂转到 w1 ≈ 10000（pos_rel ≈ 0）")
            print("  %8s %7s %8s %7s %-10s %s" %
                  ("t", "w1", "pos_rel", "w0", "mode", "进度"))
            t0 = time.time()
            last_rx = t0
            while time.time() - t0 < args.live:
                data = ser.read(256)
                now = time.time()
                if data:
                    last_rx = now
                    for f in dec.feed(data):
                        pr = f[1] - 10000
                        # 用 |pos_rel|/POS_LIMIT 画一个粗略进度条，方便眼看
                        frac = min(abs(pr) / 6000.0, 1.0)
                        bar = "#" * int(frac * 30)
                        print("  %8.2f %7d %+8d %7d %-10s %-30s %.0f%%"
                              % (now - t0, f[1], pr, f[0],
                                 MODE_NAME.get(f[2], "?"), bar, frac * 100))
                elif now - last_rx > NO_DATA_ABORT:
                    print("!! 连续 %.1f 秒收不到任何字节" % NO_DATA_ABORT)
                    break
            print("\n（实时显示结束；全程未发送任何字节）")
            return 0

        # ---- 纯监视模式：读 SECS 秒就退出，不发 'R'、不等 BALANCE ----
        if args.monitor is not None:
            print("\n只监视 %.1f 秒（不发 'R'、不改任何东西）..." % args.monitor)
            frames = []
            t0 = time.time()
            last_rx = time.time()
            while time.time() - t0 < args.monitor:
                data = ser.read(256)
                now = time.time()
                if data:
                    t_prev = last_rx
                    last_rx = now
                    got = dec.feed(data)
                    n = len(got)
                    for i, f in enumerate(got):
                        t = t_prev + (now - t_prev) * (i + 1) / float(n) if n else now
                        frames.append((t - t0, f))
                elif now - last_rx > NO_DATA_ABORT:
                    print("!! 连续 %.1f 秒收不到任何字节" % NO_DATA_ABORT)
                    break
            print("\n收到 %d 帧，失步 %d 次" % (dec.frames, dec.bad))
            if frames:
                print("\n最后 5 帧：")
                for t, f in frames[-5:]:
                    print("  t=%6.2fs  w0=%5d  w1=%5d  w2=%d %-8s  u=%6d  "
                          "phiE=%5d  dphi=%5d  xset=%5d  errx=%5d  phib=%4d"
                          % (t, f[0], f[1], f[2], MODE_NAME.get(f[2], "?"),
                             f[3], f[4], f[5], f[9], f[10], f[11]))
                m = metrics(frames)
                if m:
                    print("\n（这段里有 BALANCE 帧，指标如下；IDLE 段无意义属正常）")
                    print(fmt_metrics(m))
                else:
                    print("\n模式分布：" + ", ".join(
                        "%s×%d" % (MODE_NAME.get(k, k), v)
                        for k, v in sorted(_mode_hist(frames).items())))
            return 0

        # ---- 可选：先把系数走到位 ----
        #   ★ 顺序很重要：必须【先设系数、后启动】。
        #     原来是 --run 在前面，等于"用旧系数启动、跑起来之后再改"，
        #     那样这一次记录的前半段用的是旧值，指标就被污染了。
        #     而且带着坏系数启动更危险。改成先设后起。
        for spec in args.step:
            if ':' not in spec:
                print("!! --step 格式应为 PARAM:OPS，收到 %r" % spec)
                return 2
            param, ops = spec.split(':', 1)
            param = param.upper()
            if param not in SEL_KEY:
                print("!! 未知系数 %r" % param)
                return 2
            send(ser, SEL_KEY[param], args.allow_writes, echo=False)
            # ★★ 必须同时更新本地模型的"当前选中项" ★★
            #   GainTracker.apply('4') 会把 sel 置成 'PP'，与固件行为一一对应。
            #   原来只发串口、不调这一句，导致 gt.sel 一直停在 'KP'：
            #   于是板子上翻倍的是 PP、模型里翻倍的却是 KP，
            #   复核必然失败，而且模型从此与实机不一致（还没有任何办法读回来）。
            #   2026-10-16 实际踩到：板子 PP 已变成 -1800，模型却以为还是 -900。
            gt.apply(SEL_KEY[param])
            send(ser, ops, args.allow_writes)
            for c in ops:
                gt.apply(c)
            print("   %s 现为 %d (%.4f)" % (param, gt.g[param], gt.g[param] / 4096.0))

        for spec in args.goto:
            if '=' not in spec:
                print("!! --goto 格式应为 PARAM=VAL，收到 %r" % spec)
                return 2
            param, val = spec.split('=', 1)
            param = param.upper()
            val = int(val)
            if param not in SEL_KEY:
                print("!! 未知系数 %r" % param)
                return 2
            cur = gt.g[param]
            seq, end = find_ops(cur, val, max_ops=args.max_ops)
            if seq is None:
                print("!! %s 从 %d 走不到 %d（<=%d 步内搜索无解）—— 不发任何命令"
                      % (param, cur, val, args.max_ops))
                return 3
            print("   %s: %d -> %d 用序列 %r" % (param, cur, val, seq))
            send(ser, SEL_KEY[param], args.allow_writes, echo=False)
            gt.apply(SEL_KEY[param])      # ★ 同步模型选中项（见 --step 处的说明）
            if seq:
                send(ser, seq, args.allow_writes)
            for c in seq:
                gt.apply(c)
            if gt.g[param] != val:
                print("!! 复核失败：模型算出 %d，期望 %d —— 中止"
                      % (gt.g[param], val))
                return 4

        # ---- 可选：启动（放在设置系数【之后】）----
        if args.run:
            send(ser, 'R', args.allow_writes)
            started = True
            time.sleep(0.3)

        # ---- 录制 ----
        print("\n开始录制（等 BALANCE，最多 %.0f s，之后录 %.0f s）..."
              % (args.wait, args.measure))
        frames, note = record(ser, dec, args.measure, args.wait)
        m = metrics(frames)
        print("\n" + "=" * 72)
        print(" 结果  %s" % note)
        print("=" * 72)
        print(fmt_metrics(m))

        stamp = time.strftime("%Y%m%d_%H%M%S")
        csv = os.path.join(outdir, "auto_%s.csv" % stamp)
        # ★ 把当时的系数写进记录 —— 这是以前缺的：
        #   老版上位机的 CSV 只有遥测、不含系数，调完参数没法回溯是哪一组。
        gains = " ".join("%s=%d" % (k, gt.g[k]) for k in
                         ('KP', 'KI', 'KD', 'PP', 'PD', 'SW'))
        save_csv(csv, frames, gains, note)
        print("\nCSV: %s" % csv)
        print("系数: %s" % gains)

    except Abort as e:
        print("\n" + "!" * 72)
        print("!! 看门狗中止：%s" % e)
        print("!" * 72)
    except KeyboardInterrupt:
        print("\n!! 手动中断（Ctrl-C）")
    finally:
        # ★★★ 这一处曾经写错，是个【会害人的】错，记录在此 ★★★
        #   原来的写法是"只要开了写权限、且没加 --no-stop，退出就发 'R'"。
        #   但 'R' 是【切换】键（运行/停止是同一个键）：
        #     · 对着正在跑的板子发 'R' -> 停机   ✓ 本意
        #     · 对着已经停机的板子发 'R' -> 【启动电机】 ✗ 危险！
        #   也就是说：一个"只是打开遥测看一眼"的调用
        #   （--allow-writes --telemetry-on，本来不发任何运动命令），
        #   在退出时反而会把电机点起来。
        #   修法：只有【本次确实由我们发过 'R' 启动】时才在退出时停机。
        if started and args.allow_writes and not args.no_stop:
            try:
                ser.write(b'R')
                stopped = True
            except Exception as e:
                print("!! 停机命令发送失败：%s   ★ 请手动按 SW5 或断电！" % e)
        ser.close()
        print("串口已关闭%s" % ("，已发 'R' 停机" if stopped
                                else "（本次未启动过电机，故不发 'R'）"))
        print("收到 %d 帧，失步 %d 次" % (dec.frames, dec.bad))
    return 0


if __name__ == "__main__":
    sys.exit(main())
