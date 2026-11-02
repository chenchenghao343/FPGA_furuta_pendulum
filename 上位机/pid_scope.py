#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_scope.py  --  J280 倒立摆 实时 PID 看图工具
================================================================================
一次看到两组"目标 / 实际 / 输出"三线图：

    上子图  内环（角度环）   实际 phi | 目标 phi_bias | 输出 u_motor
    下子图  外环（位置环）   实际 w1_pos | 目标 x_settle | 误差 err_x

数据源是 v6.0 的 28 字节二进制遥测帧（AA 55 | w0..w11 | 0D 0A），
所以 **不需要改 FPGA 固件、不需要重综合**，拿现成的位流就能用。

--------------------------------------------------------------------------------
为什么不用 SerialPlot
--------------------------------------------------------------------------------
SerialPlot 读 ASCII 文本，而 v6.0 默认发的是二进制帧，且帧里的目标值
（x_settle / phi_bias）SerialPlot 无法解析。本工具直接在主机侧解码，
零固件改动。

--------------------------------------------------------------------------------
用法
--------------------------------------------------------------------------------
    # 真机
    python pid_scope.py --port COM5
    python pid_scope.py --port COM5 --up-ref 2905      # 已知直立参考就填上
    python pid_scope.py --port COM5 --record run.csv   # 顺便存 CSV

    # 无硬件自测（造一帧假数据走完整条链路，验证解码与绘图）
    python pid_scope.py --demo
    python pid_scope.py --demo --demo-file run_20261009_170249.csv

    # 无窗口自检（解码 + 换算 + 绘图链路，输出 PNG，不需要显示器）
    python pid_scope.py --selftest

串口参数固定 9600 8N1（与固件一致）。

--------------------------------------------------------------------------------
依赖
--------------------------------------------------------------------------------
    pyserial   (pip install pyserial)
    matplotlib (pip install matplotlib)
    tkinter    通常随 Python 自带
================================================================================
"""

import argparse
import os
import queue
import sys
import threading
import time

# ---------------------------------------------------------------------------
# 协议常量（与 src/uart_link.v 严格一致）
# ---------------------------------------------------------------------------
SOF0, SOF1, EOF0, EOF1 = 0xAA, 0x55, 0x0D, 0x0A
FRAME_LEN = 28          # ★ v6.0
BAUD = 9600

# 角度换算（与 serial_host.py 一致：下垂≈1010, 直立≈3096 -> 2086 counts / 180°）
ADC_FULL, ADC_DEG = 2086.0, 180.0

MODE_NAME = {0: "IDLE", 1: "SWING", 2: "BALANCE", 3: "FAULT"}

# 编码器每圈 counts（用于把位置/误差换算成"圈"）。
#   ★ v6.4：由 1024 改为 1067 —— 与固件 pendulum_top.v 的 .CNT_PER_REV
#     和 serial_host.py 的 CNT_PER_REV 保持一致（1067 = 用户 2026-10-16 给定值）。
#   ★ 如实说明：本常量目前在 pid_scope.py 里【没有任何地方引用】，
#     改了不影响画图结果。留着是为了将来加"圈"刻度时有个一致的基准，
#     不要让它停留在过期数值上误导人。
CNT_PER_REV = 1067.0


def s16(b, i):
    """大端有符号 16bit"""
    v = (b[i] << 8) | b[i + 1]
    return v - 65536 if v >= 32768 else v


class FrameDecoder:
    """从字节流里切出 28 字节帧并解出 w0..w11。

    ★ 解码逻辑与 tools/serial_host.py 完全一致（那份已经过 self-test 验证），
      这里只是把返回值改成字典，方便绘图时按名字取。
    """

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.bad = 0

    def feed(self, data):
        self.buf.extend(data)
        out = []
        while True:
            i = self.buf.find(bytes([SOF0, SOF1]))
            if i < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                    self.bad += 1
                return out
            if i > 0:
                del self.buf[:i]
                self.bad += 1
            if len(self.buf) < FRAME_LEN:
                return out
            f = bytes(self.buf[:FRAME_LEN])
            if f[26] == EOF0 and f[27] == EOF1:
                out.append({
                    "w0_angle":  (f[2] << 8) | f[3],
                    "w1_pos":    s16(f, 4),
                    "w2_mode":   (f[6] << 8) | f[7],
                    "u_motor":   s16(f, 8),
                    "phi_eff":   s16(f, 10),
                    "dphi":      s16(f, 12),
                    "p_term":    s16(f, 14),
                    "d_term":    s16(f, 16),
                    "pd_sum":    s16(f, 18),
                    # ★ v6.0 新增：外环三个量
                    "x_settle":  s16(f, 20),
                    "err_x":     s16(f, 22),
                    "phi_bias":  s16(f, 24),
                })
                self.frames += 1
                del self.buf[:FRAME_LEN]
            else:
                del self.buf[:1]
                self.bad += 1


# ===========================================================================
#  数据源
# ===========================================================================
class TextLineDecoder:
    """从字节流里切出【文本帧】，每行 6 个整数。

    ★ 什么时候用：固件里 TEXT_CH = 6（文本 6 通道）时，串口上发的是 ASCII
      而不是 28 字节二进制帧。此时用本解码器；二进制模式用 FrameDecoder。

    列顺序（与 src/uart_text6.v 严格一致）：
        1 phi        内环 实际
        2 phi_bias   内环 目标
        3 u_motor    内环 输出
        4 w1_pos     外环 实际
        5 x_settle   外环 目标
        6 err_x      外环 误差

    实测帧形如：
        +02905 -00012 -04095 +09999 +10780 -00113
    """

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.bad = 0

    def feed(self, data):
        self.buf.extend(data)
        out = []
        while True:
            i = self.buf.find(b"\n")
            if i < 0:
                # 防止半行无限堆积（真正一行最长 42 字节）
                if len(self.buf) > 4096:
                    del self.buf[:-512]
                    self.bad += 1
                return out
            line = bytes(self.buf[:i]).strip()
            del self.buf[:i + 1]
            if not line:
                continue
            parts = line.split()
            if len(parts) != 6:
                self.bad += 1
                continue
            try:
                vals = [int(p) for p in parts]
            except ValueError:
                self.bad += 1
                continue
            phi, phib, u, pos, xset, errx = vals
            # 转成与二进制帧相同的字典结构，绘图代码不用改
            out.append({
                "w0_angle":  0,
                "w1_pos":    pos,
                "w2_mode":   0,          # 文本模式没有模式字段
                "u_motor":   u,
                "phi_eff":   phi - phib,
                "dphi":      0,
                "p_term":    0,
                "d_term":    0,
                "pd_sum":    0,
                "x_settle":  xset,
                "err_x":     errx,
                "phi_bias":  phib,
                "_phi":      phi,        # 已是相对直立的偏差，不再减 up_ref
            })
            self.frames += 1
        return out


class SerialSource(threading.Thread):
    """后台线程读串口，解帧后塞进 queue。

    mode='bin'  -> 28 字节二进制帧（TEXT_CH=0）
    mode='text' -> 6 列 ASCII 文本行（TEXT_CH=6）
    """

    def __init__(self, port, q, stop_evt, mode="bin"):
        threading.Thread.__init__(self, daemon=True)
        self.port, self.q, self.stop_evt = port, q, stop_evt
        self.dec = FrameDecoder() if mode == "bin" else TextLineDecoder()
        self.err = None

    def run(self):
        try:
            import serial
        except ImportError:
            self.err = "缺少 pyserial: pip install pyserial"
            return
        try:
            ser = serial.Serial(self.port, BAUD, timeout=0.05)
        except Exception as e:
            self.err = "打开串口失败 %s: %s" % (self.port, e)
            return
        try:
            while not self.stop_evt.is_set():
                data = ser.read(256)
                if not data:
                    continue
                for fr in self.dec.feed(data):
                    self.q.put(fr)
        finally:
            try:
                ser.close()
            except Exception:
                pass


class DemoSource(threading.Thread):
    """无硬件自测：造一段合成遥测，形状模仿真实平衡段。

    目的：验证「解码 -> 换算 -> 绘图」整条链路，不需要插板子。
    波形用 sine + 缓慢漂移 + 噪声，周期 5.85 s（与实测一致）。
    """

    def __init__(self, q, stop_evt, csv_path=None):
        threading.Thread.__init__(self, daemon=True)
        self.q, self.stop_evt, self.csv_path = q, stop_evt, csv_path
        self.err = None

    def run(self):
        import math
        import random

        # 若给了真实 CSV，就直接回放（列少时缺的字段补 0）
        rows = []
        if self.csv_path and os.path.isfile(self.csv_path):
            with open(self.csv_path, newline="", encoding="utf-8-sig") as f:
                import csv as _csv
                rd = _csv.DictReader(f)
                for r in rd:
                    rows.append(r)
        t0 = time.time()
        i = 0
        while not self.stop_evt.is_set():
            if rows:
                r = rows[i % len(rows)]
                fr = {
                    "w0_angle": int(float(r.get("w0_angle", 2905))),
                    "w1_pos":   int(float(r.get("w1_pos", 10000))),
                    "w2_mode":  int(float(r.get("w2_mode", 2))),
                    "u_motor":  int(float(r.get("u_motor", 0))),
                    "phi_eff":  int(float(r.get("phi_eff", 0))),
                    "dphi":     int(float(r.get("dphi", 0))),
                    "p_term":   int(float(r.get("p_term", 0))),
                    "d_term":   int(float(r.get("d_term", 0))),
                    "pd_sum":   int(float(r.get("pd_sum", 0))),
                    "x_settle": int(float(r.get("x_settle", 0))),
                    "err_x":    int(float(r.get("err_x", 0))),
                    "phi_bias": int(float(r.get("phi_bias", 0))),
                }
            else:
                tt = time.time() - t0
                w = 2 * math.pi / 5.85                    # 与实测周期一致
                drift = -1.9 * tt                          # counts/s
                pos = 9700 + drift + 120 * math.sin(w * tt) + random.gauss(0, 3)
                phib = -8 + 22 * math.cos(w * tt)          # 外环输出（很弱）
                phi = phib + random.gauss(0, 12)           # 内环实际（跟得紧）
                errx = phib * 128                          # kp_x=-32/Q12 -> x128
                fr = {
                    "w0_angle": int(2905 + phi),
                    "w1_pos":   int(pos),
                    "w2_mode":  2,
                    "u_motor":  int(950 + 1700 * math.sin(w * tt + 1.0) +
                                    random.gauss(0, 300)),
                    "phi_eff":  int(phi - phib),
                    "dphi":     int(3 * math.cos(w * tt)),
                    "p_term":   int(21231 * (phi - phib) / 1024),
                    "d_term":   int(2697 * 64 * (3 * math.cos(w * tt)) / 1024),
                    "pd_sum":   0,
                    "x_settle": int(10000),
                    "err_x":    int(errx),
                    "phi_bias": int(phib),
                }
            fr["t"] = time.time()
            self.q.put(fr)
            i += 1
            time.sleep(0.08)     # 12.5 Hz，与真实二进制帧率一致


# ===========================================================================
#  主界面
# ===========================================================================
def run_gui(q, stop_evt, title, up_ref_arg, record_path, window_s):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from collections import deque

    # 中文显示（图内标签用英文，避免字体缺失；标题栏可以中文）
    plt.rcParams["axes.unicode_minus"] = False

    NMAX = 4000
    buf = {k: deque(maxlen=NMAX) for k in
           ("t", "phi", "phi_bias", "u", "pos", "x_settle", "err_x", "mode")}

    state = {
        "up_ref": up_ref_arg,
        "ang_hist": deque(maxlen=600),   # 用于自动估 up_ref
        "rec": None,
        "nrec": 0,
        "status": "等待数据 ...",
    }

    if record_path:
        state["rec"] = open(record_path, "w", encoding="utf-8", newline="")
        state["rec"].write("t_s,w0_angle,w1_pos,w2_mode,u_motor,phi_eff,"
                           "phi_bias,x_settle,err_x\n")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.canvas.manager.set_window_title(title)

    # ---- 上子图：内环（角度环）------------------------------------------
    ax1b = ax1.twinx()
    ln_phi, = ax1.plot([], [], "b-", lw=1.6, label="INNER actual  phi (deg-ish counts)")
    ln_tgt, = ax1.plot([], [], "r--", lw=1.6, label="INNER target  phi_bias")
    ln_u, = ax1b.plot([], [], color="purple", lw=0.9, label="INNER output  u_motor")
    ax1.set_ylabel("angle [counts rel. upright]")
    ax1b.set_ylabel("u_motor")
    ax1b.set_ylim(-4600, 4600)
    ax1.grid(True)
    lns = [ln_phi, ln_tgt, ln_u]
    ax1.legend(lns, [l.get_label() for l in lns], loc="upper left", fontsize=9)
    ax1.set_title("INNER (angle) loop   target / actual / output", fontweight="bold")

    # ---- 下子图：外环（位置环）------------------------------------------
    ln_pos, = ax2.plot([], [], "b-", lw=1.6, label="OUTER actual  w1_pos")
    ln_set, = ax2.plot([], [], "r--", lw=1.6, label="OUTER target  x_settle")
    ln_err, = ax2.plot([], [], color="darkorange", lw=1.2, label="OUTER error  err_x")
    ax2.set_ylabel("arm position [counts]")
    ax2.grid(True)
    ax2.legend(loc="upper left", fontsize=9)
    ax2.set_title("OUTER (position) loop   target / actual / error", fontweight="bold")
    ax2.set_xlabel("t [s]")
    ax2b = ax2.twinx()
    ax2b.set_ylabel("phi_bias")
    ax2b.set_ylim(-270, 270)

    txt = ax1.text(0.99, 0.02, "", transform=ax1.transAxes,
                   ha="right", va="bottom", fontsize=9, family="monospace",
                   bbox=dict(fc="white", ec="0.7", alpha=0.9))

    def update(_frame):
        drained = 0
        while True:
            try:
                fr = q.get_nowait()
            except queue.Empty:
                break
            drained += 1
            if state["up_ref"] is None and "_phi" not in fr:
                state["ang_hist"].append(fr["w0_angle"])
            t = fr["t"]
            u = state["up_ref"] if state["up_ref"] is not None else 0
            buf["t"].append(t)
            if "_phi" in fr:
                # 文本模式：phi 已经在固件侧减过 up_ref
                buf["phi"].append(fr["_phi"])
            else:
                buf["phi"].append(fr["w0_angle"] - u)
            buf["phi_bias"].append(fr["phi_bias"])
            buf["u"].append(fr["u_motor"])
            buf["pos"].append(fr["w1_pos"])
            buf["x_settle"].append(fr["x_settle"])
            buf["err_x"].append(fr["err_x"])
            buf["mode"].append(fr["w2_mode"])
            if state["rec"]:
                state["rec"].write("%.3f,%d,%d,%d,%d,%d,%d,%d,%d\n" % (
                    t, fr["w0_angle"], fr["w1_pos"], fr["w2_mode"], fr["u_motor"],
                    fr["phi_eff"], fr["phi_bias"], fr["x_settle"], fr["err_x"]))
                state["nrec"] += 1
        if drained == 0 and not buf["t"]:
            return lns

        # ---- 自动估 up_ref（前 600 帧的中值）----
        if state["up_ref"] is None and len(state["ang_hist"]) >= 200:
            s = sorted(state["ang_hist"])
            state["up_ref"] = s[len(s) // 2]
            buf["phi"] = deque((a - state["up_ref"] for a in
                                list(buf["phi"])), maxlen=NMAX)
            state["status"] = "up_ref 自动估为 %d（扶正摆杆后按串口 'U' 可重标）" % state["up_ref"]

        ts = list(buf["t"])
        if not ts:
            return lns
        t_end = ts[-1]
        t_start = t_end - window_s

        ln_phi.set_data(ts, list(buf["phi"]))
        ln_tgt.set_data(ts, list(buf["phi_bias"]))
        ln_u.set_data(ts, list(buf["u"]))
        ln_pos.set_data(ts, list(buf["pos"]))
        ln_set.set_data(ts, list(buf["x_settle"]))
        ln_err.set_data(ts, list(buf["err_x"]))

        ax1.set_xlim(t_start, t_end)
        for ax in (ax1,):
            ax.relim(); ax.autoscale_view(scalex=False)
        ax2.relim(); ax2.autoscale_view(scalex=False)

        mode = buf["mode"][-1] if buf["mode"] else 0
        pb = abs(buf["phi_bias"][-1]) if buf["phi_bias"] else 0
        txt.set_text("mode=%-8s  up_ref=%s\npairs=%d  |phi_bias|=%d/%d%s" % (
            MODE_NAME.get(mode, "?"), state["up_ref"],
            len(buf["t"]), pb, 250, "  SAT!" if pb >= 250 else ""))
        return lns

    def on_close(_evt):
        stop_evt.set()
        if state["rec"]:
            state["rec"].close()
            print("CSV 已写 %d 行 -> %s" % (state["nrec"], record_path))
        plt.close("all")

    fig.canvas.mpl_connect("close_event", on_close)
    ani = animation.FuncAnimation(fig, update, interval=80, blit=False,
                                  cache_frame_data=False)
    plt.tight_layout()
    plt.show()


# ===========================================================================
#  无窗口自检：解码 -> 换算 -> 绘图，全链路走一遍并出 PNG
# ===========================================================================
def selftest():
    ok = True

    def chk(name, exp, got):
        nonlocal ok
        good = (exp == got)
        ok = ok and good
        print("  [%s] %-46s exp=%s got=%s" % ("OK" if good else "FAIL", name, exp, got))

    print("=" * 74)
    print(" pid_scope 自检（不需要硬件、不需要显示器）")
    print("=" * 74)

    # ---- 1) 解码一帧标准 28 字节帧 -------------------------------------
    #   数值：角度3148 位置10079 模式2 u=1000 phiE=100 dphi=-100
    #         P=300 D=-200 PD=200  xset=1078 errx=-5 phib=+30
    f = bytes([0xAA, 0x55, 0x0C, 0x4C, 0x27, 0x5F, 0x00, 0x02, 0x03, 0xE8,
               0x00, 0x64, 0xFF, 0x9C, 0x01, 0x2C, 0xFF, 0x38, 0x00, 0xC8,
               0x04, 0x36, 0xFF, 0xFB, 0x00, 0x1E,
               0x0D, 0x0A])
    d = FrameDecoder()
    r = d.feed(f)
    chk("整帧解出 1 帧", 1, len(r))
    if r:
        fr = r[0]
        chk("w0_angle = 3148", 3148, fr["w0_angle"])
        chk("w1_pos   = 10079", 10079, fr["w1_pos"])
        chk("w2_mode  = 2", 2, fr["w2_mode"])
        chk("u_motor  = 1000", 1000, fr["u_motor"])
        chk("phi_eff  = 100", 100, fr["phi_eff"])
        chk("dphi     = -100", -100, fr["dphi"])
        chk("x_settle = 1078  (v6.0 新字段)", 1078, fr["x_settle"])
        chk("err_x    = -5    (v6.0 新字段)", -5, fr["err_x"])
        chk("phi_bias = 30    (v6.0 新字段)", 30, fr["phi_bias"])

    # ---- 2) 逐字节喂入（串口会碎片化）----------------------------------
    d2 = FrameDecoder()
    got = []
    for b in f:
        got += d2.feed(bytes([b]))
    chk("逐字节喂入仍解出 1 帧", 1, len(got))

    # ---- 3) 前置垃圾 + 两帧粘连 ----------------------------------------
    d3 = FrameDecoder()
    chk("前置垃圾后解出 1 帧", 1, len(d3.feed(b"\x11\x22" + f)))
    d4 = FrameDecoder()
    chk("两帧粘连解出 2 帧", 2, len(d4.feed(f + f)))

    # ---- 4) 帧尾损坏后能重新同步 ---------------------------------------
    d5 = FrameDecoder()
    bad = bytearray(f); bad[26] = 0x00
    chk("帧尾损坏后重新同步", 1, len(d5.feed(bytes(bad) + f)))

    # ---- 5) 绘图链路（Agg 后端，无显示器）------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
        ax1.plot([0, 1, 2], [0, 10, 0], "b-")
        ax1.plot([0, 1, 2], [0, 8, 0], "r--")
        ax2.plot([0, 1, 2], [10000, 9800, 10100], "b-")
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "_pid_scope_selftest.png")
        fig.savefig(out, dpi=110)
        plt.close(fig)
        chk("matplotlib 绘图 + 存 PNG", True, os.path.isfile(out) and
            os.path.getsize(out) > 1000)
        print("      （自检图: %s，可以删掉）" % out)
    except Exception as e:
        chk("matplotlib 绘图", "无异常", "异常: %s" % e)

    # ---- 6) 合成数据源能造出帧 -----------------------------------------
    q = queue.Queue()
    ev = threading.Event()
    src = DemoSource(q, ev)
    src.start()
    time.sleep(0.35)
    ev.set()
    chk("DemoSource 能产出帧", True, q.qsize() > 0)

    print("-" * 74)
    print(" 自检%s" % ("全部通过 PASS" if ok else "有失败项 FAIL"))
    print("=" * 74)
    return 0 if ok else 1


# ===========================================================================
#  CLI
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="J280 倒立摆 PID 看图工具")
    ap.add_argument("--port", help="串口号，如 COM5")
    ap.add_argument("--up-ref", type=int, default=None,
                    help="直立参考（ADC 值）。不给则用前 600 帧中值自动估，"
                         "并提示用串口 'U' 重标定")
    ap.add_argument("--record", help="顺便把所有帧记成 CSV")
    ap.add_argument("--window", type=float, default=30.0, help="横轴窗口秒数")
    ap.add_argument("--demo", action="store_true", help="无硬件自测（造数据）")
    ap.add_argument("--demo-file", help="--demo 时回放这个 CSV（可省）")
    ap.add_argument("--text", action="store_true",
                    help="串口上跑的是【文本 6 通道】模式（TEXT_CH=6）时加这个；"
                         "默认按二进制 28 字节帧解")
    ap.add_argument("--selftest", action="store_true", help="无窗口自检后退出")
    ap.add_argument("--list", action="store_true", help="列出可用串口后退出")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.list:
        try:
            import serial.tools.list_ports as lp
        except ImportError:
            print("缺少 pyserial: pip install pyserial")
            return 2
        for p in lp.comports():
            print("  %-8s %s" % (p.device, p.description))
        return 0

    if not args.demo and not args.port:
        ap.error("要么给 --port，要么用 --demo 自测。用 --list 看有哪些串口。")

    q = queue.Queue(maxsize=20000)
    stop_evt = threading.Event()

    if args.demo:
        src = DemoSource(q, stop_evt, args.demo_file)
        title = "J280 PID Scope  [DEMO]"
        print("DEMO 模式：数据是合成的（周期 5.85 s，与实测一致）。")
        print("        目的只是验证解码+绘图链路；真数据请用 --port COMx。")
    else:
        src = SerialSource(args.port, q, stop_evt,
                           mode=("text" if args.text else "bin"))
        title = "J280 PID Scope  %s  [%s]" % (args.port,
                                              "TEXT" if args.text else "BIN")
        print("打开 %s @ %d 8N1，模式 = %s ..." % (
            args.port, BAUD, "文本6通道" if args.text else "二进制28字节"))
    src.start()
    time.sleep(0.3)
    if getattr(src, "err", None):
        print("错误：", src.err)
        return 2
    run_gui(q, stop_evt, title, args.up_ref, args.record, args.window)
    stop_evt.set()
    return 0


if __name__ == "__main__":
    sys.exit(main())
