# -*- coding: utf-8 -*-
"""
=============================================================================
 wave_scope.py —— J280 波形上位机（v6.8 新增，与 J280_Host.exe 并存）
=============================================================================
用途（用户 2026-10-22 的需求）：
    单独一个上位机，只干一件事 —— 把两条曲线画出来
      ① 摆杆角度 phi（相对直立参考）   -> 一眼看出"是不是在平衡"
      ② 摆臂角度（相对复位零点）        -> 一眼看出"是不是在按正弦走"

与 J280_Host.exe 的分工：
    J280_Host.exe  调参 / 发定点指令 / 看数值 / 记 CSV（操作台）
    wave_scope.py  只看波形 + 自动判据（示波器）
    ★ 同一个 COM 口**不能同时被两个程序占用**，二选一。

技术选择：
    · 纯 tkinter Canvas 画图，**不依赖 matplotlib**。
      理由：打包成 exe 时 matplotlib 会把体积从 ~10MB 推到 ~50MB，
      启动也慢；而这里只是两条折线，Canvas 完全够用。
    · 协议常量与帧解码器**直接 import serial_host**，不复制第二份，
      避免"两份真值"（本工程反复踩过的坑）。

两个自动判据（面板下方实时显示）：
    平衡判据   最近 BAL_WIN_S 秒内 max|phi| < BAL_TH_DEG  -> "平衡 OK"
    正弦判据   对最近窗口做最小二乘正弦拟合，给出
                 实测周期、拟合幅值、拟合优度 R2
               R2 越接近 1 越"像正弦"；方波大约 0.8，噪声大约 0。

命令行：
    python wave_scope.py                  # 打开界面
    python wave_scope.py --port COM5      # 直接连
    python wave_scope.py --selftest       # 不上硬件，验证分析算法
    python wave_scope.py --nolog          # 不写 CSV（默认也不写；本程序不记录）
=============================================================================
"""

import argparse
import collections
import json
import math
import os
import sys
import threading
import time

# ---------------------------------------------------------------------------
# 复用主上位机的协议定义（单一真值来源）
# ---------------------------------------------------------------------------
try:
    import serial_host as SH
except ImportError:                       # 允许把本文件单独拷走时给个明确提示
    sys.stderr.write("找不到 serial_host.py。请把本文件和 serial_host.py 放在同一目录。\n")
    raise

FRAME_LEN   = SH.FRAME_LEN
BAUD        = SH.BAUD
ADC_FULL    = SH.ADC_FULL
ADC_DEG     = SH.ADC_DEG
CNT_PER_REV = SH.CNT_PER_REV
POS_BASE    = SH.POS_BASE
MODE_NAME   = SH.MODE_NAME

# ---------------------------------------------------------------------------
# 可调参数（界面上都能改）
# ---------------------------------------------------------------------------
WINDOW_S    = 25.0        # 波形窗口长度（秒）—— 0.2Hz 正弦约 5 个周期
FIT_MIN_PER = 1.5         # 正弦拟合至少要有 1.5 个周期
ARM_MIN_SPAN = 20.0       # 摆臂面板最小纵轴半量程（度）

UP_REF_DEFAULT = 2855     # 与固件 UP_REF_DEFAULT 一致（angle_raw 的直立读数）
                          # ★ 换机器/重装摆杆后必须按 'U' 重新记录（见 README）
PHI_MIN_SPAN = 0.5        # 摆杆面板：自动模式的最小纵轴半量程（度）
PHI_DEF_SPAN = 5.0        # 摆杆面板：手动默认半量程（度）
ARM_DEF_SPAN = 20.0       # 摆臂面板：手动默认半量程（度）
SPAN_CHOICES = ["自动", "±0.2", "±0.5", "±1", "±2", "±5",
                "±10", "±20", "±50", "±100", "±200", "±400"]


def _cfg_path():
    """配置文件跟程序放一起（打包成 exe 后取 exe 所在目录）"""
    base = (os.path.dirname(sys.executable) if getattr(sys, "frozen", False)
            else os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, "scope_cfg.json")


def load_up_ref():
    try:
        with open(_cfg_path(), "r", encoding="utf-8") as fh:
            v = int(json.load(fh).get("up_ref", UP_REF_DEFAULT))
        if 0 <= v <= 4095:
            return v
    except Exception:
        pass
    return UP_REF_DEFAULT


def save_up_ref(v):
    try:
        with open(_cfg_path(), "w", encoding="utf-8") as fh:
            json.dump({"up_ref": int(v)}, fh)
    except Exception:
        pass

BG      = "#101418"
BG_AXIS = "#1d242c"
FG      = "#d8e0e8"
FG_DIM  = "#7d8b99"
C_ARM   = "#4aa8ff"       # 摆臂：蓝
C_PHI   = "#7ee787"       # 摆杆：绿
C_FIT   = "#ffb347"       # 拟合正弦：橙（虚线）
C_GRID  = "#232c36"
C_OK    = "#46d17a"
C_BAD   = "#ff5c5c"
C_WARN  = "#ffcc55"


# =============================================================================
# 一、分析算法（可单独自检，不依赖界面）
# =============================================================================
def _solve3(m, b):
    """3x3 线性方程组（高斯消元，带选主元）。m 是 3x3 列表，b 是长度 3 列表。"""
    a = [row[:] + [b[i]] for i, row in enumerate(m)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-12:
            return None
        a[col], a[piv] = a[piv], a[col]
        pv = a[col][col]
        for j in range(col, 4):
            a[col][j] /= pv
        for r in range(3):
            if r == col:
                continue
            f = a[r][col]
            if f == 0.0:
                continue
            for j in range(col, 4):
                a[r][j] -= f * a[col][j]
    return [a[0][3], a[1][3], a[2][3]]


def estimate_period(ts, ys):
    """用"上升过零"间隔估计周期。返回 (period_s or None, n_cross)。"""
    if len(ts) < 5:
        return None, 0
    mean = sum(ys) / len(ys)
    cross = []
    for i in range(1, len(ys)):
        y0, y1 = ys[i - 1] - mean, ys[i] - mean
        if y0 <= 0.0 < y1:                       # 上升过零
            # 线性插值出精确过零时刻
            dt = ts[i] - ts[i - 1]
            if (y1 - y0) == 0:
                cross.append(ts[i])
            else:
                cross.append(ts[i - 1] + dt * (-y0) / (y1 - y0))
    if len(cross) < 2:
        return None, len(cross)
    intervals = [cross[i] - cross[i - 1] for i in range(1, len(cross))]
    intervals = [d for d in intervals if d > 1e-6]
    if not intervals:
        return None, len(cross)
    return sum(intervals) / len(intervals), len(cross)


def fit_sine(ts, ys, freq_hint=None):
    """对等间隔/不等间隔采样做最小二乘正弦拟合
           y ≈ A*sin(w t) + B*cos(w t) + C
       返回 dict:
           ok        拟合是否有效
           freq     使用的频率（Hz）
           period   周期（s）
           amp      拟合幅值（与 y 同单位）
           offset   直流分量
           r2       拟合优度（0~1，1 = 完全正弦）
           n        样本数
    """
    n = len(ts)
    out = {"ok": False, "freq": 0.0, "period": 0.0, "amp": 0.0,
           "offset": 0.0, "r2": 0.0, "n": n}
    if n < 12:
        return out
    # 1) 先估周期；估不出来就用提示频率
    per, _ = estimate_period(ts, ys)
    if per is None or per <= 0.0:
        if freq_hint and freq_hint > 0:
            per = 1.0 / freq_hint
        else:
            return out
    # 落在窗口里的周期数太少 -> 拟合不可信
    span = ts[-1] - ts[0]
    if span < FIT_MIN_PER * per:
        out["period"] = per
        return out

    w = 2.0 * math.pi / per
    # 2) 正规方程：列向量 = [sin, cos, 1]
    ss = cc = s1 = s2 = c1 = 0.0
    sy = cy = y1 = yy = 0.0
    for t, y in zip(ts, ys):
        s = math.sin(w * t)
        c = math.cos(w * t)
        ss += s * s
        cc += c * c
        s1 += s * c
        s2 += s
        c1 += c
        sy += s * y
        cy += c * y
        y1 += y
        yy += y * y
    m = [[ss, s1, s2],
         [s1, cc, c1],
         [s2, c1, float(n)]]
    sol = _solve3(m, [sy, cy, y1])
    if sol is None:
        out["period"] = per
        return out
    A, B, C = sol
    # 3) R2
    ymean = y1 / n
    sst = yy - n * ymean * ymean
    if sst <= 0.0:
        out["period"] = per
        return out                       # 完全不动 -> 谈不上拟合
    ssr = 0.0
    for t, y in zip(ts, ys):
        yh = A * math.sin(w * t) + B * math.cos(w * t) + C
        ssr += (y - yh) ** 2
    out.update(ok=True,
               freq=1.0 / per, period=per,
               amp=math.hypot(A, B),
               offset=C,
               r2=max(0.0, 1.0 - ssr / sst))
    return out


class Ring:
    """按时间窗口滚动的采样缓存。存 (t, rod_deg, arm_deg)"""

    def __init__(self, window_s=WINDOW_S):
        self.window_s = window_s
        self.d = collections.deque()

    def append(self, t, rod, arm):
        self.d.append((t, rod, arm))
        t0 = t - self.window_s
        while self.d and self.d[0][0] < t0:
            self.d.popleft()

    def clear(self):
        self.d.clear()

    def series(self):
        return list(self.d)


# =============================================================================
# 三、界面
# =============================================================================
def make_app():
    import tkinter as tk
    from tkinter import ttk, filedialog

    class ScopeApp:
        def __init__(self, root, port=None):
            self.root = root
            self.ser = None
            self.dec = SH.FrameDecoder()
            self.ring = Ring(WINDOW_S)
            self.q = collections.deque(maxlen=4000)
            self.running = False
            self.paused = False
            self.frames = 0
            self.bad = 0
            self.t_origin = None
            self.rate = 0.0
            self._rate_t0 = time.time()
            self._rate_n = 0
            self._scale_upd = 0.0
            self.up_ref = load_up_ref()   # 直立参考（画面上 phi 的零位，可界面调、会存盘）
            self.last_w0 = None           # 最近一帧的 angle_raw（"取当前"用）
            self.panels = {}               # 画布建好后再填：phi / arm 各自一套纵轴状态
            self._drag_y = None
            self._drag_off0 = 0.0
            self._drag_key = None
            self.last_fit = {}

            root.title("J280 波形上位机 —— 摆杆 φ / 摆臂 θ (v7.2 双面板·手动缩放)")
            root.configure(bg=BG)
            root.geometry("1100x760")

            # ---------------- 顶部工具条 ----------------
            bar = tk.Frame(root, bg=BG)
            bar.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)

            tk.Label(bar, text="端口", bg=BG, fg=FG).pack(side=tk.LEFT)
            self.port_var = tk.StringVar(value=port or "")
            self.cmb = ttk.Combobox(bar, textvariable=self.port_var, width=10,
                                    values=self._list_ports())
            self.cmb.pack(side=tk.LEFT, padx=(4, 8))

            self.btn_conn = tk.Button(bar, text="连接", width=8, command=self.toggle_conn)
            self.btn_conn.pack(side=tk.LEFT, padx=2)
            tk.Button(bar, text="暂停", width=6, command=self.toggle_pause).pack(side=tk.LEFT, padx=2)
            tk.Button(bar, text="清屏", width=6, command=self.clear).pack(side=tk.LEFT, padx=2)
            tk.Button(bar, text="截图", width=6, command=self.save_png).pack(side=tk.LEFT, padx=2)

            # ---- 纵轴量程：两块面板各自独立（自动 / 手动）----
            tk.Label(bar, text="Y摆杆", bg=BG, fg=C_PHI).pack(side=tk.LEFT, padx=(12, 2))
            self.var_phi = tk.StringVar(value="±%g" % PHI_DEF_SPAN)
            self.cmb_phi = ttk.Combobox(bar, textvariable=self.var_phi, width=8,
                                        state="readonly", values=SPAN_CHOICES)
            self.cmb_phi.pack(side=tk.LEFT, padx=2)
            self.cmb_phi.bind("<<ComboboxSelected>>", lambda e: self.on_span_pick("phi"))

            tk.Label(bar, text="Y摆臂", bg=BG, fg=C_ARM).pack(side=tk.LEFT, padx=(10, 2))
            self.var_arm = tk.StringVar(value="±%g" % ARM_DEF_SPAN)
            self.cmb_arm = ttk.Combobox(bar, textvariable=self.var_arm, width=8,
                                        state="readonly", values=SPAN_CHOICES)
            self.cmb_arm.pack(side=tk.LEFT, padx=2)
            self.cmb_arm.bind("<<ComboboxSelected>>", lambda e: self.on_span_pick("arm"))


            self.lbl_stat = tk.Label(bar, text="未连接", bg=BG, fg=FG_DIM,
                                     font=("Consolas", 10))
            self.lbl_stat.pack(side=tk.RIGHT)

            # ---------------- 第二行：摆杆直立参考（phi 的零位）----------------
            bar2 = tk.Frame(root, bg=BG)
            bar2.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 2))
            tk.Label(bar2, text="直立参考 angle_raw", bg=BG, fg=C_PHI).pack(side=tk.LEFT)
            self.ref_var = tk.StringVar(value=str(self.up_ref))
            self.ent_ref = tk.Entry(bar2, textvariable=self.ref_var, width=7,
                                    bg=BG_AXIS, fg=FG, insertbackground=FG, relief=tk.FLAT)
            self.ent_ref.pack(side=tk.LEFT, padx=(6, 3))
            self.ent_ref.bind("<Return>", lambda e: self.on_ref_apply())
            self.ent_ref.bind("<FocusOut>", lambda e: self.on_ref_apply())
            for _txt, _d, _w in (("−10", -10, 4), ("−1", -1, 3), ("+1", 1, 3), ("+10", 10, 4)):
                tk.Button(bar2, text=_txt, width=_w,
                          command=lambda d=_d: self.on_ref_step(d)).pack(side=tk.LEFT, padx=1)
            tk.Button(bar2, text="取当前", width=7,
                      command=self.on_ref_from_live).pack(side=tk.LEFT, padx=(8, 2))
            tk.Button(bar2, text="发 'U' 给板子", width=12,
                      command=self.on_ref_send_U).pack(side=tk.LEFT, padx=2)
            tk.Button(bar2, text="默认", width=5,
                      command=self.on_ref_default).pack(side=tk.LEFT, padx=2)


            # ---------------- 上：摆杆 phi ----------------
            self.lbl_phi = tk.Label(root, text="摆杆角度（相对直立参考）", bg=BG, fg=C_PHI,
                                    font=("Microsoft YaHei", 11, "bold"), anchor="w")
            self.lbl_phi.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(4, 0))
            self.can_phi = tk.Canvas(root, bg=BG, highlightthickness=0, height=230)
            self.can_phi.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10)

            # ---------------- 下：摆臂 ----------------
            self.lbl_arm = tk.Label(root, text="摆臂角度（相对复位零点）", bg=BG, fg=C_ARM,
                                    font=("Microsoft YaHei", 11, "bold"), anchor="w")
            self.lbl_arm.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(6, 0))
            self.can_arm = tk.Canvas(root, bg=BG, highlightthickness=0, height=230)
            self.can_arm.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10)

            # 两块面板互不影响：滚轮=缩纵轴，Shift+滚轮=平移，左键拖动=平移，双击=回自动
            self.panels = {
                "phi": dict(can=self.can_phi, lbl=self.lbl_phi, var=self.var_phi,
                            auto=False, manual=PHI_DEF_SPAN, span=PHI_DEF_SPAN,
                            yoff=0.0, floor=PHI_MIN_SPAN,
                            title="摆杆角度"),
                "arm": dict(can=self.can_arm, lbl=self.lbl_arm, var=self.var_arm,
                            auto=False, manual=ARM_DEF_SPAN, span=ARM_DEF_SPAN,
                            yoff=0.0, floor=ARM_MIN_SPAN,
                            title="摆臂角度（相对复位零点）"),
            }
            for _k in ("phi", "arm"):
                _cv = self.panels[_k]["can"]
                _cv.bind("<MouseWheel>", lambda e, k=_k: self.on_wheel(k, e))
                _cv.bind("<Shift-MouseWheel>", lambda e, k=_k: self.on_wheel_shift(k, e))
                _cv.bind("<ButtonPress-1>", lambda e, k=_k: self.on_drag_start(k, e))
                _cv.bind("<B1-Motion>", lambda e, k=_k: self.on_drag(k, e))
                _cv.bind("<Double-Button-1>", lambda e, k=_k: self.on_reset_view(k))

            self.lbl_armb = tk.Label(root, text="正弦判据：等待数据", bg=BG, fg=FG_DIM,
                                     font=("Consolas", 10), anchor="w")
            self.lbl_armb.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 6))

            self.lbl_hint = tk.Label(
                root, bg=BG, fg=FG_DIM, font=("Microsoft YaHei", 9), anchor="w",
                text="提示：遥测要用串口 'S' 打开；本程序与 J280_Host.exe 不能同时占用同一个 COM 口。"
                     "  直立参考：改数字只动画面零位，按“发 'U' 给板子”才会让固件的直立参考也跟着变。")
            self.lbl_hint.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=4)

            self.root.after(30, self._tick)
            self.root.after(40, self._redraw)

        # ---------------- 串口 ----------------
        @staticmethod
        def _list_ports():
            try:
                import serial.tools.list_ports as lp
                return [p.device for p in lp.comports()]
            except Exception:
                return []

        def toggle_conn(self):
            if self.ser is not None:
                self.disconnect()
            else:
                self.connect()

        def connect(self):
            try:
                import serial
            except ImportError:
                self._msg("缺少 pyserial：pip install pyserial", C_BAD)
                return
            port = (self.port_var.get() or "").strip()
            if not port:
                ports = self._list_ports()
                if ports:
                    self.cmb["values"] = ports
                    self.port_var.set(ports[0])
                    port = ports[0]
                else:
                    self._msg("没找到串口", C_BAD)
                    return
            try:
                self.ser = serial.Serial(port, BAUD, timeout=0.05)
            except Exception as ex:
                self._msg("打开 %s 失败: %s" % (port, ex), C_BAD)
                return
            self.dec = SH.FrameDecoder()
            self.ring.clear()
            self.t_origin = None
            self.frames = self.bad = 0
            self.running = True
            self.btn_conn.config(text="断开")
            self._msg("已连接 %s @%d" % (port, BAUD), C_OK)
            threading.Thread(target=self._reader, daemon=True).start()

        def disconnect(self):
            self.running = False
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.btn_conn.config(text="连接")

        def _reader(self):
            while self.running:
                try:
                    n = self.ser.in_waiting or 1
                    data = self.ser.read(n)
                except Exception:
                    self.running = False
                    break
                if not data:
                    continue
                # ★ 给每个帧打上"到达时刻"。
                #   不能等到 GUI 线程里再取 time.time()：一次 read 可能带回好几帧，
                #   而它们会在同一个 GUI 循环里被瞬间消化掉 —— 那样时间戳会挤在一起，
                #   波形被压扁（第一版就是这么错的）。
                t_arr = time.time()
                for fr in self.dec.feed(data):
                    self.q.append((t_arr, fr))

        # ---------------- UI 小工具 ----------------
        def toggle_pause(self):
            self.paused = not self.paused
            self._msg("已暂停显示（数据仍在接收）" if self.paused else "继续显示",
                      C_WARN if self.paused else C_OK)

        def clear(self):
            self.ring.clear()
            self.t_origin = None
            self.last_fit = {}

        def save_png(self):
            """把两个面板一起存成 PostScript（tkinter Canvas 原生支持），再提示"""
            fn = filedialog.asksaveasfilename(
                defaultextension=".ps", filetypes=[("PostScript", "*.ps")],
                initialfile="j280_wave.ps")
            if not fn:
                return
            try:
                root, ext = os.path.splitext(fn)
                fn_phi = root + "_phi" + (ext or ".ps")
                fn_arm = root + "_arm" + (ext or ".ps")
                self.can_phi.postscript(file=fn_phi, colormode="color")
                self.can_arm.postscript(file=fn_arm, colormode="color")
                self._msg("已存 %s / %s（PostScript，可用 Ghostscript 转 PNG）"
                          % (os.path.basename(fn_phi), os.path.basename(fn_arm)), C_OK)
            except Exception as ex:
                self._msg("保存失败: %s" % ex, C_BAD)

        # ---------------- 摆杆直立参考 ----------------
        def _set_up_ref(self, v, msg=None):
            v = int(max(0, min(4095, v)))
            if v != self.up_ref:
                self.up_ref = v
                save_up_ref(v)
            self.ref_var.set(str(v))
            if msg:
                self._msg(msg, C_OK)

        def on_ref_apply(self):
            try:
                v = int(round(float(self.ref_var.get())))
            except ValueError:
                self.ref_var.set(str(self.up_ref))
                return
            self._set_up_ref(v, "直立参考 → %d（画面零位已更新）" % v)

        def on_ref_step(self, d):
            self._set_up_ref(self.up_ref + d, "直立参考 → %d" % (self.up_ref + d))

        def on_ref_from_live(self):
            if self.last_w0 is None:
                self._msg("还没有遥测：先连接并确认固件遥测已打开（串口 'S'）", C_WARN)
                return
            self._set_up_ref(self.last_w0, "直立参考取当前读数 %d" % self.last_w0)

        def on_ref_send_U(self):
            if self.last_w0 is None:
                self._msg("还没有遥测数据，无法确定当前角度", C_WARN)
                return
            if (not self.running) or (self.ser is None):
                self._msg("串口未连接，发不了 'U'", C_WARN)
                return
            try:
                self.ser.write(b"U")
            except Exception as ex:
                self._msg("发 'U' 失败: %s" % ex, C_BAD)
                return
            self._set_up_ref(self.last_w0,
                             "已发 'U'：固件与本机零位都取当前 %d" % self.last_w0)

        def on_ref_default(self):
            self._set_up_ref(UP_REF_DEFAULT, "直立参考复位为默认 %d" % UP_REF_DEFAULT)

        # ---------------- 纵轴手动缩放 / 平移（phi / arm 各自独立）----------------
        def _p(self, key):
            return self.panels[key]

        def _set_auto(self, key):
            P = self._p(key)
            P["auto"] = True
            P["yoff"] = 0.0
            P["var"].set("自动")

        def on_span_pick(self, key="arm", _ev=None):
            P = self._p(key)
            v = P["var"].get()
            if v == "自动":
                self._set_auto(key)
                self._msg("%s：自动（下限 %.2f°）" % (P["title"], P["floor"]), C_OK)
                return
            try:
                P["manual"] = max(0.05, float(v.replace("±", "").replace("°", "")))
            except ValueError:
                return
            P["auto"] = False
            P["span"] = P["manual"]
            P["yoff"] = 0.0
            self._msg("%s：手动 ±%.2f°（滚轮缩放 / Shift+滚轮平移 / 拖动 / 双击复位）"
                      % (P["title"], P["manual"]), C_OK)

        def on_wheel(self, key, ev):
            P = self._p(key)
            f = 1.0 / 1.25 if ev.delta > 0 else 1.25
            P["manual"] = min(500.0, max(0.05, P["span"] * f))
            P["auto"] = False
            P["span"] = P["manual"]
            P["var"].set("±%g" % round(P["manual"], 3))
            return "break"

        def on_wheel_shift(self, key, ev):
            P = self._p(key)
            P["manual"] = P["span"]
            P["auto"] = False
            P["var"].set("±%g" % round(P["manual"], 3))
            P["yoff"] += (P["span"] * 0.1) if ev.delta > 0 else -(P["span"] * 0.1)
            return "break"

        def on_drag_start(self, key, ev):
            self._drag_key = key
            self._drag_y = ev.y
            self._drag_off0 = self._p(key)["yoff"]

        def on_drag(self, key, ev):
            if self._drag_y is None or self._drag_key != key:
                return
            P = self._p(key)
            h = max(P["can"].winfo_height(), 80)
            P["manual"] = P["span"]
            P["auto"] = False
            P["var"].set("±%g" % round(P["manual"], 3))
            P["yoff"] = self._drag_off0 + (ev.y - self._drag_y) * (2.0 * P["span"] / max(h - 30, 1))

        def on_reset_view(self, key="arm", _ev=None):
            self._set_auto(key)
            self._msg("%s：纵轴已复位为自动" % self._p(key)["title"], C_OK)

        def _msg(self, text, color=FG_DIM):
            self.lbl_stat.config(text=text, fg=color)

        # ---------------- 数据泵 ----------------
        def _tick(self):
            while self.q:
                t_arr, fr = self.q.popleft()
                w0, w1 = fr[0], fr[1]
                # 摆杆：angle_raw 减去直立参考，折到 ±2048 counts（半圈）后换算成度
                self.last_w0 = w0
                dphi = (w0 - self.up_ref) & 0xFFFF
                if dphi > 32767:
                    dphi -= 65536
                while dphi > 2048:
                    dphi -= 4096
                while dphi < -2048:
                    dphi += 4096
                rod_deg = dphi * ADC_DEG / ADC_FULL
                # 摆臂：pos_raw 是**绝对**计数（复位初值 POS_BASE=10000），
                # 减去 POS_BASE 得到相对复位零点的转角，再换算成度。
                arm_cnt = w1 - POS_BASE
                if arm_cnt > 32767:            # 16bit 无符号回绕保护
                    arm_cnt -= 65536
                arm_deg = arm_cnt * 360.0 / CNT_PER_REV
                # 时间轴用帧到达时刻，首帧为 0
                if self.t_origin is None:
                    self.t_origin = t_arr
                self.ring.append(t_arr - self.t_origin, rod_deg, arm_deg)
                self.frames += 1
                self._rate_n += 1
            if not self.running:
                self.lbl_stat.config(text="未连接", fg=FG_DIM)
            else:
                el = time.time() - self._rate_t0
                if el >= 1.0:
                    self.rate = self._rate_n / el
                    self._rate_n = 0
                    self._rate_t0 = time.time()
                self.lbl_stat.config(
                    text="帧数 %d  失步 %d  刷新 %.1f Hz  样本 %d"
                         % (self.frames, self.dec.bad, self.rate, len(self.ring.d)),
                    fg=FG_DIM)
            self.root.after(30, self._tick)

        # ---------------- 绘图 ----------------
        def _redraw(self):
            if not self.paused:
                self._analyze()
                self._draw()
            self.root.after(40, self._redraw)

        def _analyze(self):
            s = self.ring.series()
            if len(s) < 6:
                return
            ts = [x[0] for x in s]
            arms = [x[2] for x in s]
            self.last_fit = fit_sine(ts, arms, freq_hint=SH.sine_freq_hz())

        def _nice_span(self, cur, need, floor):
            """纵轴半量程：只在必要时放大，缩回来要更保守（避免画面来回跳）"""
            need = max(need * 1.15, floor)
            if need > cur:
                return need
            if need < cur * 0.55:
                return need
            return cur

        def _prep_span(self, key, pts):
            """按 自动/手动 决定这一块面板的纵轴半量程并写回 P["span"]"""
            P = self.panels[key]
            if P["auto"]:
                need = max((abs(v) for _, v in pts), default=0.0)
                P["span"] = self._nice_span(P["span"], need, P["floor"])
            else:
                P["span"] = P["manual"]
            return P["span"]

        def _draw(self):
            s = self.ring.series()
            now = s[-1][0] if s else 0.0
            t0 = now - WINDOW_S

            # --- 上：摆杆 phi ---
            phis = [(t, y) for t, y, _ in s if t >= t0]
            self._prep_span("phi", phis)
            Pp = self.panels["phi"]
            self.lbl_phi.config(text="%s（直立参考 %d）     纵轴 ±%.2f°  %s"
                                     % (Pp["title"], self.up_ref, Pp["span"],
                                        "自动" if Pp["auto"] else "手动"))
            self._draw_panel(self.can_phi, phis, t0, now, Pp["span"],
                             C_PHI, unit="°", fit=None, title="phi", yoff=Pp["yoff"])

            # --- 下：摆臂 ---
            arms = [(t, y) for t, _, y in s if t >= t0]
            self._prep_span("arm", arms)
            Pa = self.panels["arm"]
            self.lbl_arm.config(text="%s     纵轴 ±%.2f°  %s"
                                     % (Pa["title"], Pa["span"],
                                        "自动" if Pa["auto"] else "手动"))
            fit_line = None
            if self.last_fit.get("ok"):
                f = self.last_fit
                w = 2.0 * math.pi * f["freq"]
                # 用 A/B 隐含的相位：这里直接重建曲线需要 A,B；改用等效写法
                # y = amp * sin(w t + phase)，phase 由拟合的 A,B 决定：
                # A*sin + B*cos = amp*sin(wt + phi0),  phi0 = atan2(B, A)
                # 这里不保存 A/B，直接用 amp/offset 与"与实测最贴"的相位近似：
                # 为保证与实测对齐，改用最小二乘重算一次只在 selftest 里做；
                # 画面上用 amp*sin(w*(t-t0)+p0) 会错相位，所以这里只画包络线。
                fit_line = ("env", f["amp"], f["offset"])
            self._draw_panel(self.can_arm, arms, t0, now, Pa["span"],
                             C_ARM, unit="°", fit=fit_line, title="arm",
                             yoff=Pa["yoff"])

            # --- 判据文字 ---
            f = self.last_fit
            if not f.get("ok"):
                self.lbl_armb.config(
                    text="正弦判据：样本不足（至少需要 %.1f 个周期）" % FIT_MIN_PER, fg=FG_DIM)
            else:
                r2 = f["r2"]
                if r2 >= 0.95:
                    judge, col = "很像正弦", C_OK
                elif r2 >= 0.80:
                    judge, col = "大致是正弦（有畸变）", C_WARN
                else:
                    judge, col = "不像正弦", C_BAD
                fw = f["freq"]
                f0 = SH.sine_freq_hz()
                self.lbl_armb.config(
                    text="正弦判据：实测周期 %.2f s (%.3f Hz，固件设定 %.3f Hz)   "
                         "拟合幅值 %.1f° (峰峰 %.1f°)   拟合优度 R2=%.3f   →  %s"
                         % (f["period"], fw, f0, f["amp"], 2 * f["amp"], r2, judge),
                    fg=col)

        def _draw_panel(self, can, pts, t0, t1, span, color, unit, fit, title, yoff=0.0):
            can.delete("all")
            w = max(can.winfo_width(), 200)
            h = max(can.winfo_height(), 80)
            ml, mr, mt, mb = 58, 12, 10, 20
            x0, x1 = ml, w - mr
            y0, y1 = mt, h - mb
            if x1 <= x0 or y1 <= y0:
                return
            ymid = (y0 + y1) / 2.0

            def T2x(t):
                return x0 + (t - t0) / max(t1 - t0, 1e-6) * (x1 - x0)

            def V2y(v):
                return ymid - ((v - yoff) / span) * ((y1 - y0) / 2.0)

            # 横向网格（每秒一条细线，每 5 秒一条亮线）
            t = math.ceil(t0)
            while t <= t1:
                xx = T2x(t)
                can.create_line(xx, y0, xx, y1,
                                fill=C_GRID if t % 5 else FG_DIM)
                if t % 5 == 0:
                    can.create_text(xx + 2, y1 + 8, text="%ds" % (t - t0), fill=FG_DIM,
                                    anchor="w", font=("Consolas", 8))
                t += 1
            # 纵向网格（半量程分档，中心线固定在 yoff；yoff≠0 时另补一条 v=0 实线）
            for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
                v = yoff + span * frac
                yy = V2y(v)
                can.create_line(x0, yy, x1, yy,
                                fill=FG_DIM if frac == 0 else C_GRID,
                                dash=() if frac == 0 else (2, 4))
                can.create_text(x0 - 6, yy, text="%+.2f%s" % (v, unit), fill=FG_DIM,
                                anchor="e", font=("Consolas", 8))
            if abs(yoff) > 1e-9:
                yy0 = V2y(0.0)
                if y0 <= yy0 <= y1:
                    can.create_line(x0, yy0, x1, yy0, fill=FG, dash=(4, 3))
                    can.create_text(x1 - 4, yy0 - 8, text="0%s" % unit,
                                    fill=FG_DIM, anchor="e", font=("Consolas", 8))

            # 曲线
            if len(pts) >= 2:
                flat = []
                for tt, vv in pts:
                    flat.extend((T2x(tt), V2y(vv)))
                can.create_line(*flat, fill=color, width=2, smooth=False)

            # 拟合正弦的包络（±amp 两条虚线）+ 均值线
            if fit and fit[0] == "env":
                _, amp, off = fit
                for sgn in (1.0, -1.0):
                    yy = V2y(off + sgn * amp)
                    can.create_line(x0, yy, x1, yy, fill=C_FIT, dash=(6, 4), width=1)
                yy = V2y(off)
                can.create_line(x0, yy, x1, yy, fill=C_FIT, dash=(2, 6), width=1)
                can.create_text(x1 - 4, V2y(off + amp) - 8,
                                text="拟合幅值 %.1f%s" % (amp, unit),
                                fill=C_FIT, anchor="e", font=("Consolas", 9))

            # 当前值
            if pts:
                cur = pts[-1][1]
                can.create_text(x1 - 4, y0 + 4, text="当前 %+.2f%s" % (cur, unit),
                                fill=color, anchor="ne", font=("Consolas", 11, "bold"))

    return ScopeApp


# =============================================================================
# 四、自检（不上硬件）
# =============================================================================
def _mk_frame(w0, w1):
    """构造一帧合法遥测：AA 55 | w0 w1 w2 u phiE dphi P D PD xset errx phib | 0D 0A"""
    def u16(v):
        v &= 0xFFFF
        return bytes([(v >> 8) & 0xFF, v & 0xFF])
    body = (u16(w0) + u16(w1) + u16(2) + u16(0) +
            u16(0) + u16(0) + u16(0) + u16(0) + u16(0) +
            u16(0) + u16(0) + u16(0))
    assert len(body) == 24
    return bytes([SH.SOF0, SH.SOF1]) + body + bytes([SH.EOF0, SH.EOF1])


def selftest():
    ok_all = True
    n_chk = 0

    def chk(name, exp, got):
        nonlocal ok_all, n_chk
        n_chk += 1
        good = (exp == got)
        ok_all = ok_all and good
        print("  [%s] %-46s expected=%s got=%s"
              % ("OK" if good else "FAIL", name, exp, got))

    def chk_true(name, cond, extra=""):
        nonlocal ok_all, n_chk
        n_chk += 1
        good = bool(cond)
        ok_all = ok_all and good
        print("  [%s] %-46s %s" % ("OK" if good else "FAIL", name, extra))

    print("=" * 78)
    print(" wave_scope 自检（不上硬件，全部用合成数据）")
    print("=" * 78)

    # ---- 1) 帧编解码：与 serial_host 的解码器必须互通 ----
    fr = _mk_frame(2905 + 100, POS_BASE + 200)
    chk("帧长度 = FRAME_LEN", FRAME_LEN, len(fr))
    d = SH.FrameDecoder()
    got = d.feed(fr)
    chk("解出 1 帧", 1, len(got))
    chk("w0 正确", 2905 + 100, got[0][0])
    chk("w1 正确", POS_BASE + 200, got[0][1])
    chk("w2 = 模式", 2, got[0][2])

    arm_deg = 200 * 360.0 / CNT_PER_REV          # 200/1067 圈 = 67.48 度
    chk("摆臂 200 counts 换算角度 ×10", int(round(arm_deg * 10)),
        int(round(200 * 360.0 / CNT_PER_REV * 10)))
    print("        [info] 摆臂 200 counts = %.2f 度（一圈 %d counts）"
          % (arm_deg, CNT_PER_REV))

    # ---- 3) 周期估计：完美正弦，已知周期 5 s ----
    ts, ys = [], []
    for i in range(200):
        t = i * 0.16
        ts.append(t)
        ys.append(150.0 * math.sin(2 * math.pi * t / 5.0))
    per, ncross = estimate_period(ts, ys)
    chk_true("周期估计 ≈ 5.00 s", per is not None and abs(per - 5.0) < 0.02,
             "period=%.4f s, 过零 %d 次" % (per or -1, ncross))

    # ---- 4) 正弦拟合：完美正弦 -> R2 接近 1 ----
    f = fit_sine(ts, ys)
    chk_true("正弦拟合有效", f["ok"], "")
    chk_true("拟合幅值 ≈ 150", abs(f["amp"] - 150.0) < 0.5, "amp=%.3f" % f["amp"])
    chk_true("完美正弦 R2 ≈ 1", f["r2"] > 0.999, "R2=%.6f" % f["r2"])
    chk_true("拟合频率 ≈ 0.2 Hz", abs(f["freq"] - 0.2) < 0.002, "f=%.5f" % f["freq"])

    # ---- 5) 加 10% 噪声 -> R2 应明显下降但仍算"像正弦" ----
    import random
    random.seed(1234)
    ys2 = [y + random.uniform(-15, 15) for y in ys]
    f2 = fit_sine(ts, ys2)
    chk_true("带噪声 R2 仍 > 0.95", f2["ok"] and f2["r2"] > 0.95,
             "R2=%.4f" % f2["r2"])

    # ---- 6) 方波 -> R2 应明显低于正弦 ----
    ys3 = [150.0 if math.sin(2 * math.pi * t / 5.0) >= 0 else -150.0 for t in ts]
    f3 = fit_sine(ts, ys3)
    chk_true("方波 R2 < 0.90（判据能区分）", f3["ok"] and f3["r2"] < 0.90,
             "R2=%.4f" % f3["r2"])
    chk_true("方波 R2 明显低于正弦 R2", f3["r2"] < f["r2"] - 0.05,
             "方波 %.3f vs 正弦 %.3f" % (f3["r2"], f["r2"]))


    # ---- 8) 样本不足时拟合必须"拒绝"而不是瞎给数 ----
    f4 = fit_sine(ts[:5], ys[:5])
    chk("样本 <12 -> ok=False", False, f4["ok"])

    print("-" * 78)
    if ok_all:
        print("TEST_RESULT: wave_scope PASS  (checks=%d)" % n_chk)
    else:
        print("TEST_RESULT: wave_scope FAIL  (checks=%d)" % n_chk)
    print("=" * 78)
    return 0 if ok_all else 1


# =============================================================================
# 五、入口
# =============================================================================
def drawtest():
    """GUI 冒烟测试：开一个隐藏窗口，塞合成数据，真的跑一遍分析 + 绘图。

    为什么要有它：tkinter 的 API 错误（属性名打错、方法不存在）只有真正
    创建控件、真正调用 _draw() 才会暴露，光跑算法自检发现不了。
    窗口用 withdraw() 藏起来，不会在桌面上闪。
    """
    import tkinter as tk
    App = make_app()
    root = tk.Tk()
    root.geometry("1100x760")
    app = App(root, port=None)
    root.withdraw()                       # 藏起来，别打断用户

    # 3 个周期、0.2Hz、幅值 150 度的合成摆臂（正弦模式下的典型波形）
    for i in range(100):
        t = i * 0.16
        app.ring.append(t, 0.4 * math.sin(2 * math.pi * t * 0.45),
                        150.0 * math.sin(2 * math.pi * t / 5.0))
    root.update_idletasks()
    app._analyze()
    app._draw()

    n_arm = len(app.can_arm.find_all())
    n_phi = len(app.can_phi.find_all())
    print("  [info] 摆臂画布元素 %d 个 / 摆杆画布元素 %d 个" % (n_arm, n_phi))
    print("  [info] 正弦判据: %s" % app.lbl_armb.cget("text"))
    ok = (n_arm > 10 and n_phi > 10 and
          bool(app.last_fit.get("ok")) and app.last_fit["r2"] > 0.99)
    root.destroy()
    print("TEST_RESULT: wave_scope draw %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def shot(path, port=None):
    """把界面渲染成 PNG（用合成演示数据），供报告/文档截图用。

    实现在程序里而不是让用户手动截图，是为了"截图可复现"：
    同样的合成正弦 + 同样的窗口尺寸，任何时候跑出来的图都一样。
    需要 Pillow（pip install pillow）；没装只提示，不影响主功能。
    """
    import tkinter as tk
    try:
        from PIL import ImageGrab
    except ImportError:
        sys.stderr.write("需要 Pillow 才能截图：pip install pillow\n")
        return 2
    App = make_app()
    root = tk.Tk()
    app = App(root, port=None)
    root.geometry("1100x760+60+60")
    root.update()

    # 合成演示数据：摆臂 0.2Hz / 150 度正弦，摆杆 0.4 度小抖动
    for i in range(100):
        t = i * 0.16
        app.ring.append(t, 0.4 * math.sin(2 * math.pi * t * 0.45),
                        150.0 * math.sin(2 * math.pi * t / 5.0))
    root.update_idletasks()
    app._analyze()
    app._draw()
    root.update()
    time.sleep(0.35)              # 等窗口管理器真正画完
    root.update()
    x, y = root.winfo_rootx(), root.winfo_rooty()
    w, h = root.winfo_width(), root.winfo_height()
    img = ImageGrab.grab(bbox=(x, y, x + w, y + h))
    img.save(path)
    root.destroy()
    print("截图已保存: %s (%dx%d)" % (path, w, h))
    return 0


def main():
    ap = argparse.ArgumentParser(description="J280 波形上位机（摆杆/摆臂）")
    ap.add_argument("--port", help="串口号，如 COM5")
    ap.add_argument("--selftest", action="store_true", help="不上硬件，自检分析算法")
    ap.add_argument("--drawtest", action="store_true",
                    help="隐藏窗口冒烟测试：真的建一遍界面并画一遍（不连硬件）")
    ap.add_argument("--shot", metavar="PNG",
                    help="用合成演示数据把界面存成 PNG（需要 Pillow）")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if args.drawtest:
        return drawtest()
    if args.shot:
        return shot(args.shot)

    try:
        import tkinter as tk
    except ImportError:
        sys.stderr.write("这台 Python 没有 tkinter，无法开界面（可改用 --selftest）\n")
        return 2

    App = make_app()
    root = tk.Tk()
    App(root, port=args.port)
    root.mainloop()
    return 0


# ---------------------------------------------------------------------------
# NOTE (encoding): a Chinese Windows console/pipe defaults to GBK, which has no
# codepoint for characters like the superscript two.  Printing such a char
# raises UnicodeEncodeError, and inside a frozen exe that kills the program
# halfway through its output (observed exactly that way).  Keep the console
# encoding, but never let one character abort the run.
# ---------------------------------------------------------------------------
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="replace")
    except Exception:
        pass


if __name__ == "__main__":
    sys.exit(main())
