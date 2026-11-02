# -*- coding: utf-8 -*-
"""
=============================================================================
 J280 旋转倒立摆 —— PC 端上位机（串口调试 + 在线调参）
-----------------------------------------------------------------------------
 为什么需要它：
   厂商提供的《接口驱动参考案例测试工具.exe》虽然也是 pyserial 串口 GUI
   （静态分析确认：PyInstaller 打包的 Python 3.12 + tkinter + pyserial，
    波特率下拉含 9600/19200/38400/57600/115200，9600 为默认），
   但它的"数据3"槽位是**接口二编码器值**，而本设计的遥测帧 w2 发的是
   **模式(0/1/2/3)**。字段语义不同，用它只能看到一串对不上的数字，
   而且它不提供本设计自定义的调参命令。

 功能：
   · 9600 8N1 连接，自动按帧头 AA 55 重同步（不依赖字节对齐）
   · 实时显示：摆杆角度(原始值 + 角度)、摆臂位置、当前模式
   · 键盘直接发调参命令（1..5 > < * / + - P R S U D）
   · 可选 CSV 记录，便于事后分析
   · --selftest 内置自检：喂入合成帧验证解码，不需要硬件
-----------------------------------------------------------------------------
 用法：
   python serial_host.py                 # 自动列出并选串口
   python serial_host.py --port COM5
   python serial_host.py --selftest      # 不上硬件，验证解码逻辑
=============================================================================
"""
import argparse
import os
import re
import shutil
import sys
import time
import threading

# ---- 协议常量（与 src/uart_link.v、src/pid_tuner.v 严格对应）----
SOF0, SOF1, EOF0, EOF1 = 0xAA, 0x55, 0x0D, 0x0A
FRAME_LEN = 28      # ★ v6.0: 22 -> 28（新增外环三个观测量 w9/w10/w11）
BAUD = 9600

# ADC 换算：★ 改用**用户实测标度**
#   实测：摆杆自然下垂 ≈1010，竖直向上 ≈3096
#   -> 180 度对应 2086 counts -> 每度 11.5889 counts
#   原写死 ADC_FULL=4090/ADC_DEG=345 (=11.8551 counts/度) 偏差 2.30%
ADC_FULL = 2086.0
ADC_DEG  = 180.0

MODE_NAME = {0: "空闲 IDLE", 1: "起摆 SWING", 2: "平衡 BALANCE", 3: "故障 FAULT"}

# 调参命令表（直接转发给 FPGA）
CMDS = {
    "1": "选 KP(倾角)", "2": "选 KI(倾角)", "3": "选 KD(倾角)",
    "4": "选 位置P", "5": "选 位置D",
    "6": "选 起摆幅值(SWING_DUTY)",
    ">": "当前系数 +12.5%", "<": "当前系数 -12.5%",
    "*": "当前系数 ×2", "/": "当前系数 ÷2",
    "+": "当前系数 +64 (从0起步用)", "-": "当前系数取反(纠极性)",
    "P": "翻转起摆能量泵极性",
    "R": "切换 运行/停止", "S": "切换 遥测开关",
    "U": "记录直立参考 up_ref", "D": "记录下垂参考 down_ref",
    # ★★ v6.3 新增：定点 / 定量运动 ★★
    #   这四个命令在 FPGA 里会【自动切到定点模式】，按一下就走，
    #   不需要用户先记得切模式。要退回"只平衡、目标=回中点"用 'A'。
    "A": "切换 定点模式(pos_mode)",
    "0": "把当前位置定义为定点零点",
    # 定点位置请用 "M" 进编辑模式 -> +/-/>/</* 调数值 -> "!" 发送；
    # ★★ v6.3 新增：位置指令数值输入 ★★
    #   ★ 关键：'M' 进入【位置指令模式】后，'>' '<' '*' '/' '+' '-' '0'
    #     调的是"待发送增量"，而【不再改 PID 系数】（固件侧做了门控）。
    #     所以上位机的 GainTracker 在位置模式下【必须停止推算系数】，
    #     否则面板会显示一个与实机不符的值（固件没改，上位机以为改了）。
    "M": "★ 进入 位置指令模式（之后 +-<>*/ 调的是要走的 counts）",
    "!": "★ 发送位置指令（把待发送值加到定点目标）",
    # ★★ v6.7 新增：外环积分时间常数 Ti 的在线调节 ★★
    #   这两个键不进位置指令模式的编辑集合，位置模式下也照样生效（与固件一致）。
    "T": "积分 Ti 变慢一档（i_shift+1，Ti×2）",
    "t": "积分 Ti 变快一档（i_shift-1，Ti÷2）",
}
QUIT_KEYS = ("q", "Q", "\x03")   # q / Ctrl-C

# ★ v6.3：位置指令模式下的编辑键（这些键在位置模式下【不改系数】）
POS_EDIT_KEYS = (">", "<", "*", "/", "+", "-", "0")
# 位置指令模式下一次发送的上限（与固件 pos_cmd 的 DELTA_MAX 及
# traj_ramp 的 TURN_LIMIT 保持一致：v6.4 起 3201 counts = 3 圈 @1067）
POS_DELTA_MAX = 3201
# ★ 一圈的 counts。★ 必须与固件 src/pendulum_top.v 里
#   u_traj_ramp 的 .CNT_PER_REV 一致。
#   沿革：v6.2 = 1078 / v6.3 = 1024（用户指定的设计值）/ v6.4 = 1067（用户给定）。
#   这个数只影响上位机的"圈数"换算显示，不影响固件行为。
CNT_PER_REV = 1067
# 摆臂绝对坐标基准：固件 qep_encoder 的 POS_INIT = 10000，
#   所以 绝对位置 = 10000 + x_settle。用户说的"现在显示 9999"就是这个坐标系。
POS_BASE = 10000

# =============================================================================
# ★★ v6.8 新增：板载 SW2（R18）= 正弦轨迹开关（**物理键，不走串口**）★★
# -----------------------------------------------------------------------------
#   固件实现：src\traj_sine.v（新模块）+ traj_ramp 的目标改成 x_settle + x_mod。
#   上位机【不需要改协议】：遥测帧还是 28 字节，命令字符一个都没动，
#   触发是硬件按键，PC 这边看不到也管不着。
#
#   但这两个常量必须与固件对上，否则"我以为五秒一个来回"是假的：
#     AMP        正弦幅值（编码器 counts，单边）
#     PHASE_INC  每个控制节拍（3ms）的相位增量，24bit = 一整周
#   频率换算： f = PHASE_INC / (2^24 * 3ms)
#     = 10066 / (16777216 * 0.003) = 0.19999 Hz  -> 周期 5.0002 s
#
#   ★ 为什么上位机要管这个：面板/文档里写"5 秒一周"是给操作者看的，
#     一旦有人在固件里改了 AMP 或 PHASE_INC 而这里没跟着改，
#     上位机的换算就变成错的数 —— 和 INIT 偏移是同一类"显示骗人"。
#     已加自检：直接读 src\pendulum_top.v 与 src\traj_sine.v 核对。
# =============================================================================
SINE_AMP       = 200
SINE_PHASE_INC = 10066
SINE_TICK_MS   = 3.0          # 控制节拍
SINE_PHASE_MOD = 1 << 24      # 24bit 相位累加器


def sine_freq_hz():
    """正弦轨迹的实际频率（Hz）—— 与固件同一公式"""
    return SINE_PHASE_INC / float(SINE_PHASE_MOD * (SINE_TICK_MS / 1000.0))


def sine_period_s():
    """正弦轨迹的实际周期（秒）"""
    return 1.0 / sine_freq_hz()


def sine_amp_deg():
    """正弦幅值换算成摆臂角度（度），用 CNT_PER_REV 换算"""
    return SINE_AMP * 360.0 / float(CNT_PER_REV)


def pos_edit_apply(cur, ch):
    """位置指令模式下，'+' '-' '>' '<' '*' '/' '0' 对【待发送增量】的运算。

    ★★★ 必须与固件 src/pos_cmd.v 的 pend_new 逐条一致 ★★★
        '>'   v + (v >>> 3)     有符号算术右移
        '<'   v - (v >>> 3)
        '*'   v << 1
        '/'   v >>> 1
        '+'   v + 64
        '-'   -v
        '0'   0                  （取消，不发送）
        其余键：原值返回，不动。
    之后统一饱和到 ±POS_DELTA_MAX（对应 RTL 的 pend_sat）。

    ★ 为什么把这段单独抽成函数：
      原先它内联在 kb_poll() 里，而 kb_poll() 要读键盘，
      自检根本没法覆盖它 —— 于是"上位机显示的值"和"固件实际的值"
      是否一致，在 v6.3 之前【没有任何自动检查】。
      抽出来之后 selftest 可以直接调用，两边算术对不上就会 FAIL。

    ★ Python 的 >> 对负数是向下取整（floor），与 Verilog 的 >>> 一致；
      所以 '>'/'<'/'/' 在负数上和 C 语言的 >> 语义相同，不需要额外补偿。
      这一点和 GainTracker.apply 里的处理方式保持一致。
    """
    if ch == '>':
        v = cur + (cur >> 3)
    elif ch == '<':
        v = cur - (cur >> 3)
    elif ch == '*':
        v = cur * 2
    elif ch == '/':
        v = cur // 2                     # 向下取整，等价于 >>> 1
    elif ch == '+':
        v = cur + 64
    elif ch == '-':
        v = -cur
    elif ch == '0':
        v = 0
    else:
        return cur

    # 饱和（与 RTL pend_sat 一致）
    if v > POS_DELTA_MAX:
        v = POS_DELTA_MAX
    elif v < -POS_DELTA_MAX:
        v = -POS_DELTA_MAX
    return v


class GainTracker:
    """按 RTL(pid_tuner.v) 完全相同的算术跟踪 5 个系数，供现场显示。

    为什么要它：**遥测帧只有 w0/w1/w2，没有系数回读**，
    所以调参时无法从串口确认系数变成了多少。上位机知道自己发过什么命令，
    用同一套运算复算即可推算当前值。
    ★ 这是**推算值**，不是从板上读回的。若某条命令丢失，会与实机不一致；
      重新上电（按 SW5 复位）后按 INIT 值重新起算即可对齐。
    """
    # 上位机按自己发过的命令推算系数，无法从板上回读；INIT 必须与
    # pendulum_top.v 的 kp_th_w / ki_th_w / kd_th_w / kp_x_w / kd_x_w / swing_duty_w32
    # 完全一致，否则面板显示值会带一个恒定偏移。改完请跑 tools/check_init_consistency.py。
    INIT = {'KP': 15507, 'KI': 256, 'KD': 2034, 'PP': -845, 'PD': -21730, 'SW': 3700}
    NAMES = {'1': 'KP', '2': 'KI', '3': 'KD', '4': 'PP', '5': 'PD', '6': 'SW'}

    # ★★★ 饱和上下界必须与 RTL(pid_tuner.v 的 f_sat) 完全一致 ★★★
    #   v5.0 起 RTL 系数已加宽到 32bit，饱和界为 ±2147483647/-2147483648。
    #   此处曾长期停留在 16bit(±GainTracker.SAT_MAX)，导致按 '>'/'*' 把系数加到 GainTracker.SAT_MAX 后
    #   面板永远不再增长（显示 32.000），而板上其实仍在正常变化 —— 面板在骗人。
    SAT_MAX = 2147483647
    SAT_MIN = -2147483648

    @staticmethod
    def _sat(v):
        # 与 RTL f_sat 一致：夹到 32bit 有符号满量程
        return max(GainTracker.SAT_MIN, min(GainTracker.SAT_MAX, v))

    def __init__(self):
        self.reset()
        self.sel = 'KP'

    def reset(self):
        self.g = dict(self.INIT)
        # ★ v6.7：积分右移档位，与固件 pid_tuner.v 的 I_SHIFT_INIT 同源
        self.i_shift = self.I_SHIFT_INIT

    # ★ v6.7 新增：积分时间常数 Ti 的在线档位
    #   固件里是 pos_pd 的 i_shift 端口，由串口 'T'/'t' 调：
    #       'T' -> i_shift + 1 -> Ti 翻倍（积分更慢）
    #       't' -> i_shift - 1 -> Ti 减半（积分更快）
    #   范围 [8,15] 与固件 pid_tuner.v 的钳位一致。
    #   ★ Ti = 2^i_shift × |PP| / 4096 × 30ms —— 注意它【随 PP 变】！
    #     PP 减半 Ti 也减半，所以面板必须按当前 PP 实时算，不能写死 13.5s。
    I_SHIFT_INIT = 11
    I_SHIFT_MIN = 8
    I_SHIFT_MAX = 15
    OUTER_TS_MS = 30.0        # 外环节拍：内环 3ms × DIV(10)

    def ti_ms(self):
        """当前积分时间常数，单位 ms（上位机推算值，与固件同一公式）"""
        pp = abs(self.g['PP'])
        return (2.0 ** self.i_shift) * pp / 4096.0 * self.OUTER_TS_MS

    def ti_s(self):
        return self.ti_ms() / 1000.0

    def i_shift_step(self, ch):
        """'T' 变慢一档 / 't' 变快一档，返回新档位（到界不动，与固件一致）"""
        if ch == 'T' and self.i_shift < self.I_SHIFT_MAX:
            self.i_shift += 1
        elif ch == 't' and self.i_shift > self.I_SHIFT_MIN:
            self.i_shift -= 1
        return self.i_shift

    def apply(self, ch):
        if ch in self.NAMES:
            self.sel = self.NAMES[ch]
            return None
        # ★ v6.7：Ti 档位必须在下面的通用缩放分支之前拦截（'T'/'t' 不是系数）
        if ch in ('T', 't'):
            self.i_shift_step(ch)
            return ('Ti', self.i_shift)
        if ch not in '><*/+-':
            return None
        v = self.g[self.sel]
        # ★ Python 的 >> 对负数是向下取整，与 Verilog 的 >>> 算术右移一致
        if ch == '>':
            v = self._sat(v + (v >> 3))
        elif ch == '<':
            v = self._sat(v - (v >> 3))
        elif ch == '*':
            v = self._sat(v << 1)
        elif ch == '/':
            v = self._sat(v >> 1)
        elif ch == '+':
            v = self._sat(v + 64)
        elif ch == '-':
            v = self._sat(-v)
        self.g[self.sel] = v
        return (self.sel, v)

    def table(self):
        out = []
        for k in ('KP', 'KI', 'KD', 'PP', 'PD', 'SW'):
            v = self.g[k]
            out.append("%s=%6d(%7.3f)%s" % (k, v, v / 1024.0, ' <=' if k == self.sel else '   '))
        # ★ v6.7：Ti 单列一行 —— 它是"推算值"，且随 PP 变化，必须让用户看见
        out.append("| Ti=%6.2f s (i_shift=%d, 按当前 PP=%d 推算)"
                   % (self.ti_s(), self.i_shift, self.g['PP']))
        return "  ".join(out)


class FrameDecoder:
    """按 AA 55 ... 0D 0A 滑动同步解码，容忍丢字节与错位"""

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.bad = 0

    def feed(self, data: bytes):
        """喂入原始字节，产出解析好的帧 (w0, w1, w2)"""
        self.buf.extend(data)
        out = []
        while True:
            # 找帧头
            i = self.buf.find(bytes([SOF0, SOF1]))
            if i < 0:
                # 没找到帧头：只保留最后 1 字节（可能是 AA 的后半）
                if len(self.buf) > 1:
                    del self.buf[:-1]
                    self.bad += 1
                return out
            if i > 0:
                del self.buf[:i]
                self.bad += 1            # 丢掉的字节记为失步
            if len(self.buf) < FRAME_LEN:
                return out               # 等更多数据
            f = bytes(self.buf[:FRAME_LEN])
            # ★ v6.0：帧尾从 byte20/21 移到 byte26/27（帧长 28 字节）
            if f[26] == EOF0 and f[27] == EOF1:
                w0 = (f[2] << 8) | f[3]
                w1 = (f[4] << 8) | f[5]
                w2 = (f[6] << 8) | f[7]
                # ★ v3.0.2: byte8/9 = u_motor（有符号 16bit）
                u = (f[8] << 8) | f[9]
                if u >= 32768:
                    u -= 65536
                # ★ v4.4: byte10..19 = phi_eff / dphi / p_term / d_term / pd_sum
                def s16(i):
                    v = (f[i] << 8) | f[i + 1]
                    return v - 65536 if v >= 32768 else v
                phieff = s16(10)
                dphi   = s16(12)
                pterm  = s16(14)
                dterm  = s16(16)
                pdsum  = s16(18)
                # ★ v6.0: byte20..25 = x_settle / err_x / phi_bias（外环三个量）
                #   这三个是"外环到底有没有在工作"的唯一直接证据：
                #     xset  定点目标（编码器 counts）
                #     errx  位置偏差（编码器 counts）
                #     phib  倾角偏置（ADC counts，= 外环送进内环的量）
                #   phib 乘 g/R 就是被命令的摆臂角加速度，若它长期贴着 ±250
                #   不动或恒为 0，就是外环增益不对/没接上。
                xset   = s16(20)
                errx   = s16(22)
                phib   = s16(24)
                out.append((w0, w1, w2, u, phieff, dphi, pterm, dterm, pdsum,
                            xset, errx, phib))
                self.frames += 1
                del self.buf[:FRAME_LEN]
            else:
                # 帧尾不对 → 只丢 1 个字节，继续找下一个帧头
                del self.buf[:1]
                self.bad += 1


def adc_to_deg(raw):
    return raw * ADC_DEG / ADC_FULL



# ============================================================================
# ★ Panel：固定面板显示
#   用户要求"面板的标签固定、只有数据变化"。
#   做法：多行面板 + ANSI 光标控制。
#     · 首帧：正常打印 N 行
#     · 之后：光标上移 N 行 -> 逐行 "\r" + 清行 + 内容
#     · 每行按终端宽度截断，绝不折行（折行会让 \r 失效、界面滚动）
#     · 消息：先擦掉面板、打印消息，再标记需要整体重绘
# ============================================================================

# ============================================================================
# ★ 关闭 Windows 控制台的"快速编辑模式"
#   开启时，用户在窗口里点一下鼠标，程序就会被挂起（等待选择完成），
#   表现为键盘无响应、串口不刷新 —— 像死机。
#   这是控制台默认行为，与固件无关，所以"一直存在"。
#   这里在启动时程序化关掉它，用户不用手动改控制台属性。
# ============================================================================
def disable_console_quickedit():
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        # STD_INPUT_HANDLE = -10
        h = kernel32.GetStdHandle(-10)
        if h in (0, -1):
            return False
        mode = ctypes.c_uint()
        if not kernel32.GetConsoleMode(h, ctypes.byref(mode)):
            return False
        ENABLE_QUICK_EDIT = 0x0040
        ENABLE_EXTENDED  = 0x0080
        # 清掉快速编辑位，并确保扩展标志开着（否则 GetConsoleMode 会失败）
        newmode = (mode.value & ~ENABLE_QUICK_EDIT) | ENABLE_EXTENDED
        return bool(kernel32.SetConsoleMode(h, newmode))
    except Exception:
        return False


class Panel:
    def __init__(self):
        self.nlines = 0
        self.drawn = False

    @staticmethod
    def width():
        try:
            w = shutil.get_terminal_size().columns
        except Exception:
            w = 120
        return max(40, w - 1)

    def _wipe(self):
        # 光标回到面板首行并逐行清空
        if self.drawn and self.nlines:
            sys.stdout.write("\x1b[%dA" % self.nlines)
            for _ in range(self.nlines):
                sys.stdout.write("\r\x1b[K\n")
            sys.stdout.write("\x1b[%dA" % self.nlines)
            self.drawn = False

    def render(self, lines):
        w = self.width()
        lines = [l[:w] for l in lines]           # ★ 截断，防止折行
        out = []
        if self.drawn and self.nlines:
            out.append("\x1b[%dA" % self.nlines)  # 光标上移到面板首行
        for l in lines:
            out.append("\r\x1b[K" + l + "\n")   # 清行 + 写内容
        sys.stdout.write("".join(out))
        sys.stdout.flush()
        self.nlines = len(lines)
        self.drawn = True

    def message(self, text):
        # 清掉面板 -> 打印消息 -> 下次 render 会重新画整个面板
        self._wipe()
        sys.stdout.write(text + "\n")
        sys.stdout.flush()
        self.drawn = False
        self.nlines = 0

def fmt_line(w0, w1, w2, u=0, phieff=0, dphi=0, pterm=0, dterm=0, pdsum=0,
             xset=0, errx=0, phib=0, up_ref=None):
    """up_ref 已知时额外显示 phi（相对直立的偏差）——遥测帧里没有 phi，
       这是上位机按 angle_raw - up_ref 自行算出来的。

    ★ v6.0：新增外环三个量 xset / errx / phib 的显示。
      它们放在单独一行（OUTER），因为这一行才是判断"定点功能是否正常"的依据：
        · xset  定点目标：按 '9' 后应跳到 +1078，按 '0' 后应等于当前位置
        · errx  位置偏差：到位后应趋近 0
        · phib  倾角偏置：外环实际送进内环的量。摆臂动时应反向摆动；
                长期贴着 ±250 不动 = 外环饱和；恒为 0 = 外环没在工作。
    """
    base = ("w0=%4d  w1=%5d  w2=%d %-8s | phiE=%5d dphi=%4d | P=%6d D=%6d PD=%6d | u=%6d"
            % (w0, w1, w2, MODE_NAME.get(w2, "?"), phieff, dphi,
               pterm, dterm, pdsum, u))
    if up_ref is not None:
        phi = w0 - up_ref
        base += "  phi=%+5d(%+6.1f°)" % (phi, phi * ADC_DEG / ADC_FULL)
    else:
        base += "  phi=未标定(先按 U 记直立参考)"
    return base


def fmt_line_outer(xset, errx, phib, cpr=1078.0):
    """外环一行。cpr = 编码器每圈 counts，用于把 errx 换算成"度/圈"。
       ★ 1078 是默认口径；若你实测的一圈 counts 不同，改这里或加命令行参数。"""
    deg = errx * 360.0 / cpr if cpr else 0.0
    # phi_bias 的物理含义：1 count 倾角 ≈ (g/R)*(2π/4096) = 64.7*1.534e-3 = 0.0992 rad/s²
    #   所以 250 counts 满量程 ≈ 24.8 rad/s² 的摆臂角加速度上限。
    return ("OUTER | xset=%6d errx=%6d (%+7.2f°) phib=%+5d %s"
            % (xset, errx, deg, phib,
               "饱和!" if abs(phib) >= 250 else ("外环关" if phib == 0 else "")))


# ---------------------------------------------------------------------------
# 自检：不需要硬件，验证解码器与换算
# ---------------------------------------------------------------------------
def selftest():
    print("=" * 70)
    print(" 自检模式：用合成数据验证帧解码（不需要硬件）")
    print("=" * 70)
    ok = True

    def chk(name, exp, got):
        nonlocal ok
        good = exp == got
        ok = ok and good
        print("  [%s] %-42s expected=%s got=%s"
              % ("OK" if good else "FAIL", name, exp, got))

    d = FrameDecoder()

    # 1) 标准帧（★ v6.0：28 字节）
    #    AA 55 | w0=0C4C | w1=275F | w2=0002 | u=03E8 | phiE=0064 | dphi=FF9C
    #          | P=012C | D=FF38 | PD=00C8 | xset=0436 | errx=FFFB | phib=001E
    #          | 0D 0A
    #    数值：角度3148, 位置10079, 模式2, u=1000, phiE=100, dphi=-100,
    #          P=300, D=-200, PD=200, xset=1078(一圈), errx=-5, phib=+30
    f = bytes([0xAA, 0x55, 0x0C, 0x4C, 0x27, 0x5F, 0x00, 0x02, 0x03, 0xE8,
               0x00, 0x64, 0xFF, 0x9C, 0x01, 0x2C, 0xFF, 0x38, 0x00, 0xC8,
               0x04, 0x36, 0xFF, 0xFB, 0x00, 0x1E,
               0x0D, 0x0A])
    r = d.feed(f)
    chk("标准帧解出 1 帧", 1, len(r))
    chk("w0 角度 = 3148", 3148, r[0][0])
    chk("w1 位置 = 10079", 10079, r[0][1])
    chk("w2 模式 = 2", 2, r[0][2])
    chk("u_motor = 1000", 1000, r[0][3])
    # ★ v6.0 新字段
    # ★ 标签里不要写"一圈"：1078 只是本自检帧里的合成数值，
    #   与 CNT_PER_REV 无关，写"一圈"会误导。
    chk("x_settle = 1078", 1078, r[0][9])
    chk("err_x = -5", -5, r[0][10])
    chk("phi_bias = +30", 30, r[0][11])

    # 2) 逐字节喂入（模拟串口碎片），仍应解出同一帧
    d2 = FrameDecoder()
    got = []
    for byte in f:
        got += d2.feed(bytes([byte]))
    chk("逐字节喂入仍解出 1 帧", 1, len(got))
    chk("逐字节解出内容一致",
        (3148, 10079, 2, 1000, 100, -100, 300, -200, 200, 1078, -5, 30),
        got[0] if got else None)

    # 3) 前面有垃圾字节
    d3 = FrameDecoder()
    r3 = d3.feed(b"\x11\x22\x33" + f)
    chk("前置垃圾字节后仍解出 1 帧", 1, len(r3))
    chk("前置垃圾后内容仍正确",
        (3148, 10079, 2, 1000, 100, -100, 300, -200, 200, 1078, -5, 30),
        r3[0] if r3 else None)

    # 4) 两帧粘连
    d4 = FrameDecoder()
    r4 = d4.feed(f + f)
    chk("两帧粘连解出 2 帧", 2, len(r4))

    # 5) 帧尾损坏后能重新同步
    d5 = FrameDecoder()
    bad = bytearray(f); bad[26] = 0x00   # ★ v6.0: 帧尾在 byte26/27（v4.4 是 20/21）
    r5 = d5.feed(bytes(bad) + f)
    chk("帧尾损坏后重新同步到下一帧", 1, len(r5))

    # 6) 角度换算抽查
    #    ★ 期望值**从 ADC_FULL/ADC_DEG 自动推算**，不硬编码 ——
    #      否则一改标度常数自检就失效（那是"测试过期"不是"程序出错"）。
    e3148 = round(3148 * ADC_DEG / ADC_FULL, 1)
    e1015 = round(1015 * ADC_DEG / ADC_FULL, 1)
    chk("ADC 3148 → %.1f°" % e3148, e3148, round(adc_to_deg(3148), 1))
    chk("ADC 1015 → %.1f°" % e1015, e1015, round(adc_to_deg(1015), 1))
    chk("帧内 w0 换算一致", e3148, round(adc_to_deg(3148), 1))

    # 7) ★ 系数跟踪算术（必须与 pid_tuner.v 逐位一致）
    #    ★ 期望值**从 GainTracker.INIT 自动推算**，不硬编码 ——
    #      否则一改固件初值（如 KP 256->12410）自检就会失效，
    #      而那是"测试过期"而非"程序出错"，容易误判。
    def sat(v):
        # ★ 必须与 GainTracker._sat 用同一组上下界。
        #   这里曾写死 16 位（-32768/32767），而 GainTracker 已加宽到 32 位，
        #   导致自检期望值算错、报出"25206 翻倍 = 32767"这类假失败。
        #   与之前"30000 翻倍"那次同类：测试把旧行为固化成了期望值。
        return max(GainTracker.SAT_MIN, min(GainTracker.SAT_MAX, v))

    gt = GainTracker()
    K0 = GainTracker.INIT['KP']
    chk("INIT 与固件一致: KP=%d" % K0, K0, gt.g['KP'])
    gt.apply('1')                      # 选 KP
    e = sat(K0 + (K0 >> 3))
    gt.apply('>'); chk("KP %d 的 +12.5%% = %d" % (K0, e), e, gt.g['KP'])
    e2 = sat(e << 1)
    gt.apply('*'); chk("%d 翻倍 = %d" % (e, e2), e2, gt.g['KP'])
    e3 = e2 >> 1
    gt.apply('/'); chk("%d 减半 = %d" % (e2, e3), e3, gt.g['KP'])
    e4 = sat(-e3)
    gt.apply('-'); chk("%d 取反 = %d" % (e3, e4), e4, gt.g['KP'])
    gt.apply('-'); chk("再取反回 %d" % e3, e3, gt.g['KP'])
    for _ in range(20):
        gt.apply('/')
    chk("连按 / 后归零（旧版死锁点）", 0, gt.g['KP'])
    gt.apply('+'); chk("★ 从 0 按 '+' 得 64（旧版永久卡死）", 64, gt.g['KP'])
    # 负数取反（RTL 里 -1000 的 +12.5% 应为 -1125）
    gt2 = GainTracker(); gt2.g['KI'] = -1000; gt2.sel = 'KI'
    gt2.apply('>'); chk("KI -1000 的 +12.5% = -1125", -1125, gt2.g['KI'])
    gt2.apply('<'); chk("再 -12.5% = -984（-1125>>>3=-141）", -984, gt2.g['KI'])
    # 饱和
    gt3 = GainTracker(); gt3.g['KP'] = 30000; gt3.sel = 'KP'
    # ★ 这条自检的意图是"验证饱和逻辑"，但原来写死期望 32767 ——
    #   等于把"系数只能加到 16bit 上限"这个 bug 固化进了自检里，
    #   所以自检一直 PASS 却掩盖了真实缺陷。现改为分两步验证：
    gt3.apply('*'); chk("30000 翻倍 = 60000（32bit 下不提前饱和）", 60000, gt3.g['KP'])
    for _k in range(30):
        gt3.apply('*')
    chk("持续翻倍最终饱和到 SAT_MAX", GainTracker.SAT_MAX, gt3.g['KP'])
    # 负数算术右移语义（与 Verilog >>> 一致）
    gt4 = GainTracker(); gt4.g['KD'] = -1125; gt4.sel = 'KD'
    gt4.apply('/'); chk("-1125 减半 = -563（floor，不是 -562）", -563, gt4.g['KD'])

    # 固件已删除外环积分（pos_pd 是纯 PD），Ti 档位在固件里不存在，本组比对跳过。
    print("  [--] 固件无外环积分（纯 PD），跳过 Ti 档位自检")
    # ★★ v6.8 新增：正弦轨迹的换算自检（与固件同一公式，纯算术）★★
    #   比较用"放大成整数"的方式，避免浮点相等判断在边界上翻车。
    chk("正弦幅值 200 counts 换算角度 ≈ 67.5 度",
        675, int(round(sine_amp_deg() * 10)))
    chk("正弦周期 ≈ 5.00 s（5000 ms）",
        5000, int(round(sine_period_s() * 1000)))
    chk("正弦频率 ≈ 0.200 Hz",
        200, int(round(sine_freq_hz() * 1000)))
    # 设计规则 A*f <= 100（斜坡不削顶 + 外环不饱和，见 traj_sine.v 文件头）
    chk("A*f <= 100（斜坡不削顶的硬约束）",
        1, 1 if (SINE_AMP * sine_freq_hz() <= 100.0) else 0)

    # 8) ★★ v6.3 位置指令算术（必须与 pos_cmd.v 的 pend_new 逐条一致）★★
    #    这一节是 v6.3 的核心新增功能，也是**上位机与固件最容易走偏**的地方：
    #    上位机显示"待发送 +3001"，固件实际算出来的却是别的数，
    #    上板后表现就是"我输的距离和它走的距离不一样"，且很难查。
    #    所以每条运算都在这里钉死一个期望值。
    chk("位置模式：从 0 按 '+' = 64", 64, pos_edit_apply(0, '+'))
    chk("位置模式：64 按 '>' = 72（64+8）", 72, pos_edit_apply(64, '>'))
    chk("位置模式：72 按 '*' = 144", 144, pos_edit_apply(72, '*'))
    chk("位置模式：144 按 '/' = 72", 72, pos_edit_apply(144, '/'))
    chk("位置模式：1000 按 '-' = -1000", -1000, pos_edit_apply(1000, '-'))
    chk("位置模式：-1000 再按 '-' 回 1000", 1000, pos_edit_apply(-1000, '-'))
    chk("位置模式：任意值按 '0' = 0（取消）", 0, pos_edit_apply(1234, '0'))
    # 负数算术右移：-1000>>>3 = -125（floor），所以 '>' 得 -1125
    chk("位置模式：-1000 按 '>' = -1125（负数是 floor）",
        -1125, pos_edit_apply(-1000, '>'))
    chk("位置模式：-1125 按 '<' = -984（-1125>>>3=-141）",
        -984, pos_edit_apply(-1125, '<'))
    # 减半到 0 与负数的 floor
    chk("位置模式：1 按 '/' = 0", 0, pos_edit_apply(1, '/'))
    chk("位置模式：-1 按 '/' = -1（floor，不是 0）", -1, pos_edit_apply(-1, '/'))
    # 饱和到 ±POS_DELTA_MAX（= 3 圈）
    chk("位置模式：3000 按 '*' 饱和到 %d" % POS_DELTA_MAX,
        POS_DELTA_MAX, pos_edit_apply(3000, '*'))
    chk("位置模式：-3000 按 '*' 饱和到 %d" % (-POS_DELTA_MAX),
        -POS_DELTA_MAX, pos_edit_apply(-3000, '*'))
    chk("位置模式：已是上限再按 '>' 仍为 %d" % POS_DELTA_MAX,
        POS_DELTA_MAX, pos_edit_apply(POS_DELTA_MAX, '>'))
    # 非编辑键不动值
    chk("位置模式：非编辑键 'x' 不改值", 777, pos_edit_apply(777, 'x'))

    # 常量自洽：单次上限 == traj_ramp 的 TURN_LIMIT == 3 圈
    chk("POS_DELTA_MAX 换算圈数 = 3.0",
        3.0, POS_DELTA_MAX / float(CNT_PER_REV))
    chk("POS_BASE 与固件 qep_encoder 的 POS_INIT 一致 = 10000",
        10000, POS_BASE)

    # ★★ 源码交叉核对 ★★
    #   常量是"人抄"过来的（3072 / 1024 / 10000 都写了两份），
    #   手抄就会走偏。这里直接读固件源码把数字抽出来比对；
    #   找不到源码时只提示 SKIP，不算失败（比如只拷了 tools/ 到别处运行）。
    #
    #   ★ 冻结成 exe 后 __file__ 指向临时解包目录(_MEIxxxx)，那里的 ../src 不存在，
    #     必须改用 sys.executable 的所在目录 —— 发布布局是
    #     J280_v6.3\上位机\J280_Host.exe，其上一级的 src\ 正是固件源码。
    #     不改这一处的话，exe 版自检会永远"跳过"交叉核对，
    #     等于发布版少了这道把关。
    if getattr(sys, "frozen", False):
        _base = os.path.dirname(os.path.abspath(sys.executable))
    else:
        _base = os.path.dirname(os.path.abspath(__file__))
    _cands = [os.path.join(os.path.dirname(_base), "src"),   # <pkg>\上位机\x -> <pkg>\src
              os.path.join(_base, "src")]                    # <pkg>\x      -> <pkg>\src
    _src = None
    for _c in _cands:
        if os.path.isfile(os.path.join(_c, "pos_cmd.v")) and \
           os.path.isfile(os.path.join(_c, "pendulum_top.v")):
            _src = _c
            break
    if _src is not None:
        _pc = os.path.join(_src, "pos_cmd.v")
        _top = os.path.join(_src, "pendulum_top.v")
        _txt = open(_pc, "r", encoding="utf-8", errors="replace").read()
        _m = re.search(r"DELTA_MAX\s*=\s*16'sd(\d+)", _txt)
        chk("pos_cmd.v 的 DELTA_MAX 与上位机一致",
            POS_DELTA_MAX, int(_m.group(1)) if _m else None)
        _t2 = open(_top, "r", encoding="utf-8", errors="replace").read()
        _m2 = re.search(r"\.CNT_PER_REV\s*\(\s*(\d+)\s*\)", _t2)
        if _m2 is None:
            print("  [--] 固件无 .CNT_PER_REV（定点只走数值增量），跳过该项")
        else:
            chk("pendulum_top.v 的 CNT_PER_REV 与上位机一致",
                CNT_PER_REV, int(_m2.group(1)))
        _m3 = re.search(r"\.TURN_LIMIT\s*\(\s*16'sd(\d+)\s*\)", _t2)
        chk("pendulum_top.v 的 TURN_LIMIT == POS_DELTA_MAX",
            POS_DELTA_MAX, int(_m3.group(1)) if _m3 else None)

        # ---------------------------------------------------------------------
        # ★★ v6.4：6 个 PID 初值也要与固件逐一核对 ★★
        #   为什么必须查这一项：
        #     pid_tuner 的系数【没有任何非易失存储】，复位值就是这些 INIT。
        #     上位机的 GainTracker 靠自己的 INIT + 发过的命令【推算】当前系数
        #     （遥测帧没有系数回读）。一旦两边 INIT 不一致，
        #     面板就会长期显示一个与实机不符的值 —— 也就是"面板在骗人"，
        #     用户会照着错数字去调参。
        #   这正是 v6.4 要防的事：把 -18000 写进固件时，
        #     上位机这份 INIT 必须同步改成 -18000。
        #
        #   ★ 解析注意 1：负数在 RTL 里的合法写法是  -32'sd18000
        #     （符号在尺寸说明符之外），所以正则要把前导 '-' 一起抓。
        #     写成 32'sd-18000 是不合法的。
        #   ★ 解析注意 2：6 个初值【不一定都在顶层】。
        #     前 5 个由 pendulum_top 实例化时覆盖；
        #     而 SWING_INIT 没有在顶层覆盖，它的真值来源是
        #     pid_tuner.v 里 parameter 的模块默认值 32'd2600。
        #     所以必须【先查顶层、再查模块默认】，
        #     只查顶层会让 SW 这一项永远取不到值（自检会报 got=None）。
        # ---------------------------------------------------------------------
        _pid = os.path.join(_src, "pid_tuner.v")
        _t3 = ""
        if os.path.isfile(_pid):
            _t3 = open(_pid, "r", encoding="utf-8", errors="replace").read()
        for _name, _pname in (("KP", "KP_TH_INIT"), ("KI", "KI_TH_INIT"),
                              ("KD", "KD_TH_INIT"), ("PP", "KP_X_INIT"),
                              ("PD", "KD_X_INIT"), ("SW", "SWING_INIT")):
            # 顶层覆盖写成 .XXX_INIT (-32'sd18000)；
            # 模块默认写成    parameter [31:0] XXX_INIT = 32'd2600
            _val = None
            for _t in (_t2, _t3):
                _mm = re.search(_pname + r"\s*[=(]\s*(-?)32'?s?d(\d+)", _t)
                if _mm:
                    _val = int(_mm.group(2))
                    if _mm.group(1) == '-':
                        _val = -_val
                    break
            chk("固件 %s_INIT 与上位机 INIT 一致" % _name,
                GainTracker.INIT[_name], _val)

        # ---------------------------------------------------------------------
        # ★★ v6.7 新增：积分档位 I_SHIFT_INIT 也要核对 ★★
        #   Ti = 2^i_shift × |PP| / 4096 × 30ms，i_shift 初值两边不一致时，
        #   面板显示的 Ti 会系统性偏移 —— 和 6 个系数偏移是同一类"显示骗人"。
        #   ★ 它的写法与前 6 项不同，不能复用上面那条 32'sd 正则：
        #       顶层覆盖  .I_SHIFT_INIT (11)              <- 没有 32'sd
        #       模块默认  parameter integer I_SHIFT_INIT = 11
        #   所以这里单独写两条。
        # ---------------------------------------------------------------------
        _val_sh = None
        _mm_sh = re.search(r"\.I_SHIFT_INIT\s*\(\s*(\d+)\s*\)", _t2)
        if _mm_sh is None:
            _mm_sh = re.search(r"parameter\s+integer\s+I_SHIFT_INIT\s*=\s*(\d+)", _t3)
        if _mm_sh is not None:
            _val_sh = int(_mm_sh.group(1))
        if _val_sh is None:
            print("  [--] 固件无 I_SHIFT_INIT（外环是纯 PD），跳过该项")
        else:
            chk("固件 I_SHIFT_INIT（积分档位初值）与上位机一致",
                GainTracker.I_SHIFT_INIT, _val_sh)

        # ---------------------------------------------------------------------
        # ★★ v6.8 新增：正弦轨迹的 AMP / PHASE_INC 也要核对 ★★
        #   这两个常量决定"摆臂摆多大、几秒一个来回"，是给人看的数。
        #   固件改了这里不改，上位机换算出来的周期/角度就是错的
        #   （与 INIT 偏移同一类"显示骗人"）。
        #   顶层覆盖写法：  .AMP (200)        .PHASE_INC (24'd10066)
        #   模块默认写法：  parameter integer AMP = 200
        #                   parameter [23:0]  PHASE_INC = 24'd10066
        # ---------------------------------------------------------------------
        _ts = os.path.join(_src, "traj_sine.v")
        _t4 = ""
        if os.path.isfile(_ts):
            _t4 = open(_ts, "r", encoding="utf-8", errors="replace").read()
        _a = re.search(r"\.AMP\s*\(\s*(\d+)\s*\)", _t2)
        if _a is None:
            _a = re.search(r"parameter\s+integer\s+AMP\s*=\s*(\d+)", _t4)
        chk("固件 AMP（正弦幅值 counts）与上位机一致",
            SINE_AMP, int(_a.group(1)) if _a else None)
        _pinc = re.search(r"\.PHASE_INC\s*\(\s*24'd(\d+)\s*\)", _t2)
        if _pinc is None:
            _pinc = re.search(r"PHASE_INC\s*=\s*24'd(\d+)", _t4)
        chk("固件 PHASE_INC（正弦相位增量）与上位机一致",
            SINE_PHASE_INC, int(_pinc.group(1)) if _pinc else None)

        print("  [--] 源码交叉核对：已读取 %s" % _src)
    else:
        print("  [--] 源码交叉核对：未找到固件 src\\（试过 %s），跳过"
              % " , ".join(_cands))

    print()
    print("=" * 70)
    print(" 自检结果：%s" % ("全部通过 PASS" if ok else "存在失败 FAIL"))
    print("=" * 70)
    print()
    print("--- 显示格式预览 ---")
    print("  " + fmt_line(3148, 10079, 2))
    print("  " + fmt_line(1015, 10000, 0))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# 键盘输入线程（Windows 用 msvcrt，其它平台用 stdin 行缓冲）
# ---------------------------------------------------------------------------
def kb_poll(send_fn, state):
    """非阻塞地检查一次键盘并处理。
    ★ 由主循环每轮调用一次（不再单独开线程）——
      原先按键在独立线程里抢同一把串口锁，很难抢到，表现为"按了没反应"；
      且 Panel 被两个线程同时写，状态竞争导致画面错乱。改单线程后这两个问题都消失。"""
    try:
        import msvcrt
    except ImportError:
        msvcrt = None

    if msvcrt is None:
        # 非 Windows：退回按行读取（阻塞式，但至少可用）
        line = sys.stdin.readline()
        if not line:
            state["quit"] = True
            return
        ch = line.strip()[:1]
    else:
        if not msvcrt.kbhit():
            return                      # 没有按键就立刻返回，不阻塞
        raw = msvcrt.getch()
        try:
            ch = raw.decode("ascii")
        except UnicodeDecodeError:
            return

    if ch in QUIT_KEYS:
        state["quit"] = True
        return

    if ch in ('g', 'G'):
        panel_show_gains(state)
        return

    # ★★ v6.3：'!' 直接发送当前待发送值 ★★
    if ch == '!':
        send_fn('!')
        prev = state.get("pending", 0)
        state["pending"] = 0
        state["msg"] = ("  >> 已发送 '!'  —— 位置指令 %+d counts (%.2f 圈) 已发出，"
                        "待发送值清 0\n     （未进入位置指令模式时按 '!' 是空操作，"
                        "先按 'M'）"
                        % (prev, prev / float(state.get("cnt_per_rev", CNT_PER_REV))))
        return

    # ★★ v6.7：积分 Ti 档位（'T' 更慢 / 't' 更快）★★
    #   固件侧 pid_tuner 把它们译成 i_shift ±1 并钳位到 [8,15]；
    #   上位机用同一套规则推算，面板才能显示与实机一致的 Ti。
    #   ★ Ti 随 PP 变（Ti = 2^shift × |PP|/4096 × 30ms），所以提示里带上当前 PP。
    if ch in ('T', 't'):
        send_fn(ch)
        state["gt"].apply(ch)
        state["msg"] = ("  >> 已发送 '%s'  -> i_shift=%d, Ti=%.2f s\n"
                        "     （按当前 PP=%d 推算；PP 一改 Ti 会跟着变 —— "
                        "PP 减半要让 i_shift +1 才能保住 Ti）"
                        % (ch, state["gt"].i_shift, state["gt"].ti_s(),
                           state["gt"].g['PP']))
        return

    if ch in CMDS:
        send_fn(ch)
        note = CMDS[ch]
        # ★★ v6.3：位置指令模式下的门控 —— 必须与固件行为一致 ★★
        #   固件在 sel_loop==6（位置指令模式）时【不写系数】。
        #   若上位机不同步、照样用 GainTracker 推算系数，
        #   面板就会显示一个与实机不符的值 —— 这是"显示骗人"。
        if ch == 'M':
            state["pos_mode"] = True
            state["pending"] = 0
            note += "  -> 已进入位置指令模式，待发送值 = 0；此时 +-<>*/ 调的是待发送值"
        elif ch in ("1", "2", "3", "4", "5", "6"):
            state["pos_mode"] = False
            state["gt"].sel = GainTracker.NAMES[ch]
            state["gt"].apply(ch)
            note += "  -> 已退出位置指令模式，回到系数整定（当前选中 %s）" \
                    % state["gt"].sel
        elif ch in POS_EDIT_KEYS and state.get("pos_mode"):
            # 位置指令模式：改"待发送值"，【不改系数】
            #   ★ 运算抽到 pos_edit_apply()，与 RTL pos_cmd.v 一一对应，
            #     并由 selftest 逐条核对 —— 见该函数文档。
            v = pos_edit_apply(state.get("pending", 0), ch)
            state["pending"] = v
            note += "  -> 待发送值 = %+d counts (%.2f 圈 @%d/圈)；按 '!' 发送" \
                    % (v, v / float(state.get("cnt_per_rev", CNT_PER_REV)),
                       state.get("cnt_per_rev", CNT_PER_REV))
        else:
            # 系数整定模式：走原来的 GainTracker
            r = state["gt"].apply(ch)
            if r:
                note += "  -> %s = %d (%.3f)" % (r[0], r[1], r[1] / 1024.0)
            elif ch in GainTracker.NAMES:
                note += "  -> 当前选中 " + state["gt"].sel
        if ch == 'U':
            state["up_ref"] = state["last_w0"]
            note += "  -> up_ref 记为 %s" % (state["up_ref"],)
        elif ch == 'D':
            state["down_ref"] = state["last_w0"]
            note += "  -> down_ref 记为 %s" % (state["down_ref"],)
        state["msg"] = "  >> 已发送 '%s'  (%s)" % (ch, note)


def panel_show_gains(state):
    """把系数面板内容放进 state，由主循环统一打印（避免跨线程写 Panel）"""
    lines = ["  当前系数（上位机推算，Q10：1.0=1024）:", "    " + state["gt"].table()]
    # ★ v6.8：顺带提示板载 SW2 的正弦模式（物理键，串口读不到它是否生效）
    lines.append("  板载 SW2(R18) = 正弦轨迹开关：A=%d counts(%.0f°), f=%.3f Hz, 周期 %.2f s"
                 % (SINE_AMP, sine_amp_deg(), sine_freq_hz(), sine_period_s()))
    state["msg"] = "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="J280 倒立摆上位机")
    ap.add_argument("--port", help="串口号，如 COM5；不填则列出并提示")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--log", help="CSV 记录文件名（可选；默认自动记录）")
    ap.add_argument("--nolog", action="store_true",
                    help="关闭 CSV 记录（默认是**自动记录**）")
    ap.add_argument("--selftest", action="store_true", help="不上硬件，自检解码逻辑")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    try:
        import serial
        import serial.tools.list_ports as list_ports
    except ImportError:
        print("缺少 pyserial。请先执行：pip install pyserial")
        return 2

    port = args.port
    if not port:
        ports = list(list_ports.comports())
        if not ports:
            print("未发现任何串口。请确认：")
            print("  1) Type-C 线接的是**核心板的串口**（不是底板下载口）")
            print("  2) 设备管理器中应出现 USB-SERIAL CH340 (COMx)")
            return 2
        print("可用串口：")
        for i, p in enumerate(ports):
            print("  [%d] %-6s %s" % (i, p.device, p.description))
        sel = input("选择序号（回车选 0）：").strip()
        port = ports[int(sel or 0)].device

    print("打开 %s @ %d 8N1 ..." % (port, args.baud))
    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except Exception as e:
        print("打开失败：%s" % e)
        return 2

    # ---- ★ 默认自动记录 CSV ----
    #   原先只有加 --log 才记录，容易漏；现改为**默认就录**：
    #   双击 exe 开始录，按 Q 退出时自动保存并打印路径。
    csv_f = None
    logpath = None
    if args.nolog:
        print("（--nolog：本次不做 CSV 记录）")
    else:
        if args.log:
            logpath = args.log
        else:
            base = os.path.dirname(os.path.abspath(sys.argv[0]))
            d = os.path.join(base, "记录")
            try:
                os.makedirs(d, exist_ok=True)
            except Exception:
                d = base
            logpath = os.path.join(d, "run_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        try:
            csv_f = open(logpath, "w", encoding="utf-8")
            csv_f.write("t_s,w0_angle,w0_deg,w1_pos,w2_mode,u_motor,phi_eff,dphi,"
                        "p_term,d_term,pd_sum,x_settle,err_x,phi_bias\n")
            print("\n★ 正在记录到: %s" % logpath)
            print("  （按 Q 退出时自动保存；不想录就加 --nolog）\n")
        except Exception as e:
            csv_f = None
            print("!! 无法创建记录文件: %s" % e)

    # ★ 关掉控制台快速编辑模式（否则点一下窗口就"死机"）
    if disable_console_quickedit():
        print("[OK] 已关闭控制台快速编辑模式（防止误点窗口导致卡住）")
    else:
        print("[提示] 未能自动关闭快速编辑模式。若出现\"点窗口就卡住\"，\n        请右键控制台标题栏 -> 属性 -> 取消勾选\"快速编辑模式\"")

    dec = FrameDecoder()
    panel = Panel()
    # ★ v6.3 新增状态：
    #     pos_mode : 是否处于位置指令模式（按 'M' 置位，按 '1'..'6' 清除）
    #     pending  : 待发送的增量（上位机侧跟踪，与固件 pos_cmd 同步）
    #     cnt_per_rev : 一圈 counts，用于把 counts 换算成"圈"显示。
    #                   ★ 必须与固件 pendulum_top.v 的 .CNT_PER_REV 一致。
    #                     本版按用户要求用 1024。
    state = {"quit": False, "gt": GainTracker(),
             "up_ref": 2855, "down_ref": None, "last_w0": None, "msg": None,
             "pos_mode": False, "pending": 0, "cnt_per_rev": CNT_PER_REV}

    def send_fn(ch):
        ser.write(ch.encode("ascii"))

    t0 = time.time()
    last_rx = time.time()
    last = 0
    try:
        while not state["quit"]:
            # ★ 先轮询键盘（非阻塞），再读串口 —— 单线程，无竞争
            kb_poll(send_fn, state)
            if state["quit"]:
                break
            if state.get("msg"):
                panel.message(state["msg"])
                state["msg"] = None
            data = ser.read(64)
            if not data:
                # 看门狗：
                #   本版固件遥测复位即开，正常不会走到这里；一旦静默，
                #   先自动补一发 'S'（万一被关掉），再等；仍无数据才提示查线路。
                gap = time.time() - last_rx
                if gap > 1.5 and not state.get("_tel_kick") and state.get("_tel_try", 0) < 3:
                    send_fn("S")
                    state["_tel_try"] = state.get("_tel_try", 0) + 1
                    state["_tel_kick"] = True
                    panel.message("  .. 1.5 秒无数据，已自动发送 'S' 打开遥测")
                elif gap > 5.0 and not state.get("_warned"):
                    panel.message("  !! 已 5 秒未收到任何数据 —— 请检查："
                                  "① 板子是否在运行  ② 串口/波特率 9600  "
                                  "③ 遥测开关（串口发 'S'）")
                    state["_warned"] = True
                continue
            state["_warned"] = False
            state["_tel_kick"] = False      # 收到数据 -> 收工，下次静默可再补发
            last_rx = time.time()
            for (w0, w1, w2, u, phieff, dphi, pterm, dterm, pdsum,
                 xset, errx, phib) in dec.feed(data):
                now = time.time() - t0
                state["last_w0"] = w0
                # ★ v6.3：位置指令一行
                #   绝对坐标 = POS_BASE + x_settle（与 w1_pos 同一个坐标系）
                #   摆臂绝对位置 w1 就在上面那行，这里给的是"目标"和"还差多少"。
                cpr = state.get("cnt_per_rev", CNT_PER_REV)
                tgt_abs = POS_BASE + xset
                mode_tag = "★位置指令模式" if state.get("pos_mode") else "系数整定模式"
                pend = state.get("pending", 0)
                # ★ 固定面板：多行，原地刷新
                panel.render([
                    "w0 = %5d   w1 = %5d   w2 = %d %s"
                    % (w0, w1, w2, MODE_NAME.get(w2, "?")),
                    "phiE = %6d    dphi = %5d" % (phieff, dphi),
                    "P = %7d    D = %7d    PD = %7d"
                    % (pterm, dterm, pdsum),
                    "u = %7d    %s" % (u, ("★饱和" if abs(u) >= 4090 else "")),
                    # ★ v6.0：外环一行。这一行是判断定点功能是否正常的依据。
                    fmt_line_outer(xset, errx, phib),
                    # ★ v6.3：位置指令行
                    "位置: 目标=%6d(绝对%6d)  实际=%6d  还差=%+6d  [%s]"
                    % (xset, tgt_abs, w1, tgt_abs - w1, mode_tag),
                    "待发送 %+6d counts (%.2f 圈 @%d)   按 'M' 进模式 / '+-<>*/' 调 / '!' 发送"
                    % (pend, pend / float(cpr), cpr),
                ])
                # ★ 每帧都记：遥测本身约 13Hz（v6.0 帧长 28 字节），
                #   起摆周期约 0.8s，每周期约 10 个点，够看状态跳变。
                if csv_f:
                    csv_f.write("%.3f,%d,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
                                % (now, w0, adc_to_deg(w0), w1, w2, u,
                                   phieff, dphi, pterm, dterm, pdsum,
                                   xset, errx, phib))
                    last = now
    except KeyboardInterrupt:
        pass
    finally:
        state["quit"] = True
        ser.close()
        if csv_f:
            csv_f.close()
        print("\n\n本次统计：收到 %d 帧，失步 %d 次" % (dec.frames, dec.bad))
        print("已关闭 %s" % port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
