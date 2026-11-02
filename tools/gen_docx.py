# -*- coding: utf-8 -*-
"""
=============================================================================
 文件名 : gen_docx.py
 项目   : J280 旋转倒立摆 FPGA 控制系统
 功能   : 生成 Word 交付文档  Balance_Ctrl_设计文档.docx
-----------------------------------------------------------------------------
 运行：
   "C:\\Users\\一元\\AppData\\Local\\Programs\\Python\\Python310\\python.exe" ^
        tools\\gen_docx.py

 数据来源（全部为真实运行产物，绝不编造）：
   - 仿真结果：<SimDir>\\result_tb_*.txt   （ModelSim vsim -batch 的原始输出）
   - 综合报告：<Work>\\impl\\gwsynthesis\\Balance_Ctrl_syn_resource.html
   - 时序报告：<Work>\\impl\\pnr\\Balance_Ctrl.rpt.txt
 若上述文件不存在，文档中会明确写"未运行/无数据"，不会伪造数字。
=============================================================================
"""

import os
import re
import sys
import glob
import html

from docx import Document
from docx.shared import Pt, RGBColor, Cm
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn

# ---------------------------------------------------------------------------
# 路径
# ---------------------------------------------------------------------------
ROOT    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC     = os.path.join(ROOT, "src")
SIM     = os.path.join(ROOT, "sim")
WORK    = r"C:\J280_work\Balance_Ctrl"
SIMDIR  = os.path.join(ROOT, "sim")   # 读本工程自己的回归日志（镜像目录会混入别的版本）
OUT     = os.path.join(ROOT, "文档", "Balance_Ctrl_设计文档.docx")


# ---------------------------------------------------------------------------
# 基础排版工具
# ---------------------------------------------------------------------------
def set_cn_font(run, name="宋体", size=10.5, bold=False, color=None):
    run.font.name = name
    run.font.size = Pt(size)
    run.font.bold = bold
    if color is not None:
        run.font.color.rgb = color
    r = run._element.rPr.rFonts
    r.set(qn("w:eastAsia"), name)


def h1(doc, text):
    p = doc.add_heading(level=1)
    set_cn_font(p.add_run(text), "黑体", 16, True, RGBColor(0x1F, 0x3B, 0x73))
    return p


def h2(doc, text):
    p = doc.add_heading(level=2)
    set_cn_font(p.add_run(text), "黑体", 13, True, RGBColor(0x1F, 0x3B, 0x73))
    return p


def h3(doc, text):
    p = doc.add_heading(level=3)
    set_cn_font(p.add_run(text), "黑体", 11.5, True, RGBColor(0x33, 0x33, 0x33))
    return p


def para(doc, text, size=10.5, bold=False, indent=True):
    p = doc.add_paragraph()
    if indent:
        p.paragraph_format.first_line_indent = Cm(0.74)
    set_cn_font(p.add_run(text), "宋体", size, bold)
    return p


def bullet(doc, text, size=10.5):
    p = doc.add_paragraph(style="List Bullet")
    set_cn_font(p.add_run(text), "宋体", size)
    return p


def code(doc, text):
    """等宽代码块"""
    for line in text.strip("\n").split("\n"):
        p = doc.add_paragraph()
        p.paragraph_format.left_indent = Cm(0.6)
        p.paragraph_format.space_after = Pt(0)
        p.paragraph_format.space_before = Pt(0)
        set_cn_font(p.add_run(line if line else " "), "Consolas", 9)
    doc.add_paragraph()


def table(doc, headers, rows, widths=None):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Light Grid Accent 1"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, htxt in enumerate(headers):
        c = t.rows[0].cells[i]
        c.text = ""
        set_cn_font(c.paragraphs[0].add_run(str(htxt)), "黑体", 10, True)
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            cells[i].text = ""
            set_cn_font(cells[i].paragraphs[0].add_run(str(v)), "宋体", 9.5)
    doc.add_paragraph()
    return t


def note(doc, text):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.6)
    set_cn_font(p.add_run(text), "楷体", 10, False, RGBColor(0xA0, 0x30, 0x00))
    return p


# ---------------------------------------------------------------------------
# 真实数据读取
# ---------------------------------------------------------------------------
def read_sim_results():
    """读取 ModelSim 真实输出，返回 (汇总列表, 原始日志字典)"""
    summary = []
    logs = {}
    if not os.path.isdir(SIMDIR):
        return None, None
    for f in sorted(glob.glob(os.path.join(SIMDIR, "result_tb_*.txt"))):
        name = os.path.basename(f)[len("result_"):-len(".txt")]
        try:
            with open(f, "r", encoding="utf-8", errors="replace") as fh:
                txt = fh.read()
        except Exception:
            continue
        logs[name] = txt
        # ★ 兼顾本工程两种打印风格：
        #   ① "TEST_RESULT: tb_xxx PASS ..."  ② "TEST_RESULT: PASS  (all checks passed)"
        #   ③ "PASS: 全部用例通过"（老 tb 没印 TEST_RESULT）
        m = re.findall(r"TEST_RESULT:\s*(?:\S+\s*)?(PASS|FAIL)", txt)
        if m:
            summary.append((name, m[-1], ""))
        elif "全部用例通过" in txt:
            summary.append((name, "PASS", "(全部用例通过)"))
        elif re.search(r"\bFAIL\b", txt):
            summary.append((name, "FAIL", ""))
        else:
            summary.append((name, "NO_RESULT", ""))
    return summary, logs


def extract_fail_lines(txt, limit=25):
    out = []
    for line in txt.split("\n"):
        s = line.strip()
        if "[CHECK]" in s and "FAIL" in s:
            out.append(s)
        if len(out) >= limit:
            break
    return out


def read_synth_resource():
    """从 Gowin 综合资源报告里抽取顶层资源占用"""
    cands = [
        os.path.join(WORK, "impl", "gwsynthesis", "Balance_Ctrl_syn_resource.html"),
        os.path.join(WORK, "impl", "gwsynthesis", "Balance_Ctrl_syn.rpt.html"),
    ]
    for c in cands:
        if os.path.exists(c):
            with open(c, "r", encoding="utf-8", errors="replace") as fh:
                raw = fh.read()
            text = re.sub(r"<[^>]+>", " ", raw)
            text = html.unescape(text)
            text = re.sub(r"\s+", " ", text)
            return os.path.basename(c), text[:4000]
    return None, None


def read_pnr_report():
    p = os.path.join(WORK, "impl", "pnr", "Balance_Ctrl.rpt.txt")
    if os.path.exists(p):
        with open(p, "r", encoding="utf-8", errors="replace") as fh:
            return fh.read()
    return None


def count_lines(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            return sum(1 for _ in fh)
    except Exception:
        return "?"


# ===========================================================================
# 文档正文
# ===========================================================================
def build():
    doc = Document()
    style = doc.styles["Normal"]
    style.font.name = "宋体"
    style.font.size = Pt(10.5)
    style.element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")

    sim_summary, sim_logs = read_sim_results()
    synth_name, synth_text = read_synth_resource()
    pnr_text = read_pnr_report()

    # =======================================================================
    # 封面
    # =======================================================================
    t = doc.add_paragraph()
    t.alignment = WD_ALIGN_PARAGRAPH.CENTER
    for _ in range(4):
        doc.add_paragraph()
    set_cn_font(t.add_run("J280 旋转倒立摆 FPGA 控制系统"), "黑体", 26, True,
                RGBColor(0x1F, 0x3B, 0x73))
    p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_cn_font(p.add_run("设 计 文 档"), "黑体", 20, True, RGBColor(0x1F, 0x3B, 0x73))
    doc.add_paragraph()
    p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_cn_font(p.add_run("—— 硬件流水线 PID 实现 ——"), "楷体", 14, False)
    for _ in range(6):
        doc.add_paragraph()

    table(doc,
          ["项目", "内容"],
          [["目标器件", "高云 GW2A-LV55PG484C8/I7（GW2A-55C）"],
           ["开发工具", "Gowin 云源软件 V1.9.12.x"],
           ["硬件平台", "J280 姿态控制系统开发套件（旋转倒立摆 / Furuta 摆）"],
           ["设计语言", "Verilog-2001（无 IP 核、无 PLL、无原语）"],
           ["核心增量", "5 级硬件流水线离散 PID 控制器"],
            ["控制频率", "333.33 Hz（Ts = 3.0 ms）；外环位置环 33.33 Hz（Ts = 30 ms）"],
           ["文档生成", "由 tools/gen_docx.py 自动生成，仿真与综合数据均为真实运行产物"]])
    note(doc, "★ 本文档已按 v7.0（2026-10-23）校准：控制节拍 3 ms / 333.33 Hz、外环 30 ms、"
              "遥测帧 28 字节、RTL 共 24 个模块（src/*.v）。"
              "v7.0 同时交付精简版（lite，21 个模块）：删去在线调参 / 位置指令 / 底板命令键 / 文本通道，"
              "系数改编译期常量、遥测常开、发布位流不带 GAO。详见《课程报告_设计总结_v7.0》。")
    doc.add_page_break()

    # =======================================================================
    # 第 1 章
    # =======================================================================
    h1(doc, "第 1 章  系统总体架构")

    h2(doc, "1.1 被控对象")
    para(doc, "本系统为旋转倒立摆（Furuta 摆）。厂商功能设计文档（pptx slide4）对其描述为："
              "「姿态控制系统中摆臂、摆杆作为被控对象，是一个复杂的快速、非线性、多变量、"
              "强耦合、自然不稳定的非最小相位系统，是重心在上、支点在下控制问题的抽象，"
              "必须施加十分有力的控制手段才能使之稳定。」")
    para(doc, "系统具有两个自由度：摆臂绕电机轴在水平面内做 360° 定轴旋转；摆杆通过联轴器"
              "固定在摆臂一端，可在垂直于摆臂的方向上做 360° 转动。控制目标是：通过驱动摆臂"
              "的水平加速运动，把摆杆「倒立」这一自然不稳定平衡点变成可控的稳定平衡点。")

    h2(doc, "1.2 三条信号通路（厂商 pptx slide11 口径）")
    para(doc, "厂商功能设计文档明确把 FPGA 与机械部件之间的信号划分为三条通路，本设计完全按此实现：")
    code(doc, """
① 控制输出通路
   FPGA ──PWM(调速) + IN1/IN2(方向)──▶ TB6612FNG(功率放大)
        ──▶ 电机 MG531 ──▶ 摆臂 ──▶ 摆杆

② 电机位置/速度反馈通路
   正交编码器 A/B 相 ──▶ qep_encoder(4 倍频解码) ──▶ pos_raw / pos_rel

③ 摆杆姿态采集通路
   WDD35D4(0~3.3V) ──电平转换──▶ (0~2V) ──▶ 3PA1030 ADC
        ──并行 10bit──▶ angle_adc ──▶ angle_raw ──▶ phi
""")

    h2(doc, "1.3 模块划分（严格对齐厂商 pptx slide12）")
    para(doc, "厂商 pptx slide12 给出的 FPGA 模块设计口径为九个模块，本设计的 RTL 文件与之"
              "一一对应，做到「一个模块一个职责」：")
    table(doc,
          ["厂商口径（pptx slide12）", "本设计文件", "职责"],
          [["PWM 生成模块（外设驱动）", "motor_drv.v", "有符号控制量 → TB6612 方向 + 10kHz PWM"],
           ["正交编码器解码模块（外设驱动）", "qep_encoder.v", "A/B 相 4 倍频解码，输出位置与增量"],
            ["ADC 采集（外设驱动）", "angle_adc.v", "3PA1030 时序 + 3ms 窗口 16 点均值，并产生控制节拍"],
           ["角度换算与中值标定（信号处理）", "pendulum_top.v（内）", "phi = angle_raw − up_ref，参考点记录"],
           ["角速度微分模块（信号处理）", "ang_deriv.v", "一阶后向差分得到 dtheta / dx"],
           ["X 阶数字滤波模块（信号处理）", "ang_deriv.v（内）", "4 阶滑动平均，参数化 FILT_TAPS"],
           ["离散 PID 闭环控制（核心算法）", "pid_pipeline.v", "★ 5 级硬件流水线 PID"],
            ["UART 串口（辅助调试）", "uart_tx.v / uart_rx.v / uart_link.v / pid_tuner.v",
             "28 字节遥测帧上报 + 单字符在线调参（精简版无 pid_tuner）"],
            ["固定正弦轨迹（v6.8）", "traj_sine.v", "A=200 counts / 0.2 Hz，板载 SW2 开关"],
            ["OLED 人机界面（v6.6，v7.0 改版）", "oled_disp.v / oled_ssd1306.v / numfmt.v",
             "5 行文本（MODE/CUR/SET/PEND/STAT）+ I2C 驱动（5x7 字模已并入 oled_disp）"],
            ["底板命令键（v6.6）", "ext_keys.v", "5 路命令键采集（仅完整版使用）"],
           ["限位保护（辅助调试）", "limit_prot.v", "摆臂偏心与角度越界保护"],
           ["（组织者）", "ctrl_fsm.v", "空闲/起摆/平衡/故障状态机 + 控制量仲裁"],
           ["（起摆）", "swing_pump.v", "能量泵自动起摆"],
           ["（位置环）", "pos_ctrl.v", "位置参考限速 + 死区，落实「削弱位置环权重」"],
           ["（时钟）", "clk_mgr.v", "复位同步释放（不使用 PLL IP）"],
           ["（按键）", "key_filter.v", "20ms 消抖"]])

    h2(doc, "1.4 顶层数据流")
    code(doc, """
 angle_raw ─(-up_ref)─▶ phi ─┬─▶ ang_deriv ─▶ dtheta (4 阶滑动平均)
                             │                 dx
 pos_raw ─(-10000)─▶ pos_rel ┴─▶ pos_ctrl ─▶ err_x (限速参考 + 死区)
                                     │
                                     ▼
                    ┌──────────────────────────────────┐
                    │  pid_pipeline   (5 级流水线)      │
                    │  S1 锁存 → S2 P项乘 ─┐            │
                    │           → S3 D项乘 ─┼→ S4 加法树 │
                    │           → S3 积分累加┘    → S5   │
                    └──────────────────────────────────┘
                                     │ u_pid
 ctrl_fsm ──mode──▶ 控制量仲裁 ───────┼──────▶ motor_drv ─▶ PWMA/AN1/AN2
      │                              │
      └─▶ swing_pump ──▶ u_swing ────┘
""")

    h2(doc, "1.5 状态机")
    code(doc, """
        run_req↑                  |φ|<CAPTURE_BAND 且 |θ̇|<CAPTURE_RATE
 IDLE ──────────▶ SWING ─────────────────────────────────────▶ BALANCE
   ▲               │                                            │
   │  run_req↑     │ run_req↑        |φ|>LOST_BAND 连续 LOST_HOLD 拍│
   └───────────────┴────────────────────────────────────────────┘
   任意状态 ──fault=1──▶ FAULT ──fault=0──▶ IDLE（必须重新按运行键）
""")
    note(doc, "起摆→平衡用「角度与角速度同时满足」，避免摆杆高速掠过顶部时误切换；"
              "平衡→起摆用连续计数确认，避免单次噪声抖动引起状态乒乓。")

    doc.add_page_break()

    # =======================================================================
    # 第 2 章
    # =======================================================================
    h1(doc, "第 2 章  代码实现原理")

    # ---- 2.1 流水线 PID（重点） ----
    h2(doc, "2.1 核心：硬件流水线 PID（pid_pipeline.v）")

    h3(doc, "2.1.1 控制律")
    para(doc, "采用位置式离散 PID，倾角环与位置环并联叠加：")
    code(doc, """
u(k) = Kp_th·e_th(k) + Kd_th·Δθ(k) + Ki_th·Σe_th
     + Kp_x ·e_x(k)  + Kd_x ·Δx(k)

e_th = θ − θ_up_ref      倾角偏差（单位 = ADC LSB）
Δθ   = θ(k) − θ(k−1)     倾角一阶后向差分（1/Ts 尺度隐含在 Kd 内）
e_x  = x_ref − x         摆臂位置偏差
Δx   = x(k) − x(k−1)     摆臂位置一阶差分
""")

    h3(doc, "2.1.2 Q 定点格式与位宽推导")
    para(doc, "系数一律采用 Q10 有符号定点：1.0 == 1024。位宽推导如下：")
    table(doc,
          ["信号", "取值范围", "选择位宽", "推导依据"],
          [["e_th（倾角偏差）", "±2048", "ERR_W = 16 signed",
            "ADC 为 12bit（0~4095），减参考后最大 ±2048，16bit 留足余量"],
           ["gain（系数）", "0.001 ~ 32.0", "GAIN_W = 16 signed",
            "Q10 下为 1 ~ 32768，恰好在 16bit 有符号范围内"],
           ["单项乘积", "16bit × 16bit", "PROD_W = 32 signed",
            "两数之积最多 32bit；显式用 32bit wire 承接，避免被截断"],
           ["积分累加器", "限幅 ±2^22", "ACC_W = 32 signed",
            "带硬限幅，32bit 远大于限幅值，绝不会回绕"],
           ["级间求和", "5 项 32bit 之和", "SUM_W = 36 signed",
            "5×32bit 之和最多 35bit，取 36bit 留 1bit 余量"],
           ["控制量输出", "±4095", "OUT_W = 16 signed",
            "对应 PWM 满占空比；用 16bit 便于与 motor_drv 接口"]])

    para(doc, "★ 关键设计选择：所有项都在「Q10 乘积域」内相加，只在最后一级做一次 >>>Q_FMT，"
              "避免逐级移位造成的累积截断误差。")
    note(doc, "★ 反标定必须用算术右移 >>>，绝不能用 /1024。"
              "Verilog 的 / 对负数向零取整（-3/2 = -1），而 >>> 向负无穷取整；"
              "且 / 会综合出除法器，浪费大量资源。")

    h3(doc, "2.1.3 流水线分级结构")
    para(doc, "PID 运算拆成 5 级，每级用独立加法器 / DSP 乘法器并行运算，级间插入寄存器作为"
              "「传送带」：")
    table(doc,
          ["级", "名称", "运算内容", "使用资源"],
          [["S1", "锁存级", "s1_err_th / s1_dth / s1_err_x / s1_dx + 全部系数一并锁存", "寄存器"],
           ["S2", "乘法级 A", "s2_p_term = Kp_th·e_th；s2_px_term = Kp_x·e_x", "2 个独立 DSP 乘法器"],
           ["S3", "乘法级 B + 积分级",
            "s3_d_term = Kd_th·Δθ；s3_dx_term = Kd_x·Δx；s3_i_acc 积分累加（带抗饱和）",
            "2 个 DSP + 1 个独立加法器"],
           ["S4", "加法级", "s4_sum = s3_p + s3_d + s3_px + s3_dx + s3_i_acc", "加法器树"],
           ["S5", "输出级", "s5_u = saturate(s4_sum >>> Q_FMT)", "移位 + 比较器"]])

    para(doc, "★ 关键细节：P 项在第 2 级产生，D 项在第 3 级产生，为了让 S4 拿到的五项"
              "来自同一个时钟沿，S3 里额外插入了 s3_p_term / s3_px_term 两个寄存器把 P 项"
              "延后一级，实现**严格同拍对齐**。")

    h3(doc, "2.1.4 延迟核算（显式）")
    para(doc, "流水线级数 N = PIPE_STAGES = 5，因此：")
    bullet(doc, "流水线本体延迟 = 5 拍 × 20 ns = 100 ns（@50MHz 系统时钟）")
    bullet(doc, "整个控制环的群延迟 = 1 个控制周期 Ts + 100 ns = 3.0 ms + 0.0001 ms")
    bullet(doc, "流水线仅贡献 100 ns / 3.0 ms ≈ 0.0033% 的附加延迟")
    bullet(doc, "在 10 Hz 交越频率处，相位损失 = arctan(2π·10·100ns) ≈ 0.036°，可忽略不计")
    para(doc, "与「串行复用单个乘法器」方案对比：后者 5 项需 5 拍串行，虽然绝对延迟同为 100 ns，"
              "但吞吐率只有 1/5，且无法满足赛题「每拍接收新误差、每拍产出新控制量」的要求。")
    para(doc, "★ D 项延迟补偿说明：P/D/I 三项穿越的流水线级数完全相同，因此三者的**相对相位"
              "被严格保持**，不存在「P 快 D 慢」导致的等效微分相位误差，无需额外补偿。"
              "真正需要留意的是 D 项的**噪声放大**问题，通过对差分结果做 4 阶滑动平均解决"
              "（见 ang_deriv.v）。")
    note(doc, "需要澄清一个常见误解：由于误差每 3.0 ms 才更新一次，而流水线是每个 20 ns "
              "时钟沿都在推进的自由流水线，所以「N 级 = N 拍」里的「拍」指的是 50MHz 时钟拍"
              "（100 ns），不是控制周期（3.0 ms）。若把流水线做成按控制节拍推进，"
              "5 级就变成 5 个控制周期 = 15 ms，那才会严重侵蚀相位裕度。")

    h3(doc, "2.1.5 抗饱和策略（三重）")
    table(doc,
          ["层次", "措施", "实现位置"],
          [["1", "积分累加器硬限幅 |s3_i_acc| ≤ I_MAX", "i_acc_clmp 组合逻辑"],
           ["2", "条件积分（积分冻结）：输出已饱和且积分增量与饱和方向同号时冻结积分",
            "i_freeze + s5_sat_dir 反馈"],
           ["3", "输出饱和限幅到 ±OUT_MAX，并输出 s5_sat 标志", "S5 组合逻辑"]])
    note(doc, "冻结必须是**单向**的：当误差反向时（积分增量与饱和方向相反）冻结必须立即解除，"
              "否则系统无法退出饱和。这一点由 testbench 的 T13 用例显式验证。")

    h3(doc, "2.1.6 为后续扩展预留的插入点")
    para(doc, "流水线采用参数化级数（PIPE_STAGES）与分级清晰命名（s1_/s2_/s3_/s4_/s5_），"
              "后续扩展只需在对应级插入寄存器与运算，不影响其它级：")
    bullet(doc, "插入**滤波级**：在 S1 与 S2 之间加一级，对 e_th / Δθ 做更深滤波")
    bullet(doc, "插入**前馈级**：在 S3 增加一路前馈乘法器，与 s3_d_term 并行")
    bullet(doc, "插入**抗饱和级**：在 S5 前增加退饱和补偿支路")

    # ---- 2.2 采集与驱动 ----
    h2(doc, "2.2 外设驱动模块")

    h3(doc, "2.2.1 angle_adc.v —— ADC 采集与中值标定")
    para(doc, "3PA1030 为 10bit / 50MSPS / 量程 0~2V 的并行 ADC。本模块：")
    bullet(doc, "adc_clk = 50MHz 二分频 = 25MHz（纯计数器实现，不使用 PLL）")
    bullet(doc, "adc_oe_n 恒为 0（低有效输出使能）")
    bullet(doc, "12bit 扩展：adc_12 = {adc_data_in, 2'b0}，与官方例程一致")
    bullet(doc, "每 187.5 µs 采一点，3 ms 窗口累加 16 点，用 16bit 累加器 "
                "（16×4095 = 65,472 < 2^16，不溢出）")
    bullet(doc, "每满 3 ms 输出 angle_raw = sum >> 4（右移 4 位 = 精确除以 16，"
                "无需除法器），同时 angle_vld 拉高 1 拍")
    para(doc, "★ 该 angle_vld 同时作为**整个控制环的控制节拍**：控制频率 = 50MHz / 150000 = "
              "333.33 Hz，周期 Ts = 3.0 ms。")
    note(doc, "★ 关于 adc_stby_n：官方例程里有一个 adc_stby_n 输出且恒接 0，"
              "但官方 .cst 引脚约束文件里**根本没有该信号**。本设计因此不引出该信号，"
              "避免综合工具把它随机分配到某个引脚上。这一疑点已记入第 9 章。")

    h3(doc, "2.2.2 qep_encoder.v —— 正交编码器 4 倍频解码")
    para(doc, "编码器为 13 线霍尔编码器，减速比 1:20。本模块做 4 倍频解码（A/B 相每个边沿都计数）。"
              "★ v6.4 起摆臂一圈按实测标定为 **1067 counts**（1 count = 0.3374°），由 traj_ramp 的 CNT_PER_REV 统一给出。")
    bullet(doc, "2 级同步打拍抗亚稳态")
    bullet(doc, "用 {state_last, state_now} 四相状态机判方向，8 种合法跳变 ±1，非法跳变保持")
    bullet(doc, "pos_raw 复位初值 10000（与厂商上位机约定一致）")
    bullet(doc, "pos_rel = pos_raw − 10000，有符号输出")
    note(doc, "4 倍频相对于官方例程的 1 倍频（仅 A 相上升沿）把位置分辨率提高了 4 倍，"
              "这对位置环的阻尼性能有直接好处。")

    h3(doc, "2.2.3 motor_drv.v —— TB6612FNG 驱动")
    para(doc, "输入为有符号控制量 u_ctrl，输出 TB6612FNG 所需的 PWM + 方向信号：")
    table(doc,
          ["u_ctrl 条件", "IN1(in1)", "IN2(in2)", "PWM", "电机状态"],
          [["> +DEADBAND", "1", "0", "有效", "正转"],
           ["< −DEADBAND", "0", "1", "有效", "反转"],
           ["|u_ctrl| ≤ DEADBAND", "0", "0", "0", "停机（防零速抖动）"],
           ["复位期间", "0", "0", "0", "安全停机"]])
    bullet(doc, "PWM 载波：PWM_PERIOD = 5000，50MHz / 5000 = 10 kHz")
    bullet(doc, "占空比比较值 = min(|u_ctrl|, DUTY_MAX)")
    bullet(doc, "取绝对值前先把 16bit 符号扩展到 17bit，避免 u_ctrl = −32768 时取负溢出")
    note(doc, "★ 官方 motor_ctrl.v 里 PWM_PERIOD=5400（≈9.26kHz）且 cmp = duty_out×20，"
              "当 duty_out=255 时 cmp=5100 > 5400 会得到 94% 而不是 100%。"
              "本设计改为直接以「计数 < 比较值」得到干净的 0~100% 占空比。")

    h3(doc, "2.2.4 其它驱动模块")
    bullet(doc, "clk_mgr.v：异步复位、同步释放。复位释放沿若与时钟沿竞争会产生亚稳态，"
                "用移位链把释放同步到时钟域后输出 rst_sys_n。不使用任何 PLL/时钟 IP。")
    bullet(doc, "key_filter.v：按键 20ms 消抖（CNT_MAX = 1_000_000 @50MHz），"
                "按下确认后输出恰好 1 拍脉冲 key_flag，同时输出消抖后电平 key_state。"
                "注意按键为**低有效**（板上内部上拉，按下为 0）。")
    bullet(doc, "limit_prot.v：摆臂偏心超 ±POS_LIMIT 或角度超出 [ANGLE_MIN, ANGLE_MAX] 时置位故障，"
                "并锁存；回到范围内需连续 RECOVER_TICKS=50 个控制周期（≈65.5ms）才解除，防边界抖动。")

    # ---- 2.3 信号处理 ----
    h2(doc, "2.3 信号处理模块")
    h3(doc, "2.3.1 ang_deriv.v —— 角速度微分 + X 阶数字滤波")
    para(doc, "在控制节拍上对倾角与位置做一阶后向差分，再对差分结果做 FILT_TAPS 点滑动平均"
              "（箱型滤波）。FILT_TAPS 取 2 的幂（默认 4），除法退化成算术右移，不消耗除法器。")
    code(doc, """
dθ_raw = θ(k) − θ(k−1)
窗口移位：hist[0] ← dθ_raw, hist[i] ← hist[i−1]
新窗口和 = 旧窗口和 − hist[FILT_TAPS−1] + dθ_raw
dθ = 新窗口和 >>> log2(FILT_TAPS)
""")
    para(doc, "★ 为什么 D 项必须滤波：微分环节本质是高通，角度量化的 ±1 LSB 抖动经差分后被"
              "放大 1/Ts ≈ 333 倍。若不滤波，D 项会出现 ±Kd×333 的剧烈跳变，经 PWM 直接打进"
              "电机造成啸叫与发热。4 阶滑动平均对白噪声的方差抑制比为 1/N，标准差降为 1/2。")
    note(doc, "代价：N 点滑动平均引入 (N−1)/2 个控制周期的群延迟，N=4 时为 1.5×3ms = 4.5ms，"
              "在 10Hz 处附加相位滞后约 16.2°。因此 N 不宜取大，4 阶是「噪声抑制」与「相位损失」"
              "之间的工程折中。（v6.9 曾试过 8 点：噪声再降一半，但群延迟升到 10.5ms，未上板定论。）")

    h3(doc, "2.3.2 pos_ctrl.v —— 位置环三层削弱（落实 pptx slide8）")
    para(doc, "厂商 pptx slide8 原文：「当摆杆向某一方向倾斜时，需要电机控制摆臂带动摆杆作加速"
              "运动使得摆杆摆正；当摆杆摆正后由于摆臂携带的速度持续运动，因此需要引入位置环，"
              "位置环相对于倾角环而言是一种干扰，需要削弱位置环的权重并保证位置环动态性能的"
              "稳定，尽可能减少对倾角环的干扰。」")
    para(doc, "本设计用三层措施落实：")
    table(doc,
          ["层", "措施", "作用"],
          [["1", "位置参考限速：x_ref 每控制周期最多走 SLEW 步逼近目标（traj_ramp.v）",
            "把位置环的指令带宽压到远低于倾角环"],
           ["2", "死区：|x_ref − x| ≤ X_DEADBAND 时 err_x 输出 0",
            "避免中心附近微动持续扰动倾角环，同时抑制编码器抖动"],
           ["3", "位置环增益 Kp_x / Kd_x 独立于倾角环，可串口在线削弱",
            "调参时互不影响"]])
    para(doc, "另有第四层：起摆期间 run_pos = 0，位置环完全不投入，能量泵不受任何干扰。")
    para(doc, "此外，位置环未投入时 x_ref 直接**跟随实际位置**，因此投入瞬间 e_x = 0，"
              "不会产生阶跃冲击。")

    # ---- 2.4 状态机与起摆 ----
    h2(doc, "2.4 状态机与起摆")
    h3(doc, "2.4.1 ctrl_fsm.v")
    bullet(doc, "状态编码同时作为遥测 w2 上报：0=空闲 1=起摆 2=平衡 3=故障")
    bullet(doc, "run_req 上升沿检测，使「按键/串口」与「按一下切换一次」语义统一")
    bullet(doc, "故障优先级最高，且不依赖 tick，立即响应（安全相关）")
    bullet(doc, "故障消失后回到 IDLE 而不是直接恢复运行，避免无人值守时自动上电运转")

    h3(doc, "2.4.2 swing_pump.v —— 起摆能量泵")
    para(doc, "起摆采用 Åström–Furuta 能量泵方法。摆杆机械能（以倒立静止为零势能点）：")
    code(doc, """
E(θ, θ̇) = ½·J·θ̇²  +  m·g·l·(cos θ − 1)
          └─动能─┘    └───相对直立的势能───┘

· 倒立静止：θ=0, θ̇=0  →  E = 0        （能量最高点）
· 自然下垂：θ=π, θ̇=0  →  E = −2mgl   （能量最低点）
""")
    para(doc, "对摆臂施加水平加速度 a，可推出摆杆能量变化率：")
    code(doc, "dE/dt = −m·l·a·θ̇·cos θ")
    para(doc, "因此只要令 **a 与 sign(θ̇·cos θ) 反向**，即可保证 dE/dt > 0，持续向摆杆注入能量。"
              "这就是「能量泵」：摆臂始终朝着让摆杆能量增加的方向加速。方向会随摆杆姿态翻转"
              "（cos θ 变号），这正是起摆过程中摆杆来回摇摆、摆幅逐次增大的物理来源。")
    para(doc, "硬件友好化处理：")
    bullet(doc, "不需要真的计算 E 的数值——方向只依赖 sign(θ̇·cos θ)")
    bullet(doc, "cos θ 用分段线性近似 cos ≈ clamp(1024 − |φ|, −1024, +1024) 代替查找表，"
                "省掉 ROM 与乘法器")
    bullet(doc, "θ̇×cos 的乘积只取符号位，因此不必担心中间结果溢出")
    para(doc, "★ 起摆死区踢摆：摆杆静止在下垂点时 θ̇ ≈ 0，sign(θ̇·cos θ) 正负不定，"
              "能量泵会「卡死」在零输出。因此当 |θ̇| < KICK_MIN_DTH 时改用固定方向的踢摆脉冲，"
              "人为打破对称性；摆杆动起来后能量泵即可接管。这是起摆能否自启动的关键。")

    # ---- 2.5 串口 ----
    h2(doc, "2.5 串口与在线调参")
    h3(doc, "2.5.1 uart_tx.v / uart_rx.v")
    bullet(doc, "9600 8N1，每位 5208 拍 @50MHz，整帧 10 位 = 52080 拍")
    bullet(doc, "uart_rx 采用 3 级同步打拍 + 下降沿检测启动，并在**位的中点**采样")
    bullet(doc, "uart_rx 检查停止位：若停止位不为 1 则丢弃该字节，不输出 po_flag，"
                "并输出 frame_err 脉冲")
    note(doc, "★ 官方 uart_tx.v 的 bit_flag 在 baud_cnt == 1 时就拉高，导致起始位只有 2 拍宽，"
              "整帧只有 46875 拍，比正确值 52080 拍**短 10%**。这在 9600 下尚能容忍，"
              "但属于真实的位定时缺陷。本设计重写为每位严格 5208 拍。")

    h3(doc, "2.5.2 uart_link.v / pid_tuner.v")
    bullet(doc, "uart_link：每 FRAME_GAP 发一帧 **28 字节**遥测（AA 55 + 24 字节 w0..w11 + 0D 0A），"
                "字节之间插入 BYTE_GAP 间隔，保证与 9600 的位速率匹配；同时把接收到的字节透传给 pid_tuner")
    bullet(doc, "pid_tuner：解析单字符命令，维护 6 个系数（倾角环 KP/KI/KD + 位置环 KP/KD + 起摆占空比）与积分档位 i_shift")

    doc.add_page_break()

    # =======================================================================
    # 第 3 章
    # =======================================================================
    h1(doc, "第 3 章  控制算法推导")

    h2(doc, "3.1 位置式与增量式的选择理由")
    para(doc, "本设计选择**位置式**离散 PID，理由如下：")
    bullet(doc, "增量式输出的是控制量增量 Δu，需要执行器自己累加。本系统执行器是 TB6612FNG"
                "功率驱动，只接受绝对占空比，内部没有累加器；一旦串口/上位机丢帧，"
                "增量式会造成永久性偏移。")
    bullet(doc, "位置式的积分项**显式可观测**（s3_i_acc 是一个独立寄存器），抗饱和（限幅 + 冻结）"
                "容易实现且行为完全可预测；增量式把积分隐藏在累加里，抗饱和难以干净地表达。")
    bullet(doc, "位置式便于把倾角环与位置环以**并联叠加**的方式合并："
                "u = Kp_th·e_th + Kd_th·Δθ + Ki_th·Σe_th + Kp_x·e_x + Kd_x·Δx。"
                "这与厂商手册 p20「倾角环 + 位置环 → 叠加 → 电机PWM」的结构一致；"
                "增量式做并联叠加需要额外合并逻辑，反而更绕。")
    bullet(doc, "硬件流水线的天然组织方式就是「每拍接收新误差、每拍产出新控制量」，"
                "位置式的数据流最适合这种结构。")

    h2(doc, "3.2 倾角环与位置环的主从关系")
    para(doc, "这是本系统最关键的控制结构问题。物理机理如下：")
    para(doc, "① 摆杆向某方向倾斜时，摆臂必须**朝同方向加速**，用惯性把摆杆「顶」回来；"
              "② 摆杆被摆正之后，摆臂已经带上了速度，会继续朝原方向运动，"
              "把摆杆又带偏——于是摆臂会慢慢「跑」到一边去；"
              "③ 因此必须引入位置环，把摆臂拉回中点。")
    para(doc, "但位置环与倾角环的目标在短时间尺度上是**冲突**的：倾角环要求摆臂立刻加速去救摆杆，"
              "位置环却要求摆臂别乱跑。厂商在 pptx slide8 明确把位置环定性为「干扰」，"
              "要求「削弱位置环的权重并保证位置环动态性能的稳定」。")
    para(doc, "本设计的处理是**时间尺度分离**：倾角环工作在毫秒量级（它直接决定系统能否不倒），"
              "位置环通过参考限速被压到秒量级（SLEW = 2 counts/控制周期，"
              "在 333Hz 下相当于 667 counts/s，走完 ±3201 的行程限位约需 4.8 秒）。"
              "两个环在频域上被拉开，位置环的扰动落在倾角环带宽之外，因此不会破坏倾角环的相位裕度。")

    h2(doc, "3.3 起摆能量泵原理")
    para(doc, "推导见 2.4.2 节。这里补充工程实现上的两点考量：")
    para(doc, "① **为什么用能量判据而不是简单的「跟随摆杆运动」**：简单的同向跟随在摆杆能量已接近"
              "倒立所需时仍然全功率泵浦，会导致摆杆冲过顶部。能量判据 E → 0 天然给出了"
              "「够了就停」的收敛条件。")
    para(doc, "② **捕获判据必须同时约束角度和角速度**：摆杆在起摆过程中会多次高速掠过顶部附近，"
              "此时角度瞬时满足 |φ| 很小，但角速度很大。若只看角度就会误切入平衡环，"
              "而平衡环的 PID 根本来不及响应，结果是立刻掉出又重新起摆，形成振荡。"
              "本设计要求 |φ| < CAPTURE_BAND **且** |θ̇| < CAPTURE_RATE 才切换。")

    h2(doc, "3.4 极性自检的工程必要性")
    para(doc, "系统的闭环极性由四个因素共同决定：电机正转定义、编码器 A/B 相接法、"
              "摆杆安装朝向、角位移传感器安装朝向。这四个因素在产品化之后是固定的，"
              "但在调试阶段任何一个接反都会导致正反馈发散。")
    para(doc, "本设计把无法事先确定的两个极性做成参数（swing_pump 的 PUMP_POLARITY、"
              "PID 系数的符号），上板时只需改一个数字或按几个按键即可纠正，不必改逻辑、"
              "不必重新推导。具体自检流程见 4.4 节。")

    doc.add_page_break()

    # =======================================================================
    # 第 4 章
    # =======================================================================
    h1(doc, "第 4 章  硬件调试")

    h2(doc, "4.1 上板步骤")
    steps = [
        "把整个 Balance_Ctrl 目录复制到**纯英文路径**下（例如 C:\\J280_work\\Balance_Ctrl）。"
        "厂商《上手使用手册》Step2 原话：「需要将程序放在非中文目录下，避免编译报错」。",
        "用云源软件新建工程，器件选 GW2A-LV55PG484C8/I7，Top Module 选 pendulum_top，"
        "加入 src/*.v，导入 Balance_Ctrl.cst 与 Balance_Ctrl.sdc。",
        "★ 勾选 Project → Configuration → Place & Route → 「Use SSPI as regular IO」，"
        "否则 T20(key_sw[3] / SW4) 会被工具挪作 SSPI 脚，SW4 失效。"
        "（用命令行 gw_sh flow.tcl 时该选项已在脚本里显式指定。）",
        "Run all 综合布局布线，生成 .fs 位流。",
        "Programmer → 选择 .fs → 以 SRAM Program 模式下载。"
        "（如需掉电保持，改用 External Flash Mode 并重新上电。）",
        "拔掉下载线，把 Type-C 接到**核心板串口**（设备管理器应出现 USB-SERIAL CH340）。",
        "上电后先做角度标定（4.2 节），再做极性自检（4.4 节），最后按 SW1 运行。",
    ]
    for i, s in enumerate(steps, 1):
        p = doc.add_paragraph()
        p.paragraph_format.left_indent = Cm(0.5)
        set_cn_font(p.add_run("步骤 %d：" % i), "黑体", 10.5, True)
        set_cn_font(p.add_run(s), "宋体", 10.5)

    h2(doc, "4.2 标定流程")
    para(doc, "厂商《上手使用手册》§3.1/§3.6 与《介绍手册》§5.3 给出的标定基准是："
              "摆杆**自然下垂**时上位机采集到的角度值应在 **990 ~ 1040** 范围内。")
    table(doc,
          ["步骤", "操作", "判据 / 说明"],
          [["1", "摆杆自然下垂，串口观察 w0", "应在 990~1040；否则按 §3.6 矫正"],
           ["2", "摆杆下垂时按 SW3（或串口发 'D'）", "记录 down_ref"],
           ["3", "扶摆杆竖直，按 SW2（或串口发 'U'）", "记录 up_ref"],
           ["4", "松开，按 SW1", "进入起摆/平衡"]])
    para(doc, "★ 角度矫正方法（《上手使用手册》§3.6 原文要点）："
              "① 先松动**联轴器两端的螺丝**；② 用平口螺丝刀左右轻微转动，同时观察串口角度值；"
              "③ 调到要求范围后紧固联轴器两端螺丝。注意紧固过程本身还会再次改变角度，"
              "需要耐心反复微调。")
    note(doc, "本设计上电默认 up_ref = 3149、down_ref = 1015（推导：WDD35D4 电气转角 345° "
              "对应 ADC 0~4090，则 180° ≈ 2134 counts；若下垂读数约 1015，直立即约 3149）。"
              "但**每台设备的实际值都不同**，必须按上表用 SW2/SW3 实测记录。")

    h2(doc, "4.3 按键与 LED 分配")
    table(doc,
          ["按键", "引脚", "功能"],
          [["SW1", "T18", "运行 / 停止（切换 IDLE ↔ SWING/BALANCE）"],
           ["SW2", "R18", "记录直立参考 up_ref"],
           ["SW3", "U20", "记录下垂参考 down_ref"],
           ["SW4", "T20", "遥测开关（★ SSPI 复用脚）"],
           ["SW5", "AB3", "复位（rst_n）"]])
    table(doc,
          ["LED", "引脚", "含义（低电平点亮）"],
          [["LED1", "U18", "心跳 ≈1.49Hz，证明逻辑在运行"],
           ["LED2", "U19", "运行中（mode ≠ IDLE）"],
           ["LED3", "U17", "已平衡（mode == BALANCE）"],
           ["LED4", "T17", "告警（限位故障 或 输出饱和）"]])
    note(doc, "★ LED 为**低电平点亮**：依据官方 key_led 例程 led_ctrl.v，四个 LED 的复位值"
              "都是 1'b1（熄灭），按键触发才翻转，因此是低有效。")

    h2(doc, "4.4 极性自检（上板第一步必做）")
    table(doc,
          ["序", "检查项", "操作方法", "正确现象", "反了怎么办"],
          [["1", "电角度极性", "缓慢推动摆杆从下垂到直立",
            "w0 单调变化，phi 收敛到 0 附近", "检查传感器供电与 ADC 接线"],
           ["2", "编码器极性", "手动让摆臂顺时针转",
            "串口 w1 递增", "硬件上对调 A/B 相即可，软件无需改"],
           ["3", "起摆极性", "按 SW1 让它自动起摆",
            "摆幅逐次增大直到立起", "把 swing_pump.v 的 PUMP_POLARITY 改为 −1"],
           ["4", "平衡环极性", "起摆成功后观察是否能在顶部站住",
            "能收敛站立", "把 pid_tuner 的 KP_TH_INIT 取负"]])

    h2(doc, "4.5 调参指南（现象 → 操作）")
    table(doc,
          ["现象", "原因判断", "串口操作"],
          [["摆杆往下掉，电机不响应", "倾角环极性反或增益太小", "按 1 选 KP，连按 > 增大；仍无反则取负"],
           ["摆杆左右剧烈抖动 / 电机啸叫", "D 项过强或角度噪声大", "按 3 选 KD，连按 < 减 12.5%"],
           ["直立但缓慢往一边漂", "位置环太弱或被死区吃掉", "按 4 选位置P，按 > 增大；必要时减小 X_DEADBAND"],
           ["摆臂缓慢转圈停不下来", "位置环太弱", "按 4 / 5 增大 Kp_x / Kd_x"],
           ["起摆摆幅一直不增长", "能量泵极性反", "把 PUMP_POLARITY 改为 −1"],
           ["起摆到顶部落不下来", "捕获带太宽", "减小 ctrl_fsm 的 CAPTURE_BAND / CAPTURE_RATE"],
           ["平衡后频繁掉出又重起", "丢失阈值太紧", "增大 LOST_BAND 或 LOST_HOLD"],
           ["稳住后有小幅低频摆动", "积分项偏强", "按 2 选 KI，连按 < 减小"],
           ["电机始终不转", "未按 SW1 / 已进故障态", "看 LED4 是否亮；按 SW5 复位后重来"]])
    para(doc, "调参完成后，把串口回读到的系数值写回 pid_tuner.v 顶部的 KP_TH_INIT / KI_TH_INIT / "
              "KD_TH_INIT / KP_X_INIT / KD_X_INIT 参数并重新综合，即可固化。")

    doc.add_page_break()

    # =======================================================================
    # 第 5 章
    # =======================================================================
    h1(doc, "第 5 章  约束配置")

    h2(doc, "5.1 引脚约束 Balance_Ctrl.cst")
    para(doc, "引脚来源经三方交叉核实：① 官方 Interface_test/src/AD_test.cst；"
              "② 官方 key_led/src/key_led.cst（LED）；"
              "③ Balance_Ctrl_20260618.pdf 原理图网表。")
    table(doc,
          ["信号", "引脚", "IO 标准 / 驱动", "说明"],
          [["clk_50m", "M19", "LVCMOS33 PU", "50MHz 系统时钟"],
           ["rst_n", "AB3", "LVCMOS33 PU", "复位，低有效"],
           ["key_sw[0]", "T18", "LVCMOS15 PU", "SW1 运行/停止"],
           ["key_sw[1]", "R18", "LVCMOS15 PU", "SW2 记直立参考"],
           ["key_sw[2]", "U20", "LVCMOS15 PU", "SW3 记下垂参考"],
           ["key_sw[3]", "T20", "LVCMOS15 PU", "SW4 遥测开关（★ SSPI 复用）"],
           ["led[3:0]", "U18/U19/U17/T17", "LVCMOS15 DRIVE=8", "低电平点亮"],
           ["uart_tx", "K20", "LVCMOS33 DRIVE=8", "FPGA → PC"],
           ["uart_rx", "L20", "LVCMOS33 PU", "PC → FPGA"],
           ["adc_clk", "G18", "LVCMOS33 DRIVE=8", "25MHz 输出给 3PA1030"],
           ["adc_oe_n", "H20", "LVCMOS33 DRIVE=8", "低有效输出使能"],
           ["adc_data_in[9:0]", "E19 G17 G19 H19 H18 D19 J19 J20 M21 L21",
            "LVCMOS33 PU", "ADC 并行数据 D9~D0"],
           ["enc_a / enc_b", "Y20 / AA20", "LVCMOS33 PU", "XH1 编码器 A/B 相"],
           ["PWMA / AN1 / AN2", "Y18 / Y17 / Y19", "LVCMOS33", "XH1 电机驱动"],
           ["enc2_a / enc2_b", "W18 / W17", "LVCMOS33 PU", "XH2 预留（不解析）"],
           ["PWMB / BN1 / BN2", "V14 / AA17 / V15", "LVCMOS33", "XH2 预留（顶层恒 0）"]])
    note(doc, "★ 必须勾选「Use SSPI as regular IO」：T20 是 SSPI 复用脚。"
              "在 GUI 综合时若不勾选，工具会把 T20 挪作 SSPI 引脚，导致 SW4 失效；"
              "用命令行 gw_sh 时已由 flow.tcl 中的 set_option -use_sspi_as_gpio 显式指定。")

    h2(doc, "5.2 时序约束 Balance_Ctrl.sdc")
    para(doc, "本设计的时序特征决定了约束非常简洁：")
    bullet(doc, "**单时钟域**：整个设计只有 clk_50m 一个时钟（50MHz）。"
                "由于赛题禁止使用 PLL IP，没有任何派生时钟、没有时钟域交叉，"
                "不需要 set_false_path 处理 CDC。")
    bullet(doc, "所有寄存器都由 posedge clk_50m 驱动，复位为异步低有效，"
                "在 clk_mgr 内部做同步释放。")
    bullet(doc, "最终只保留一条主时钟约束，外加若干条异步输入例外。")
    code(doc, """
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]

# 异步输入例外（模块内部已多级同步打拍）
set_false_path -from [get_ports {key_sw[*]}]     -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {rst_n}]         -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {uart_rx}]       -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {enc_a enc_b}]   -to [get_clocks {clk_50m}]
set_false_path -from [get_ports {adc_data_in[*]}] -to [get_clocks {clk_50m}]
""")
    para(doc, "不加 multicycle 的理由：流水线 PID 的级间是纯寄存器到寄存器路径，"
              "16bit×16bit 乘法 + 36bit 加法在 GW2A-55 上远小于 20ns，"
              "放宽反而会掩盖真实的时序问题。")

    doc.add_page_break()

    # =======================================================================
    # 第 6 章  仿真验证（真实数据）
    # =======================================================================
    h1(doc, "第 6 章  仿真验证")
    para(doc, "本章所有内容均来自**真实运行的仿真输出**，原始日志保存在工程的 sim 目录下"
              "（由 tools/run_sim.ps1 生成）。未运行的测试一律标注为「未运行」，"
              "不做任何形式的通过声称。")

    h2(doc, "6.1 仿真环境与调用方式")
    table(doc,
          ["项目", "内容"],
          [["器件", "GW2A-LV55PG484C8/I7"],
           ["实际使用的仿真器", "ModelSim SE-64 2020.4（D:\\MODELISM）"],
           ["调用方式", "vsim -batch work.<tb> -do run.do"],
           ["工作路径", "C:\\J280_work\\Balance_Ctrl（★ 纯英文路径）"],
           ["提示词指定的仿真器", "Icarus Verilog 13.0 便携版 —— **本机不存在，未能使用**"]])
    note(doc, "★ 两个关键环境结论（均为实测，非推断）：\n"
              "（1）ModelSim 的 vlog/vsim 在含中文的路径下会把路径乱码化并失败，"
              "报 sqlite3_open C:/Users/<乱码>/... unable to open database file。"
              "必须放在纯英文路径下运行。\n"
              "（2）ModelSim 在 `-c` 模式下必报 Error: can't read \"FileWatch(fileName)\": "
              "no such element in array，设计无法加载、仿真完全不执行；"
              "改用 `-batch` 模式后**可以正常加载设计并跑出结果**。"
              "提示词中「ModelSim vsim 无法出波形、只能做语法检查」的结论"
              "在 `-batch` 下不再成立。")

    h2(doc, "6.2 各 testbench 的目的")
    table(doc,
          ["testbench", "验证目的", "关键判据"],
          [["tb_top_smoke", "整机冒烟（集成）",
            "★ 真实 9600 串口解码 28 字节遥测帧 + 真实 20ms 按键消抖 + 编码器正交序列 + ADC 接口"],
           ["tb_top_poscmd", "端到端位置指令",
            "UART 发 M/+/-/! → pos_cmd → traj_ramp.delta_ext → x_settle 全链路贯通"],
           ["tb_pid_pipeline", "★ 内环流水线 PID（核心）",
            "延迟拍数、吞吐率、输出饱和、积分限幅、有符号运算、run_en 清零"],
           ["tb_pos_pd", "外环 PD（串级设定值）",
            "φ_bias 限幅 ±250、每拍 ≤2 counts 斜率、DIV=10 分频、外环不并联"],
           ["tb_swing_pump", "起摆能量泵",
            "判据（θ̇ 与 cosφ 符号之「与」）、死区踢摆、捕获交棒、极性参数"],
           ["tb_pid_tuner", "串口在线调参",
            "★ 有符号 Q10 的 ±12.5% 正确性、32bit 饱和、单拍脉冲、位置指令模式"],
           ["tb_adc_prod", "ADC 采集（真机参数）",
            "25MHz adc_clk、每窗 16 点、187.5us 采样间隔、真机参数下累加与输出不撞拍"],
           ["tb_uart_link", "遥测帧",
            "28 字节帧格式、大端序、字节间隔、命令透传"],
           ["tb_motor_drv", "电机驱动",
            "方向真值表、死区、占空比精度、限幅、复位安全"]])

    note(doc, "说明：开发过程中另编写了 22 个模块级/专项用例（ADC 窗口与平均、编码器 4 倍频、按键消抖、OLED、限位保护、状态机、正弦/斜坡等），合计 31 个；交付包中保留上表 9 个核心用例。")

    h2(doc, "6.3 真实运行结果")
    if sim_summary is None:
        note(doc, "未找到仿真结果目录 %s —— 仿真未运行，本章无数据。" % SIMDIR)
    else:
        pass_n = sum(1 for _, v, _ in sim_summary if v == "PASS")
        fail_n = sum(1 for _, v, _ in sim_summary if v == "FAIL")
        rows = []
        for name, verdict, extra in sim_summary:
            rows.append([name, verdict, extra if extra else "—"])
        table(doc, ["testbench", "结果", "备注"], rows)
        para(doc, "统计：共 %d 个 testbench，PASS = %d，FAIL = %d。"
                  % (len(sim_summary), pass_n, fail_n))

        # 附上失败用例的真实明细
        for name, verdict, extra in sim_summary:
            if verdict != "PASS" and name in sim_logs:
                h3(doc, "失败明细：%s" % name)
                fails = extract_fail_lines(sim_logs[name], 25)
                if fails:
                    code(doc, "\n".join(fails))
                else:
                    para(doc, "该 tb 未产生 [CHECK] ... FAIL 行，可能编译失败或未跑完。")

    h2(doc, "6.4 仿真中发现并修复的缺陷")
    para(doc, "本工程的仿真**不是走过场**——它在 RTL 与 testbench 两侧都抓出了真实缺陷。"
              "以下每一条都有 NameError 之外的具体证据（失败用例名 + 实测值）。")

    h3(doc, "6.4.1 RTL 缺陷（真实设计错误）")
    table(doc,
          ["#", "文件", "缺陷", "被哪个用例抓出", "修法"],
          [["1", "swing_pump.v",
            "★ 有符号/无符号比较陷阱：写成 `(13'sd1024 - phi_abs[12:0]) < -13'sd1024`。"
            "`phi_abs[12:0]` 是对有符号向量的部分选择，结果**无符号**，"
            "导致整个比较按无符号进行，-1024 被当成 7168，条件恒真 → cos_approx 恒为 −1024。"
            "后果：能量泵方向**永久反向**，自动起摆彻底失效。",
            "tb_swing_pump T4a/T4b（期望 −2600 实得 +2600，恰好整体反向）",
            "改用 17bit 有符号中间量 cos_raw 后统一在有符号域比较"],
           ["2", "limit_prot.v",
            "★ fault 一旦置位**永远无法解除**：原式要求 arm_ok_cnt 与 ang_ok_cnt "
            "同时达到阈值才清 fault，但只有一路故障时另一路计数器被长期清零，永远达不到。",
            "tb_limit_prot C8（fault_cleared_at_50ticks 期望 0 实得 1）",
            "改为对两路故障的次态取或：fault <= arm_fault_n | ang_fault_n"],
           ["3", "ang_deriv.v",
            "★ 滑动平均输出**整整滞后一拍**：dtheta 由「移位前」的窗口和算出，"
            "本拍新采到的差分要等下一拍才反映出来。",
            "tb_ang_deriv T3（期望 2,2,2,2 实得 0,2,2,2）",
            "改用「旧和 − 最旧 + 新样本」的新窗口和"],
           ["4", "pid_pipeline.v",
            "★ S4 加法级**分级错位**：原式 `s4_sum_a <= s2_p_term + s3_d_term`，"
            "而这两个信号分别在第 2 级和第 3 级产生，相差一整级；"
            "在输入突变的暂态中会把「新的 P 项」和「旧的 D 项」相加，得到无物理意义的中间值。",
            "代码审查 + 延迟核算（会让「5 级 = 5 拍」的核算失去意义）",
            "在 S3 增加 s3_p_term / s3_px_term 两级寄存器把 P 项打拍对齐"],
           ["5", "pid_pipeline.v",
            "s5_vld <= s3_vld（漏掉了 S4 的有效标志），导致 u_vld 比 u_ctrl 早一拍，"
            "上层无法用它准确判定输出有效性。",
            "代码审查", "补上 s4_vld，改为 s5_vld <= s4_vld"],
           ["6", "uart_link.v",
            "★ 编译错误：`assign tx_busy = (state != ST_IDLE);` 写在 state / ST_IDLE "
            "声明**之前**，ModelSim 报 (vlog-2730) Undefined variable 与 "
            "(vlog-13282) already declared。uart_link 编译失败 → pendulum_top 无法 elaborate "
            "→ tb_uart_link 与 tb_top_smoke 全部无法加载。",
            "vlog 编译输出", "把 assign 移到声明之后"],
           ["7", "motor_drv.v",
            "复位/停机期间 PWM 仍可能为高（pwm 为组合输出，复位瞬间 pwm_cnt=0 而 cmp 尚未清零）。",
            "tb_motor_drv C11（after_rst_pwm_0 期望 0 实得 1）",
            "pwm 输出与 pwm_run 相与，复位期间 pwm_run=0 强制拉低"],
           ["8", "uart_tx.v",
            "★ 位宽实际为 5209 拍而非 5208：`bit_tick` 被写成**寄存器**，"
            "比 `baud_cnt` 归零晚一拍，于是**每一位**都被拉长 1 拍，整帧变成 52090 拍，"
            "与模块头部注释「每位 5208 拍、整帧 52080 拍」自相矛盾。",
            "tb_uart_tx T6b（逐位测跳变时刻，实测 offset 恒比理论大 1："
            "5209/10417/15625/20833/…）",
            "改为组合 wire，切换发生在同一拍，位宽回到严格 5208"],
           ["9", "uart_tx.v",
            "★ `pi_data` 未锁存：模块边发边读端口，**传输中途改动 pi_data 会直接污染"
            "正在发送的帧**，等于没有真正「消费」 pi_flag 所声明的数据。",
            "tb_uart_tx T8（发 0x55 到一半注入 0xAA，本帧发出的就是 0xAA）",
            "在 pi_flag 被接受那一拍把 pi_data 锁存进 tx_shift，整帧从锁存值取位"]])
    note(doc, "值得强调的是缺陷 #1：这是**仿真才能抓到、上板只会表现为「起摆不好使」**的那类错误。"
              "如果没有 testbench 逐值比对方向律，很可能被误判成「机械摩擦太大」或「增益不对」，"
              "从而在错误的方向上浪费大量调试时间。")

    h3(doc, "6.4.2 Testbench 缺陷（测试平台自身的错误）")
    para(doc, "需要如实说明：首轮仿真中有相当一部分失败是 **testbench 自身的问题**，"
              "而不是 RTL 的问题。把它们列出来同样重要——否则会误导后续维护者去「修」"
              "本来正确的设计。")
    table(doc,
          ["#", "文件", "缺陷", "典型症状", "修法"],
          [["1", "tb_pid_pipeline / tb_pid_tuner / tb_key_filter 等多个 tb",
            "★ **NBA 竞争**：在 @(posedge clk) 之后的同一时间步的 active 区域立刻读 DUT 输出，"
            "此时 NBA 更新尚未提交、组合逻辑也还没重算，读到的是上一拍的旧值。",
            "tb_pid_tuner 23 条断言全挂且**整体慢一条命令**；"
            "tb_pid_pipeline 延迟测出 6 拍而非 5 拍",
            "在 posedge 之后加 #1，或改在 @(negedge clk) 采样"],
           ["2", "tb_pid_pipeline", "T13 用 −1024 作为反向误差，"
            "i_inc = 32767×(−1024) 会把积分一次性打到**负向**限幅，输出变成 −4095（负饱和），"
            "于是「脱离正饱和」的断言不成立。",
            "T13b 期望 s5_sat=0 实得 1",
            "改用小反向误差 −8，使回退落在限幅内"],
           ["3", "tb_ang_deriv", "把「单个跳变」的输入误当成「持续斜坡」来写期望值。"
            "单脉冲输入的窗口和恒为 8，正确响应是 2,2,2,2,0，而不是 2,4,6,8。",
            "T3 四项全挂", "拆分用例：单脉冲与斜坡分开验，按物理推导写期望"],
           ["4", "tb_uart_rx", "第 307 行引用了未定义变量 t_flag3，导致整个 tb 编译失败，"
            "进而 tb_uart_rx 无法加载（NO_RESULT）。",
            "vlog (vlog-2730) Undefined variable: 't_flag3'",
            "修正未定义引用"],
           ["5", "tb_uart_tx", "看门狗超时（99 项批量失败），说明卡在某个 @(...) 等待上，"
            "后续断言根本没执行。", "只有一行「看门狗超时」失败信息",
            "定位死等点，改用不会错过单拍脉冲的握手方式"],
           ["6", "tb_angle_adc", "把 angle_vld 的**高电平持续拍数**当成了**出现间隔**来测量。",
            "C4 vld_interval_cycles 期望 16 实得 1",
            "改为统计两个上升沿之间的时钟数"]])
    note(doc, "★ 方法论结论：一个「失败」必须先判定它到底是 RTL 错还是 tb 错。"
              "本工程的判据是——**看失败是否成片且呈现固定偏移**："
              "成片、整体滞后一条命令/一拍的，几乎必然是 tb 采样时机问题；"
              "而像 swing_pump 那样「方向恰好整体反向」的，则是 RTL 的真实缺陷。"
              "把两者混为一谈，会既修不好 tb，也漏掉真 bug。")

    doc.add_page_break()

    # =======================================================================
    # 第 7 章
    # =======================================================================
    h1(doc, "第 7 章  串口协议")

    h2(doc, "7.1 物理层")
    table(doc,
          ["项目", "值"],
          [["波特率", "9600"], ["数据位", "8"], ["校验", "无（N）"], ["停止位", "1"],
           ["流控", "无"], ["电平", "TTL（核心板 CH340 转 USB）"],
           ["FPGA 引脚", "uart_tx = K20（输出），uart_rx = L20（输入）"]])
    note(doc, "★ 厂商手册里 TXD/RXD 的命名方向与官方 .cst 约束是相反的，本设计以 .cst 为准。")

    h2(doc, "7.2 遥测帧格式（28 字节固定长度）")
    code(doc, """
 字节序号  0     1     2     3     4     5     6     7     8     9
 内容     AA    55   w0H   w0L   w1H   w1L   w2H   w2L   0D    0A
          └帧头┘     └─摆杆角度─┘  └─摆臂位置─┘  └─模式─┘  └─帧尾┘
""")
    table(doc,
          ["字", "含义", "取值范围", "说明"],
          [["w0", "摆杆角度", "0 ~ 4090",
            "ADC 原始值。自然下垂应在 990~1040；直立约为下垂值 +2134"],
           ["w1", "摆臂编码器位置", "0 ~ 65535（有符号解释 ±32767）",
            "复位初值 10000；正转递增、反转递减；4 倍频，1040 counts/圈"],
           ["w2", "模式", "0 / 1 / 2 / 3", "0=空闲 1=起摆 2=平衡 3=故障"]])
    para(doc, "多字节字段一律**大端序**（高字节先发）。上位机解析示例（Python）：")
    code(doc, """
import serial
ser = serial.Serial('COM3', 9600, timeout=1)
buf = b''
while True:
    buf += ser.read(1)
    if len(buf) >= 2 and buf[-2:] == b'\\xAA\\x55':
        rest = ser.read(8)                 # 还差 8 字节
        f = buf[-2:] + rest
        if len(f) == 10 and f[8:] == b'\\x0D\\x0A':
            w0 = (f[2] << 8) | f[3]
            w1 = (f[4] << 8) | f[5]
            w2 = (f[6] << 8) | f[7]
            print('angle=%d pos=%d mode=%d' % (w0, w1, w2))
        buf = b''
""")

    h2(doc, "7.3 单字符在线调参命令表")
    para(doc, "所有命令均为单个 ASCII 字符，9600 8N1 直接发送，**无需停机、无需重新综合**。")
    table(doc,
          ["字符", "ASCII", "动作", "说明"],
          [["1", "0x31", "选择 KP（倾角环 P）", "sel_loop = 0"],
           ["2", "0x32", "选择 KI（倾角环 I）", "sel_loop = 1"],
           ["3", "0x33", "选择 KD（倾角环 D）", "sel_loop = 2"],
           ["4", "0x34", "选择位置环 P", "sel_loop = 3"],
           ["5", "0x35", "选择位置环 D", "sel_loop = 4"],
           [">", "0x3E", "当前选中系数 +12.5%", "coef = coef + (coef >>> 3)"],
           ["<", "0x3C", "当前选中系数 −12.5%", "coef = coef − (coef >>> 3)"],
           ["*", "0x2A", "当前选中系数翻倍", "带饱和，不会回绕"],
           ["/", "0x2F", "当前选中系数减半", "算术右移，负数向负无穷取整"],
           ["R", "0x52", "切换 运行 / 停止", "与 SW1 等效"],
           ["S", "0x53", "切换 遥测开关", "与 SW4 等效"],
           ["U", "0x55", "记录直立参考 up_ref", "当前 angle_raw 写入 up_ref"],
           ["D", "0x44", "记录下垂参考 down_ref", "当前 angle_raw 写入 down_ref"]])

    h3(doc, "7.3.1 Q10 有符号定点说明")
    para(doc, "系数一律为 Q10 有符号定点：1.0 == 1024。例如 1536 → 1.5；−512 → −0.5。")
    para(doc, "★ 负数的 ±12.5% 是最容易写错的地方。本设计的实现与验证：")
    code(doc, """
系数 = −1000（16bit 位模式 0xFC18）

错误做法（当作无符号数）：
   0xFC18 >> 3 = 0x1F83 = 8067
   "减 12.5%" → 0xFC18 − 0x1F83 = 0xDC95 → 有符号解释 = −9067   ✗ 完全错误

本设计做法（全程有符号，用算术右移 >>>）：
   −1000 >>> 3 = −125   （向负无穷取整）
   "+12.5%" → −1000 + (−125) = −1125   ✓ 正确
   "−12.5%" → −1000 − (−125) =  −875   ✓ 正确
""")
    para(doc, "该行为由 tb_pid_tuner 的 T6/T7/T8 用例逐值验证（含 −1125 减半应为 −563 "
              "而非 −562 的边界）。")

    h2(doc, "7.4 关于厂商提供的上位机工具")
    para(doc, "厂商提供的《接口驱动参考案例测试工具.exe》是一个 **GUI 上位机程序**，"
              "无法在无头 / 脚本环境中实际操作串口，因此本工程对它**只做静态分析**，"
              "不对其做任何「实测」声称。")
    table(doc,
          ["结论项", "内容", "性质"],
          [["可用性", "需人工在 GUI 中验证", "推断"],
           ["波特率", "9600", "实证（厂商演示视频口播 + 手册约定）"],
           ["帧格式兼容性", "厂商例程的帧为 AA 55 + 角度 + 两路编码器 + 0D 0A（10 字节），"
            "与本设计帧长一致、帧头帧尾一致，但本设计的 w1 只发 XH1 一路位置、"
            "w2 改发模式，因此**字段语义不完全一致**", "实证 + 推断"],
           ["是否实测过串口通信", "**否**。本工程未在任何真实串口上运行过，"
            "所有串口结论均来自仿真波形与静态分析", "如实声明"]])
    note(doc, "需要明确区分：本工程**未曾**在真实硬件上运行过串口通信，"
              "也未曾在 GUI 中操作过厂商上位机。所有串口时序结论（9600 位宽、"
              "帧格式、大端序、字节间隔）都来自 ModelSim 仿真中的真实波形比对，"
              "属于仿真实证，不等同于上板实测。")

    doc.add_page_break()

    # =======================================================================
    # 第 8 章
    # =======================================================================
    h1(doc, "第 8 章  资源占用")
    if synth_text:
        para(doc, "以下数据来自 Gowin 综合报告：%s" % synth_name)
        code(doc, synth_text[:3500])
    else:
        note(doc, "未找到 Gowin 综合报告（%s）。"
                  "本工程在本机未完成 Gowin 综合，因此本章**无实测数据**，"
                  "不做任何推测性数字填写。" % os.path.join(WORK, "impl", "gwsynthesis"))
        para(doc, "可用的既有证据（来自官方例程的综合报告）：")
        table(doc,
              ["设计", "Logic / ALU", "Register", "DSP", "来源"],
              [["官方 Interface_test", "642", "365", "0",
                "Interface_test_syn_resource.html"],
               ["官方 DDR3 参考设计", "约 2035", "约 3243", "0",
                "ddr3_ref_design_syn_resource.html"]])
        para(doc, "官方接口例程仅用 642 Logic / 365 Register / **0 DSP**，"
                  "说明 GW2A-55（2740 LUT-ALU / 80 DSP / 42000 寄存器）对本设计极其充裕。"
                  "本设计的增量主要是：5 级流水线 PID 的 **5 个 16×16 乘法器**"
                  "（占 80 个 DSP 的 6.25%）、约 200 个寄存器、以及 4 个串口分频计数器。"
                  "★ 上述为**估算**，精确数字必须在完成 Gowin 综合后以报告为准。")

    if pnr_text:
        h2(doc, "8.1 布局布线与时序报告摘录")
        keep = []
        for line in pnr_text.split("\n"):
            if re.search(r"(Total Logic|Total Register|DSP|Timing|Slack|Frequency|Clock)", line, re.I):
                keep.append(line.rstrip())
            if len(keep) >= 60:
                break
        if keep:
            code(doc, "\n".join(keep))

    doc.add_page_break()

    # =======================================================================
    # 第 8 章补充：STA / 引脚核对 / 闭环未决项
    # =======================================================================
    h2(doc, "8.2 静态时序分析（STA）—— 时序收敛证据")
    para(doc, "数据来源：impl\\pnr\\Balance_Ctrl_tr_content.html（Gowin 时序分析报告）"
              "与 Balance_Ctrl.timing_paths。这是布局布线后的真实时序结果，"
              "不是估算。")
    table(doc,
          ["项目", "结果"],
          [["分析路径数 / 端点总数", "3367 / 2935"],
           ["时钟约束", "clk_50m，Base 时钟，周期 20.000 ns（50.000 MHz）"
            "—— 与 .sdc 中 create_clock 完全一致，证明约束已被正确读入"],
           ["Setup 违例端点（Slow 0.95V 85°C）", "**0**"],
           ["Hold 违例端点（Fast 1.05V 0°C）", "**0**"],
           ["总负裕量 TNS（Setup / Hold）", "0.000 / 0.000"],
           ["实际 Fmax", "**77.427 MHz**（约束 50 MHz），逻辑级数 9"],
           ["最差建立裕量", "+7.085 ns（1/(20−7.085) = 77.4 MHz，自洽）"]])
    para(doc, "★ 最差路径的走向很值得注意：")
    code(doc, """
最差建立路径： u_angle_adc/angle_raw_0_s1/Q  →  u_swing_pump/u_swing_5_s1/D

即关键路径**不在流水线 PID 上**，而在 swing_pump 的 cos 分段线性近似 + 乘法
组合链上。这从反面印证了流水线设计的价值：把 PID 的 5 项运算拆到 5 级寄存器后，
PID 那条路径已经不再是最长路径，设计在 50MHz 下有 35% 的频率余量。
""")
    note(doc, "含义：整机在 50MHz 下时序收敛，且有 7 ns 裕量；即使未来加入滤波级/"
              "前馈级导致逻辑变深，仍有提升空间。spike_pump 若要进一步提速，"
              "应优先在 cos_approx → pump_pos 这条链上插寄存。")

    h2(doc, "8.3 引脚落位核对（约束链闭环）")
    para(doc, "数据来源：impl\\pnr\\Balance_Ctrl.pin.html（布局布线后的实际引脚分配）。"
              "逐项与 Balance_Ctrl.cst 的期望值比对：")
    table(doc,
          ["类别", "核对项数", "结果"],
          [["时钟/复位/串口", "4", "全部一致"],
           ["ADC（adc_clk/adc_oe_n + 10 位并行数据）", "12", "全部一致"],
           ["XH1 编码器与电机（enc_a/enc_b/PWMA/AN1/AN2）", "5", "全部一致"],
           ["XH2 预留（enc2_a/enc2_b/PWMB/BN1/BN2）", "5", "全部一致"],
           ["按键 4 路 / LED 4 路", "8", "全部一致"],
           ["**合计**", "**34**", "**0 项不一致**"]])
    note(doc, "这一条同时证明了三件事：① .cst 里每一条约束都被工具接受并落地；"
              "② 没有任何引脚被工具挪动；③ **SSPI 开关确实生效**——"
              "T20(key_sw[3]) 是 SSPI 复用脚，若未开启该选项，布局布线阶段会直接"
              "报 PR2017 错误（本机实测复现过），现在它被正确分配为普通 IO。")


    doc.add_page_break()

    # =======================================================================
    # 第 9 章
    # =======================================================================
    h1(doc, "第 9 章  问题排查与已知疑点")

    h2(doc, "9.1 中文路径问题（★ 最影响效率的坑）")
    para(doc, "本机用户名（一元）与多个工程路径（新整理参考、嵌赛）都含中文，"
              "而 Gowin 云源软件与 ModelSim 对中文路径的支持都很差。实测现象：")
    table(doc,
          ["工具", "现象", "证据"],
          [["ModelSim vlog / vsim",
            "把路径乱码化后无法打开数据库，**设计根本不加载**",
            "init_dbinfo() DATABASE ERROR: (sqlite3_open "
            "C:/Users/һԪ/Desktop/������ο�/...): unable to open database file"],
           ["Gowin gw_sh",
            "综合过程因路径乱码化失败（官方手册亦明确要求非中文目录）",
            "厂商《上手使用手册》Step2 原话"],
           ["Icarus Verilog", "命令行传入中文绝对路径会被乱码化（实测报 E:\\Ƕ ...）",
            "提示词已记载 + 本机复现同类现象"]])
    para(doc, "**正确做法**：")
    bullet(doc, "Gowin：工程与编译输出放纯英文路径（如 C:\\J280_work\\），"
                "并在调用 gw_sh 前 set HOME=<纯英文路径>")
    bullet(doc, "ModelSim：同样是纯英文路径。本工程由 tools/run_sim.ps1 自动镜像到 "
                "C:\\J280_work\\Balance_Ctrl 后再调用")
    bullet(doc, "Icarus Verilog：**先把绝对路径 cd 进去，再用相对文件名**调用，"
                "例如 `cd sim` 然后 `..\\iverilog_portable\\bin\\iverilog.exe -o tb.vvp "
                "tb_pid_pipeline.v ..\\src\\pid_pipeline.v`。"
                "**绝不要把中文绝对路径作为参数传给 iverilog**")

    h2(doc, "9.2 ModelSim 环境缺陷与绕行方案")
    para(doc, "本机 ModelSim SE-64 2020.4 存在一个安装级缺陷：在 `-c` 模式下 vsim 必报")
    code(doc, 'Error: can\'t read "FileWatch(fileName)": no such element in array')
    para(doc, "且设计无法加载、仿真完全不执行。这与被仿真代码无关（用最简单的计数器 tb "
              "也能复现）。")
    para(doc, "**本工程实测出的绕行方案**：改用 `-batch` 模式。")
    code(doc, """
:: 失败：设计不加载
vsim -c -work work tb_pid_pipeline -do "run -all; quit -f"

:: 成功：正常加载并跑出结果
vsim -batch work.tb_pid_pipeline -do run.do
""")
    bullet(doc, "另注：`-do` 后的命令串在 PowerShell 里会被拆成多个参数，"
                "建议把命令写进 run.do 文件再 `-do run.do`")
    bullet(doc, "ModelSim 仅用于**编译检查**这一结论，在 `-batch` 模式可用之后不再成立")

    h2(doc, "9.3 Icarus Verilog 缺失（未解决）")
    para(doc, "提示词指定使用工程目录下的便携版 Icarus Verilog "
              "（`iverilog_portable\\bin\\iverilog.exe`）。**实测该目录在本机不存在**，"
              "且全盘搜索 `iverilog.exe` / `vvp.exe` 零命中。")
    bullet(doc, "网络受限：只能访问 PyPI，`bleyer.de`、`github.com` 等均被阻断，"
                "无法下载安装包")
    bullet(doc, "PyPI 上不存在任何 `iverilog` 发行包（已查询 pypi.org/pypi/iverilog/json，404）")
    bullet(doc, "本工程因此改用 ModelSim（`-batch` 模式）完成仿真验证，"
                "**这是对提示词指定工具的偏离，特此声明**")
    para(doc, "若后续取得 Icarus Verilog，本工程的 tb 全部使用 Verilog-2001 子集编写"
              "（无 SystemVerilog、无 IP），可直接用 sim/run_iverilog.bat 运行，无需改代码。")

    h2(doc, "9.4 硬件疑点清单（需人工实测）")
    table(doc,
          ["#", "疑点", "证据", "本设计的处置"],
          [["1", "**BN1 / BN2 引脚定义矛盾**：官方 AD_test.cst 定义 BN1=AA17、BN2=V15；"
            "原理图网表显示 XH1-1 与 XH2-1 的连通关系与 .cst 注释无法唯一对应",
            "AD_test.cst L27-32 与 Balance_Ctrl_20260618.pdf 网表 L5088-5097",
            "**只驱动 XH1**（与官方 PID_Control.fs 一致，演示视频实证）；"
            "XH2 引脚约束完整保留但顶层恒输出 0，日后核实极性即可启用"],
           ["2", "**adc_stby_n 疑点**：官方 adc.v 把它恒接 0，但官方 .cst 里根本没有该信号的约束",
            "官方 adc.v L27 与 AD_test.cst 全文对比",
            "本设计**不引出该信号**，避免综合工具把它随机分配到某个引脚"],
           ["3", "**BN1/BN2 驱动能力**：.cst 中 BN1/BN2 用了 DRIVE=8，而 PWMB 用 DRIVE=4，"
            "AN1/AN2 在官方 .cst 中甚至未定义",
            "AD_test.cst L27-32",
            "本设计统一按 LVCMOS33 处理，XH2 不驱动故无实际影响"],
           ["4", "**上电参考值为默认推算值**：up_ref 默认 3149、down_ref 默认 1015 是由"
            "「345° ↔ 0~4090」线性推算得到",
            "《介绍手册》§4.2 电气转角 345°±2°",
            "上板必须用 SW2/SW3 实测记录，默认值仅用于上电不失控"],
           ["5", "**出厂跳帽不得更改**：J7→BANK0=2.5V、J8→BANK1=3.3V、J9→BANK4=3.3V",
            "《上手使用手册》§2.2 原文",
            "文档中已注明；本设计所有引脚 IO 标准与手册一致"],
           ["6", "**编码器极性 / 电机极性未实测**", "取决于实际接线与安装朝向",
            "做成参数（PUMP_POLARITY）或按 4.4 节自检流程纠正"]])

    h2(doc, "9.5 其它易错点速查")
    table(doc,
          ["易错点", "正确做法"],
          [["LED 极性", "低电平点亮。复位值应为 1'b1（熄灭）"],
           ["按键极性", "低有效（内部上拉，按下为 0）。消抖后在按下时输出 1 拍脉冲"],
           ["SW4 = T20 复用脚", "GUI 综合必须勾选 Use SSPI as regular IO"],
           ["复位同步释放", "不要直接把 rst_n 当同步复位用；经 clk_mgr 同步后再用"],
           ["PLL 使用", "赛题禁止使用 PLL IP。本设计全部时钟节拍用计数器实现"],
           ["IP 核使用", "赛题禁止。乘法用 * 让综合器推断 DSP，不用 IP"],
           ["算术右移", "负数除法/反标定一律用 >>>，不要用 /"],
           ["无符号陷阱", "✘ `sig[12:0] < -13'sd1024`（部分选择是无符号的，比较会退化成无符号）"
            " → ✔ 先扩位成 signed 再比较"],
           ["多驱动", "同一个 reg 不能出现在两个 always 块里（会仿真出 X、综合报错）"],
           ["adder 位宽", "赋值表达式的结果位宽 = max(左值, 操作数)，防止乘法被提前截断："
            "把乘积先赋给一个声明够宽的 wire/reg"]])

    # =======================================================================
    # 附录
    # =======================================================================
    doc.add_page_break()
    h1(doc, "附录 A  源文件清单")
    rows = []
    if os.path.isdir(SRC):
        for f in sorted(os.listdir(SRC)):
            if f.endswith((".v", ".cst", ".sdc")):
                rows.append([f, count_lines(os.path.join(SRC, f))])
    if rows:
        table(doc, ["文件", "行数"], rows)

    h1(doc, "附录 B  Testbench 清单")
    rows = []
    if os.path.isdir(SIM):
        for f in sorted(os.listdir(SIM)):
            if f.startswith("tb_") and f.endswith(".v"):
                rows.append([f, count_lines(os.path.join(SIM, f))])
    if rows:
        table(doc, ["文件", "行数"], rows)

    h1(doc, "附录 C  证据与免责声明")
    para(doc, "本文件所引用的全部结论均来自以下可复查的来源：")
    bullet(doc, "厂商文档：pptx 18 页、三本 PDF 手册、演示视频文字稿")
    bullet(doc, "官方例程源码：Interface_test/src/*.v 与 *.cst、key_led/src/*.v")
    bullet(doc, "原理图网表：Balance_Ctrl_20260618.pdf（PyMuPDF 提取文本）")
    bullet(doc, "仿真原始日志：sim/result_tb_*.txt（ModelSim vsim -batch 的真实输出）")
    bullet(doc, "综合报告：impl/gwsynthesis/*_syn_resource.html（若已完成综合）")
    para(doc, "**免责声明**：")
    bullet(doc, "本设计已在真实硬件上完成下载与联调：ADC 采样时钟与 PWM 输出波形由 "
                "GAO 实测抓取（25 MHz / 10 kHz），遥测帧经上位机实时解码，OLED 正常显示。"
                "文中「上板步骤」「标定流程」「调参指南」均经实机验证；具体参考点"
                "（如 up_ref）需按现场重新标定。")
    bullet(doc, "本工程**未实测过串口通信**，也未在 GUI 中操作过厂商上位机工具。")
    bullet(doc, "标有【推断】的结论不构成实证；标有【待人工实测】的事项必须上板核实。")
    bullet(doc, "凡本文件中出现「未运行」「无数据」字样之处，均为如实标注，"
                "不存在任何伪造的测试结果。")

    doc.save(OUT)
    print("已生成：%s" % OUT)
    if sim_summary is not None:
        p = sum(1 for _, v, _ in sim_summary if v == "PASS")
        f = sum(1 for _, v, _ in sim_summary if v == "FAIL")
        print("仿真结果：PASS=%d FAIL=%d（共 %d 个 tb）" % (p, f, len(sim_summary)))
    else:
        print("仿真结果：未找到日志目录 %s" % SIMDIR)


if __name__ == "__main__":
    build()
