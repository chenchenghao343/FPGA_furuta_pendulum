#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
 check_init_consistency.py —— 固件 INIT 与上位机 INIT 的一致性检查
-----------------------------------------------------------------------------
 为什么需要它（这是本项目真实踩过的坑，不是假想）：

   遥测帧只有 22 字节，里面【没有系数字段】，所以上位机无法回读系数，
   只能按"自己发过的命令"用同一套算术复算。于是：

       面板显示值 = 固件实际值 + (host_INIT - firmware_INIT)

   是一个【恒定偏移】。面板不会报错、不会提示，只会安静地显示错数 ——
   用户据此调参就会越调越偏。

   历史上真的漏同步过两次：
       v5.3         固件 12950/0/9375/256/1600     host 同步了   OK
       v5.4 ~ v5.7  固件 22406/0/3372/128/0        host 同步了   OK
       v5.8         固件 KP_X 620 / KD_X 10240     host 漏了     X
       v5.10        固件 21231/256/2697/-32/64     host 漏了     X

 本脚本直接解析两边的源文件逐项比对，不一致就非零退出。

 用法：
     python tools/check_init_consistency.py
     返回码 0 = 一致；1 = 不一致；2 = 解析失败
=============================================================================
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TOP  = os.path.join(ROOT, "src", "pendulum_top.v")
TUNER = os.path.join(ROOT, "src", "pid_tuner.v")
HOST = os.path.join(HERE, "serial_host.py")

# 固件参数名  ->  上位机 INIT 的键名
PAIRS = [
    ("KP_TH_INIT", "KP"),
    ("KI_TH_INIT", "KI"),
    ("KD_TH_INIT", "KD"),
    ("KP_X_INIT",  "PP"),
    ("KD_X_INIT",  "PD"),
    ("SWING_INIT", "SW"),
]


def _num_from(text, param, where):
    """从一段文本里解析 .PARAM (...) 或 parameter ... PARAM = 值"""
    # ① 实例覆盖形式：  .KP_X_INIT  (-32'sd32),
    m = re.search(
        r"\.%s\s*\(\s*(-?)\s*(\d+)?\s*'\s*[sS]?\s*([dDhHbBoO])\s*(-?\d+)\s*\)" % param,
        text)
    if m:
        sign1, _size, base, digits = m.groups()
        if digits.startswith("-"):
            # 32'sd-32 不是合法 Verilog —— 符号必须写在尺寸说明符之外
            raise ValueError(
                "%s 里的 .%s 写成了 32'sd%s —— 这不是合法 Verilog，"
                "应写成 -32'sd%s" % (where, param, digits, digits.lstrip("-")))
        v = int(digits, {"d": 10, "h": 16, "b": 2, "o": 8}[base.lower()])
        return -v if sign1 == "-" else v
    # ② 模块默认值形式：  parameter [31:0] SWING_INIT = 32'd2600
    m = re.search(
        r"parameter[^;=]*\b%s\b\s*=\s*(-?)\s*(\d+)?\s*'\s*[sS]?\s*([dDhHbBoO])\s*(-?\d+)"
        % param, text)
    if m:
        sign1, _size, base, digits = m.groups()
        v = int(digits, {"d": 10, "h": 16, "b": 2, "o": 8}[base.lower()])
        return -v if sign1 == "-" else v
    return None


# ---------------------------------------------------------------------------
# ★ v7.2：lite 版没有 pid_tuner.v —— 5 个系数是 pendulum_top.v 里的常量：
#       wire signed [31:0] kp_th_w = 32'sd18003;
#       wire [31:0]        swing_duty_w32 = 32'd2600;
#   所以第三种（也是 lite 唯一有效的）来源是这些 *_w 常量。
# ---------------------------------------------------------------------------
LITE_CONST = {
    "KP_TH_INIT": "kp_th_w",
    "KI_TH_INIT": "ki_th_w",
    "KD_TH_INIT": "kd_th_w",
    "KP_X_INIT":  "kp_x_w",
    "KD_X_INIT":  "kd_x_w",
    "SWING_INIT": "swing_duty_w32",
}


def _const_from(text, var, where):
    """③ lite 顶层常量形式：  kp_th_w = 32'sd18003 / swing_duty_w32 = 32'd2600"""
    if not var:
        return None
    m = re.search(
        r"\b%s\b\s*=\s*(-?)\s*(\d+)?\s*'\s*[sS]?\s*([dDhHbBoO])\s*(-?\d+)"
        % re.escape(var), text)
    if m:
        sign1, _size, base, digits = m.groups()
        v = int(digits, {"d": 10, "h": 16, "b": 2, "o": 8}[base.lower()])
        return -v if sign1 == "-" else v
    return None


def parse_firmware(top_path, tuner_path):
    """先看顶层实例覆盖，没有则回退到 pid_tuner.v 的模块默认值，
    再没有则回退到 lite 版 pendulum_top.v 里的 *_w 常量。

    ★ 为什么要回退：顶层并不覆盖全部 6 个参数。例如 v5.8 起
      顶层没有 .SWING_INIT(...)，实际生效的是 pid_tuner.v 里的
      parameter SWING_INIT = 32'd2600。只查顶层会漏判。
    ★ v7.2：lite 版整个没有 pid_tuner.v（系数是常量），所以还必须
      再回退一层，否则 lite 树里这个脚本会直接抛 FileNotFoundError。
    """
    top = open(top_path, encoding="utf-8", errors="replace").read()
    tun = ""
    if os.path.isfile(tuner_path):
        tun = open(tuner_path, encoding="utf-8", errors="replace").read()
    out = {}
    for param, _ in PAIRS:
        v = _num_from(top, param, os.path.basename(top_path))
        if v is None and tun:
            v = _num_from(tun, param, os.path.basename(tuner_path))
        if v is None:
            v = _const_from(top, LITE_CONST.get(param, ""),
                            os.path.basename(top_path))
        out[param] = v            # 可能仍是 None，交给 main 报缺失
    return out


def parse_host(path):
    """从 serial_host.py 里取 INIT = {...} 这一行"""
    src = open(path, encoding="utf-8", errors="replace").read()
    m = re.search(r"^\s*INIT\s*=\s*\{([^}]*)\}", src, re.M)
    if not m:
        raise ValueError("在 %s 里找不到 INIT = {...}" % path)
    out = {}
    for k, v in re.findall(r"['\"](\w+)['\"]\s*:\s*(-?\d+)", m.group(1)):
        out[k] = int(v)
    return out


def main():
    try:
        fw = parse_firmware(TOP, TUNER)
        ho = parse_host(HOST)
    except Exception as exc:
        print("[解析失败] %s" % exc)
        return 2

    print("=" * 72)
    print(" 固件 INIT  vs  上位机 INIT  一致性检查")
    print("=" * 72)
    print(" 固件源  : %s  (+ %s 的模块默认值)" % (os.path.relpath(TOP, ROOT), os.path.relpath(TUNER, ROOT)))
    print(" 上位机源: %s" % os.path.relpath(HOST, ROOT))
    print("-" * 72)
    print(" %-4s %-14s %14s %14s %10s" % ("键", "固件参数", "固件", "上位机", "状态"))
    print("-" * 72)

    bad = 0
    for param, key in PAIRS:
        f = fw.get(param)
        h = ho.get(key)
        if f is None or h is None:
            status = "缺失"
            bad += 1
        elif f == h:
            status = "OK"
        else:
            status = "不一致"
            bad += 1
        print(" %-4s %-14s %14s %14s %10s" % (key, param, f, h, status))
    print("-" * 72)

    if bad == 0:
        print(" 结论：一致（%d 项全部匹配）" % len(PAIRS))
        print("=" * 72)
        return 0

    print(" 结论：有 %d 项不一致！" % bad)
    print()
    print(" ★ 后果：面板显示的系数 = 固件实际值 + (上位机 INIT - 固件 INIT)，")
    print("   是一个恒定偏移。面板不会报错，只会安静地显示错数。")
    print(" ★ 修法：把 上位机/serial_host.py 里 GainTracker.INIT 改成固件的值，")
    print("   改完重新打包 J280_Host.exe（PyInstaller）。")
    print("=" * 72)
    return 1


if __name__ == "__main__":
    sys.exit(main())
