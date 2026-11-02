# -*- coding: utf-8 -*-
"""抽取用于文档的关键代码片段，并校验其确实存在于源文件中"""
import io, os, json

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, 'src')
OUT = os.path.join(HERE, 'code_snippets.json')

def grab(fname, start_pat, nlines, must_contain=None):
    """从 fname 中找含 start_pat 的行，取其后 nlines 行"""
    path = os.path.join(SRC, fname)
    lines = io.open(path, encoding='utf-8').read().split('\n')
    for i, ln in enumerate(lines):
        if start_pat in ln:
            seg = lines[i:i+nlines]
            txt = '\n'.join(seg).rstrip()
            if must_contain and must_contain not in txt:
                continue
            return txt
    print("!! 未找到片段: %s 里 %r" % (fname, start_pat))
    return None

out = {}

# ---- pid_pipeline: S1~S5 主流水线 ----
out['pipe_s1'] = grab('pid_pipeline.v', 's1_err_th <= err_th;', 12)
out['pipe_s2s3'] = grab('pid_pipeline.v', 's2_p_term  <= s1_kp_th * s1_err_th;', 16)
out['pipe_s4s5'] = grab('pid_pipeline.v', 's4_sum <= s3_p_term', 10)
out['pipe_antiwindup'] = grab('pid_pipeline.v', 'wire i_freeze =', 3)
out['pipe_shift'] = grab('pid_pipeline.v', 's5_u_full <= s4_sum >>> Q_FMT;', 2)

# ---- angle_adc ----
out['adc_avg'] = grab('angle_adc.v', 'else if (out_pulse) begin', 14)

# ---- qep_encoder: 4 倍频查表 ----
out['qep_case'] = grab('qep_encoder.v', 'case ({state_last, state_now})', 14)

# ---- motor_drv ----
out['motor_dir'] = grab('motor_drv.v', 'if (u_ctrl > $signed(DEADBAND', 18)

# ---- uart_rx ----
out['rx_start'] = grab('uart_rx.v', 'wire start_bit_detect', 1)
out['rx_mid'] = grab('uart_rx.v', 'baud_cnt == BIT_CNT_MID', 2)
out['rx_shift'] = grab('uart_rx.v', 'rx_data <= {rx_sync3, rx_data[7:1]};', 1)

# ---- uart_tx ----
out['tx_tick'] = grab('uart_tx.v', 'wire bit_tick = busy &&', 1)
out['tx_latch'] = grab('uart_tx.v', 'tx_shift <= pi_data;', 1)

# ---- pid_tuner: Q10 有符号算术 ----
out['tuner_up'] = grab('pid_tuner.v', 'g_new = f_sat(g_cur + (g_cur >>> 3));', 1)
out['tuner_neg'] = grab('pid_tuner.v', 'g_new = f_sat(-g_cur);', 1)
out['tuner_sat'] = grab('pid_tuner.v', 'function signed [31:0] f_sat;', 6)

# ---- swing_pump ----
out['pump_cos'] = grab('swing_pump.v', 'wire signed [16:0] cos_raw', 8)
out['pump_dir'] = grab('swing_pump.v', 'wire pump_pos = ((dtheta >= 0) == (cos_approx >= 0));', 6)

# ---- ctrl_fsm ----
out['fsm_case'] = grab('ctrl_fsm.v', 'MODE_IDLE: begin', 20)

# ---- clk_mgr ----
out['clk_rst'] = grab('clk_mgr.v', 'rst_sync_chain <= {rst_sync_chain', 2)

# ---- limit_prot ----
out['lim_sign'] = grab('limit_prot.v', 'assign pos_ext = $signed(pos_rel);', 1)

# ---- pos_ctrl ----
out['pos_ramp'] = grab('pos_ctrl.v', 'wire signed [W:0] dead_ext = dead_en', 6)

# ---- ang_deriv ----
out['deriv_sum'] = grab('ang_deriv.v', 'dtheta <= dth_new_sum >>> FILT_SHIFT;', 2)

# ---- 校验：每段都必须非空
for k, v in out.items():
    if not v:
        print("   (跳过缺失片段 %s)" % k)
        continue
    print("[OK] %-18s %2d 行" % (k, len(v.split('\n'))))

io.open(OUT, 'w', encoding='utf-8').write(json.dumps(out, ensure_ascii=False, indent=1))
print("\n共 %d 段，已存 %s" % (len(out), OUT))
