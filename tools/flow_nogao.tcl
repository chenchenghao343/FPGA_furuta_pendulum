# =============================================================================
# flow_nogao.tcl -- J280 Balance_Ctrl, NO GAO
# Device : GW2A-LV55PG484C8/I7  (GW2A-55C)
# Tool   : Gowin V1.9.12.x   ->   gw_sh.exe flow_nogao.tcl
# -----------------------------------------------------------------------------
# ASCII ONLY.  Chinese original kept as tools/flow_nogao.tcl.zh
#
# RUN THIS FROM THE PROJECT ROOT, not from tools/:
#     set HOME=C:\J280_work
#     "D:\Gowin_V1.9\...\IDE\bin\gw_sh.exe" flow_nogao.tcl
# The src/... paths below are relative, so running it from tools/ fails with
# EX5998 "Cannot open Verilog file".
#
# WHY HOME MUST BE REDIRECTED: Gowin writes cache/license files under %HOME%.
# If HOME points at a path containing non-ASCII characters the run fails.
#
# DIFFERENCE FROM tools/flow.tcl: exactly one line --
#     add_file src/Balance_Ctrl.rao      <-- commented out here
# Everything else (device, options, file list) is identical on purpose.
#
# OUTPUT: impl\pnr\Balance_Ctrl.fs
#         (tools/flow.tcl with GAO produces impl\pnr\ao_0.fs instead -- they
#          are NOT the same artifact, do not mix them up)
#
# NOTE: status text is written as #comments rather than puts.  A puts that
#   contains a Windows path such as "...\pnr\..." makes Tcl eat the backslash
#   as an escape (\p is invalid), the string never closes, and the run exits
#   non-zero EVEN THOUGH the bitstream was generated correctly.  Cost real
#   debugging time twice; hence comments.
# =============================================================================

puts "===== Balance_Ctrl build start (no GAO) ====="

set_device -name GW2A-55C -device_version C GW2A-LV55PG484C8/I7
set_option -top_module pendulum_top
set_option -output_base_name Balance_Ctrl

# -----------------------------------------------------------------------------
# MANDATORY: use SSPI as regular IO
#   T20 is used as key_sw[3] (SW4) and T20 is an SSPI function pin.  Without
#   this option place&route aborts with:
#       ERROR (PR2028) : The constrained location is useless in current package
#       ERROR (PR2017) : 'key_sw[3]' cannot be placed according to constraint,
#                        for the location is a dedicated pin (SSPI)
#   The GUI needs the same box ticked under
#   Project -> Configuration -> Place & Route -> "Use SSPI as regular IO".
# -----------------------------------------------------------------------------
set_option -use_sspi_as_gpio 1

# -----------------------------------------------------------------------------
# RTL sources
# -----------------------------------------------------------------------------
add_file src/clk_mgr.v
add_file src/key_filter.v
add_file src/qep_encoder.v
add_file src/motor_drv.v
add_file src/angle_adc.v
add_file src/limit_prot.v
add_file src/ang_deriv.v
add_file src/traj_ramp.v
add_file src/traj_sine.v
add_file src/pos_ctrl.v
add_file src/pos_pd.v
add_file src/pos_cmd.v
add_file src/ctrl_fsm.v
add_file src/swing_pump.v
add_file src/pid_pipeline.v
add_file src/uart_tx.v
add_file src/uart_rx.v
add_file src/uart_link.v
add_file src/pid_tuner.v
add_file src/ext_keys.v
add_file src/numfmt.v
add_file src/oled_disp.v
add_file src/oled_ssd1306.v
add_file src/pendulum_top.v

# constraints
add_file src/Balance_Ctrl.cst
add_file src/Balance_Ctrl.sdc

# GAO deliberately NOT added here -- that is the whole point of this script.
# add_file src/Balance_Ctrl.rao

run all

puts "===== Balance_Ctrl build done (no GAO) ====="