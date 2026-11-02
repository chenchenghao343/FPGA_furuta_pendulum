# =============================================================================
# ___?: flow.tcl
# ___   : J280 ___?FPGA ___
# ___   : GW2A-LV55PG484C8/I7  (GW2A-55C)
# ___   : Gowin ___ V1.9.12.x  ___? gw_sh.exe flow.tcl
# -----------------------------------------------------------------------------
# ___ ___
#
#   1) ___?Balance_Ctrl ___**___?*___ C:\J280_work\Balance_Ctrl
#   2) ___?cmd ___
#        set HOME=C:\J280_work
#        "D:\Gowin_V1.9\Gowin\Gowin_V1.9.12.02_SP2_x64\IDE\bin\gw_sh.exe" flow.tcl
#
#   ___ HOME___?#     Gowin ___ %HOME% ___?license ___ HOME ___
#     ___ C:\Users\___?#     ___?___"___?Step2 ___?#
#   ___?___ SSPI___?#     ___ T20 ___ key_sw[3]___W4___20 ___?SSPI ___?#     ___?**GUI** ___ Project ___?Configuration ___?Place & Route ___?#     "Use SSPI as regular IO"___?T20 ___ SSPI___W4 ___?#     ___ gw_sh___ set_option ___?# =============================================================================

puts "===== Balance_Ctrl ___?====="

# ---------------------------------------------------------------------------
# ___?# ___?set_device ___art number___
#   ___?GW2A-LV55PG484C8/I7___?#     "there are more than one device named GW2A-LV55PG484C8/I7,
#      please specify its name as well"
#   ___ part number ___ device name___/___?#   ___?.gprj___?#     <Device name="GW2A-55C" pn="GW2A-LV55PG484C8/I7">gw2a55c-005</Device>
# ---------------------------------------------------------------------------
set_device -name GW2A-55C -device_version C GW2A-LV55PG484C8/I7
set_option -top_module pendulum_top

# ---------------------------------------------------------------------------
# ___?# ---------------------------------------------------------------------------
set_option -output_base_name Balance_Ctrl

# ---------------------------------------------------------------------------
# ___ ___se SSPI as regular IO___?#   ___?*___?gw_sh ___?*___?#       ERROR (PR2028) : The constrained location is useless in current package
#       ERROR (PR2017) : 'key_sw[3]' cannot be placed according to constraint,
#                        for the location is a dedicated pin (SSPI)
#   ___ run all ___?#   ___?gw_sh ___?**___?*___?#         ___?#   ___ impl/*_process_config.json ___?"SSPI": true___
# ---------------------------------------------------------------------------
set_option -use_sspi_as_gpio 1

# ---------------------------------------------------------------------------
# ___RTL___?# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# ___
# ---------------------------------------------------------------------------
add_file src/Balance_Ctrl.cst
add_file src/Balance_Ctrl.sdc

# ---------------------------------------------------------------------------
# ___?GAO ___.rao___?#   ___?GAO ___?GAO Analyzer ___?#   ___ GAO ___?impl/pnr/ao_0.fs___?Balance_Ctrl.fs___
#         ___?11MB ___?18MB___?#   ___?GAO ___?# ---------------------------------------------------------------------------
add_file src/Balance_Ctrl.rao

# ---------------------------------------------------------------------------
# ___ ___?___ ___?___
# ---------------------------------------------------------------------------
run all

puts "===== Balance_Ctrl ___ ====="
# puts "___mpl\\pnr\\Balance_Ctrl.fs  ___?Programmer ___?SRAM___?
# puts "___mpl\\gwsynthesis\\*_syn_resource.html  ___"
# puts "      impl\\pnr\\Balance_Ctrl.rpt.txt          ___ + ___?
