# J280 旋转倒立摆（Furuta Pendulum）FPGA 控制系统

基于高云 **GW2A-LV55PG484C8/I7** 的全数字旋转倒立摆控制系统：能量泵起摆 → 串级 PID 平衡 → 定点往返 → 正弦轨迹摆动，配套 OLED 实时显示、9600 bps 遥测、串口在线调参上位机与波形上位机。

纯 Verilog-2001 实现，**不使用 PLL、不使用任何 IP 核**，全设计单时钟域（50 MHz）。

> 赛题要求对应：定点模式下限定 2000 counts 实现摆臂旋转两圈；轨迹状态下摆臂按 0.2 Hz 频率、200 counts 振幅正弦往复，并可抗扰恢复；额外增加 OLED 显示摆杆状态、摆臂位置与两种运动模式；配套调参上位机与波形显示上位机。

---

## 1. 功能清单

### 1.1 控制

| 功能 | 说明 | 文件 |
|---|---|---|
| 四状态机 | `IDLE → SWING → BALANCE`，任一状态可手动停机回 `IDLE`；限位/故障进 `FAULT` | `ctrl_fsm.v` |
| 起摆（能量泵） | 继电器式能量控制 `u = −D·sign(θ̇·cosφ)`，`cos` 用 `1024−\|φ\|` 分段线性近似（纯加减，**0 DSP**）；静止时固定方向踢摆打破对称；进入捕获带（`\|φ\|<130` 且 `\|θ̇\|<55`）自动交棒给平衡环 | `swing_pump.v` |
| 平衡（串级 PID） | 内环 **3 ms** 倾角 PID（5 级流水，DSP 硬核乘加）；外环 **30 ms** 位置 PD，输出 `φ_bias` **移动内环角度目标**（不是并联力矩），带 ±250 counts 限幅与每拍 ≤2 counts 斜率限制 | `pid_pipeline.v` / `pos_pd.v` |
| 定点往返 | 串口 `M` 进入指令编辑，`+`/`−` 每次 ±100 counts，`!` 提交；`0` 归零；累计行程上限 ±3201 counts（约 3 圈），斜坡限速 2 counts/3 ms | `pos_cmd.v` / `traj_ramp.v` |
| 正弦轨迹 | 幅值 **200 counts**（≈±67.5°）、频率 **0.2 Hz**（周期 5.0 s），板载 SW2 在过零点切换 | `traj_sine.v` |
| 限位保护 | 位置 `\|x\|>6000` counts 或角度越量程 → `FAULT`，电机立即停 | `limit_prot.v` |

### 1.2 人机与通信

| 功能 | 说明 |
|---|---|
| OLED 显示 | 128×64 SSD1306，I²C 软件时序，显示模式/角度/位置/定点目标/参数，约 21 帧/s |
| 遥测 | 9600 8N1，28 字节定长帧，约 **13 帧/s** |
| 在线调参 | 串口单字符命令实时修改 6 个系数（有符号 Q10，32 bit 饱和），**无需重新综合/下载** |
| 上位机 | `上位机/J280_Host.exe`（调参 + 数字面板）、`上位机/J280_Scope.exe`（波形显示） |

### 1.3 按键

| 位置 | 键 | 功能 |
|---|---|---|
| 板载 | SW1 | 启动 / 停止（起摆 ↔ 平衡 由状态机决定） |
| 板载 | SW2 | 正弦轨迹开关 |
| 板载 | SW6 | 硬件复位（异步置位、同步释放 8 拍） |
| 底板 | SW1～SW5 | 启动 / `+100` / `−100` / `M`（进入位置编辑）/ `!`（提交） |

---

## 2. 关键参数

| 项 | 值 | 出处 |
|---|---|---|
| 系统时钟 | **50 MHz**（引脚 M19） | `Balance_Ctrl.cst` |
| ADC 时钟 | **25 MHz**（50 MHz 二分频输出，占空比 50 %） | `angle_adc.v` |
| ADC 采样间隔 | **187.5 µs**（9374 拍） | `SMP_STEP` |
| 内环控制周期 | **2.9997 ms ≈ 3 ms（333 Hz）**，= 16 点块平均窗口 `NPT × STEP_P = 149984` 拍 | `angle_adc.v` |
| 外环控制周期 | **30 ms（33 Hz）** | `pos_pd` 的 `DIV=10` |
| 电机 PWM | **10 kHz**，`PWM_PERIOD = DUTY_MAX = 5000` 拍 | `motor_drv.v` |
| 输出死区 | `\|u\| < 20` 时占空比置 0 | `DEADBAND` |
| 编码器 | 正交 4 倍频，1067 counts/圈 | `qep_encoder.v` |
| 串口 | 9600 bps，8 数据位 / 无校验 / 1 停止位 | `uart_tx/rx.v` |

---

## 3. 目录结构

```
J280_furuta_pendulum/
├─ src/                  RTL 源码（24 个 .v）+ 约束
│   ├─ pendulum_top.v        顶层：模块互联与参数
│   ├─ clk_mgr / key_filter / ext_keys        时钟复位、消抖、底板 5 键
│   ├─ angle_adc / ang_deriv                  ADC 采集与 16 点块平均、角速度微分
│   ├─ pid_pipeline / pid_tuner               内环流水线 PID、串口在线调参
│   ├─ pos_pd / pos_ctrl / pos_cmd / traj_ramp / traj_sine   位置环与轨迹
│   ├─ swing_pump / ctrl_fsm                  起摆能量泵、状态机
│   ├─ motor_drv / limit_prot / qep_encoder   电机驱动、限位、编码器
│   ├─ uart_tx / uart_rx / uart_link          串口与 28 字节遥测帧
│   ├─ oled_disp / oled_ssd1306 / numfmt      OLED 显示与十进制格式化
│   ├─ Balance_Ctrl.cst      引脚约束
│   ├─ Balance_Ctrl.sdc      时序约束（50 MHz）
│   └─ Balance_Ctrl.rao      GAO 调试配置（正式位流不使用）
├─ sim/                  9 个核心 testbench（自检式，打印 TEST_RESULT: PASS/FAIL）
├─ tools/                构建、仿真、文档脚本（无二进制）
├─ 上位机/               PC 端程序：2 个 exe + 3 个 py 源码 + 批处理
├─ impl/                 编译产物（综合报告、PnR 报告、位流 .fs）
├─ Balance_Ctrl.gprj     高云 IDE 工程文件
└─ README.md
```

---

## 4. 编译方法（Gowin EDA V1.9.12）

### 4.1 正式位流（不含 GAO，下载到板子上跑控制用这个）

```bat
set HOME=C:\J280_work
cd /d E:\J280_furuta_pendulum
"D:\Gowin_V1.9\Gowin\Gowin_V1.9.12.02_SP2_x64\IDE\bin\gw_sh.exe" tools\flow_nogao.tcl
```

产物：`impl\pnr\Balance_Ctrl.fs`

**三个必须注意的点**（否则会报莫名其妙的错）：

1. **必须在工程根目录执行** —— 脚本里的 `src/...` 是相对路径，在 `tools\` 里跑会报 `EX5998 Cannot open Verilog file`；
2. **必须把 `HOME` 指到纯 ASCII 路径** —— 高云会在 `%HOME%` 下写缓存，中文路径直接失败；
3. `gw_sh.exe` **即使成功也可能返回非 0 退出码**（Tcl 串里含 `\p` 这类反斜杠路径会吞掉转义），判断依据是**有没有生成 `impl\pnr\Balance_Ctrl.fs`**，不要只看 exit code。

### 4.2 调试位流（带 GAO 逻辑分析仪）

```bat
"D:\...\IDE\bin\gw_sh.exe" tools\flow.tcl             :: 带 GAO（与 flow_nogao 只差一行 add_file src/Balance_Ctrl.rao）
"D:\...\IDE\bin\gw_sh.exe" tools\build_gao_ascii.tcl  :: 用指定 .rao 的 GAO 构建
```

> 两者产物**不是同一个文件**：正式位流是 `impl\pnr\Balance_Ctrl.fs`，GAO 位流是 `impl\pnr\ao_0.fs`，不要混用。实测 GAO 版本额外占用约 **+609 LUT、+12 块 BSRAM、+4 个 I/O**，只用于抓波形。

### 4.3 IDE 方式

用 Gowin FPGA Designer 打开 `Balance_Ctrl.gprj` → Synthesize → Place & Route → Programmer 下载。

---

## 5. 仿真方法（ModelSim）

```powershell
# 跑全部 9 个核心 testbench
powershell -ExecutionPolicy Bypass -File tools\run_sim.ps1 -SrcRoot E:\J280_furuta_pendulum -Work C:\J280_work\sim

# 只跑其中一个
powershell -ExecutionPolicy Bypass -File tools\run_sim.ps1 -SrcRoot E:\J280_furuta_pendulum -Work C:\J280_work\sim -Only tb_pid_pipeline
```

- 脚本先把 `src\` 与 `sim\` **镜像到纯 ASCII 路径**（`-Work`）再调 `vlog/vsim`：ModelSim 在中文路径下会报 `sqlite3_open ... unable to open database`；
- 用 `vsim -batch` 而不是 `-c`：本机 ModelSim 2020.4 的 `-c` 有 `FileWatch(fileName)` 缺陷，设计加载不起来；
- 每个 tb 自带期望值比对，结果写入 `result_tb_*.txt`，末行形如 `TEST_RESULT: PASS`；`tools\summarize_regression.ps1` 可汇总成表。

核心 testbench 与覆盖点：

| testbench | 覆盖内容 |
|---|---|
| `tb_top_smoke` | 整机集成：ADC 接口、编码器正交序列、20 ms 按键、真实 9600 串口解出 28 字节遥测帧 |
| `tb_top_poscmd` | 端到端：UART `M/+/-/!` → `pos_cmd` → `traj_ramp` → 位置目标贯通 |
| `tb_pid_pipeline` | 内环：流水延迟、吞吐率、饱和、积分限幅、有符号运算 |
| `tb_pos_pd` | 外环：`φ_bias` 限幅/斜率、`DIV=10` 分频、外环不并联 |
| `tb_swing_pump` | 能量泵：判据符号比较、死区踢摆、捕获交棒、极性参数 |
| `tb_pid_tuner` | 在线调参：±12.5 %、翻倍/减半/取反、32 bit 饱和、单拍脉冲 |
| `tb_adc_prod` | ADC：25 MHz `adc_clk`、每窗 16 点、187.5 µs 间隔、累加与输出不撞拍 |
| `tb_uart_link` | 遥测帧：28 字节、大端序、字节间隔、命令透传 |
| `tb_motor_drv` | 电机驱动：方向真值表、死区、占空比精度、限幅、复位安全 |

---

## 6. 串口协议

**物理层**：9600 bps，8 数据位，无校验，1 停止位。

### 6.1 命令（单字符）

| 字符 | 功能 |
|---|---|
| `1`～`6` | 选择要修改的系数：1=kp_th 2=ki_th 3=kd_th 4=kp_x 5=kd_x 6=swing_duty |
| `>` / `<` | 选中系数 ±12.5 % |
| `*` / `/` | 翻倍 / 减半 |
| `-` | 取反 |
| `+` | 加 64（Q10 的 0.0625） |
| `R` | 运行 / 停止 |
| `S` | 遥测开关（上电默认开） |
| `U` / `D` | 记录直立参考 / 下垂参考 |
| `P` | 翻转起摆能量泵极性（换电机或换安装方向时用） |
| `A` | 切换定点模式 |
| `M` | 进入位置指令编辑；`+`/`-` 每次 ±100 counts；`!` 提交 |
| `0` | 定点归零 |

### 6.2 遥测帧（28 字节）

```
AA 55 | w0 w1 ... w11（各 2 字节，大端） | 0D 0A
```

| 字 | 含义 | 字 | 含义 |
|---|---|---|---|
| w0 | 角度 `angle_raw`（12 bit ADC） | w6 | P 项 |
| w1 | 位置 `pos`（编码器 counts） | w7 | D 项 |
| w2 | 模式（0 IDLE / 1 SWING / 2 BALANCE / 3 FAULT） | w8 | P+D 和 |
| w3 | 电机控制量 `u_motor` | w9 | 定点目标 `x_settle` |
| w4 | 有效倾角 `phi_eff` | w10 | 位置偏差 `err_x` |
| w5 | 角速度 `dphi` | w11 | 倾角偏置 `phi_bias` |

---

## 7. 资源占用与时序（GW2A-55，正式位流）

| 资源 | 用量 | 占比 |
|---|---|---|
| Logic（LUT 3807 + ALU 620） | 4487 / 54720 | **9 %** |
| Register | 1629 / 42000 | 4 % |
| CLS | 2789 / 27360 | 11 % |
| **DSP** | **10.5 / 20** | **53 %（主要瓶颈）** |
| BSRAM | 1 / 140 | <1 %（OLED 帧缓冲） |
| I/O | 42 / 320 | 14 % |

DSP 分布：内环流水线 6 个、外环 PD 2 个、正弦查表 1 个、OLED 数字格式化 2 个；**起摆能量泵 0 个**（纯符号比较与加减）。

**时序**：约束 50 MHz，实测 **Fmax 50.678 MHz**；5026 条分析路径 setup/hold **违例均为 0**，TNS = 0，最差路径 slack **+0.268 ns**（位于 OLED 渲染通路）。

---

## 8. 实测波形（Gowin GAO，采样时钟 50 MHz）

| 信号 | 实测 | 端口 |
|---|---|---|
| `adc_clk` | 25.000 MHz，周期 40 ns，占空比 50 % | G18 |
| `PWMA` | 10.000 kHz（周期 100 µs），平衡态占空比 5.2 %（对应 `u_motor = 259`，满量程 ±5000） | Y18 |
| `AN1` / `AN2` | 方向脚：`10` 正转 / `01` 反转 / `00` 停 | Y17 / Y19 |

---

## 9. 已知限制

- 增益初值为经验值，`up_ref` 必须现场用 `U` 命令重新标定（默认值只对某一次装配有效）；
- 只驱动 **XH1** 一路电机接口，`enc2_a/enc2_b/PWMB/BN1/BN2` 为预留；
- 起摆极性与幅值需按实际电机接线用 `P` 命令或 `6` 通道修正；
- 位置环与倾角环的增益标度不同（Q12 / Q10），跨通道搬参数时要换算。

---

## 10. 工具与致谢

- 编译：Gowin EDA V1.9.12.02_SP2（`gw_sh.exe` 命令行流程脚本见 `tools/`）
- 仿真：ModelSim SE-64 2020.4
- 器件手册、原理图与例程来自高云半导体与赛题资料
