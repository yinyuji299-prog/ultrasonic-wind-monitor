# 机柜超声波风速监测系统

基于易灵思 Trion T20F256 FPGA 的超声波风速监测终端。两枚换能器对射测时，用互相关求声波到达时间差，换算风速与对流热耗散。

无转动部件，适合机柜、井场等长期无人值守场景。

---

## 目录导航

| 位置 | 内容 |
|---|---|
| [`poster.pdf`](poster.pdf) | 作品海报 |
| [`src/`](src) | Verilog RTL（15 个源文件） |
| [`embedded_sw/`](embedded_sw) | RISC-V 软件（`main.c` 4319 行） |
| [`docs/`](docs) | **演示文稿、讲稿、界面图、板卡照片，附中文说明** |
| [`sapphire_soc.xml`](sapphire_soc.xml) | Efinity 工程 |

**材料怎么看**：先读 [`docs/README.md`](docs/README.md)，里面逐项说明了每份材料的用途、页序、关键数字和出处。

---

## 硬件平台

| 项 | 内容 |
|---|---|
| FPGA | 易灵思 Trion **T20F256C4** |
| 系统时钟 | 100 MHz（片内 PLL，外部 25 MHz 晶振） |
| 软核 | **VexRiscv RV32IMC**，由 Efinity Sapphire SoC 生成 |
| 模数转换 | **LTC2315**（12 位，1.5152 MSPS） |
| 数模转换 | **DAC7811**（12 位，增益控制） |
| 可变增益 | **AD8336**（VGA） |
| 前置放大 | **AD8421**（约 10×） |
| 显示 | 串口屏（800×480，UART 115200） |

---

## 目录结构

```
src/                    Verilog RTL
├── State_machine.v     主控状态机（严格三段式，八态）
├── PWM_gen.v           200 kHz 激励生成
├── Delay_gen.v         接收窗口延时
├── LTC2315_Ctrl.v      ADC SPI 时序
├── DAC7811_Ctrl.v      DAC 移位时序
├── cfu_proc.v          互相关硬件加速（SMAC4）
├── Frame_fifo.v        双帧缓存
├── apb_periph.v        APB 外设总线
├── tjc_screen.v        串口屏组帧
└── sapphire_soc.v      顶层

embedded_sw/            RISC-V 软件
└── EfxSapphireSoc/
    ├── bsp/            板级支持包
    └── software/standalone/bringup/
        └── src/main.c  主程序（4319 行）

sapphire_soc.xml        Efinity 工程
sapphire_soc.peri.xml   引脚约束
sapphire_soc.pt.sdc     时序约束
```

---

## 工作原理

### 一帧的流程

```
IDLE → FIRE → WAIT → ARM → ACQ → DRAIN → GAP → IDLE
```

| 状态 | 动作 | 退出条件 |
|---|---|---|
| FIRE | 发 6 个 200 kHz 周期 | `pwm_ack` |
| WAIT | 等回波飞到（声程 81.6 mm） | `delay1_ack` |
| ARM | FIFO 预置 | `fifo_arm_ack` |
| ACQ | 采 256 点 | `fifo_full` |
| DRAIN | 搬出数据 | `read_all_done` |

**每个状态只等一个 ack**，抖动上限 1 个时钟周期 = 10 ns。

任一状态超时（`MEAS_TIMEOUT = 20 000 000`，200 ms）即进 `S_HALT`，PWM / ADC / DAC 全部关闭。

### 互相关

- 采样率 **1.5152 MSPS**，每点 **660 ns**
- 每周期采样点数 **7.5758**（200 kHz 载波）
- 滞后范围 **±30**（61 点）
- 61 个乘加并行，由 `SMAC4` 指令驱动，DSP 36 块全部用满
- 三点余弦插值细化到 **1/16 采样点 = 41 ns**

### 风速换算

```c
v = c² · dTOF / L
```

`L = 81.6 mm`，`c` 随温度修正：`c = 331.3 + 0.606·T`（m/s）。

单轴对射，风沿声路，**无角度修正项**。

---

## 关键数据

| 指标 | 数值 | 来源 |
|---|---|---|
| 时延分辨力 | 41.25 ns | 1/16 采样点 |
| 风速分辨力 | 0.08 m/s | 按 41 ns 折算 |
| 帧时长 | 390 µs | 板上实测 |
| DSP 占用 | 36 / 36 | Efinity 报告 |
| EBR 占用 | 182 / 196 | Efinity 报告 |
| WNS | +0.211 ns | 100 MHz 约束 |
| Fmax | 102.155 MHz | seed 1 |

---

## 构建

**固件**（需 RISC-V 工具链）
```bash
python docs/_build_fw_py.py
```

**比特流**（需 Efinity 2026.1）
```bash
python docs/build_soc.py --map --pnr --seeds 1,2,3
```

**烧写**（必须用 Efinity 自带 Python 3.11）
```bash
set PYTHONHOME=D:/efinity/python311
set PYTHONPATH=
python <efinity>/bin/program_flash.py ...
```

---

## 说明

- `src/EfxSapphireSoc.v` 为 Efinity 自动生成的 SoC 顶层，请勿手工修改
- `src/*.bin` 为片上 RAM 初始化文件，随固件重新编译更新
- 风速换算中的 `g_l_mm` 为实测声程，改动后需重新标定零点
