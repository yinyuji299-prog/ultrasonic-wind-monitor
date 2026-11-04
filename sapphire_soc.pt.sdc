# PLL CLKOUT0 = 100 MHz (SoC / xcorr sys / sample domain)
create_clock -period 10.0000 pll_inst1_CLKOUT0
# PLL CLKOUT1 kept in peri.xml but not used as the FIFO write clock
create_clock -period 20.0000 pll_inst1_CLKOUT1
# JTAG TAP clock (Debugger default ~3 MHz, give 20 MHz slack)
create_clock -period 50.0000 jtag_inst1_TCK
set_clock_groups -asynchronous -group {pll_inst1_CLKOUT0} -group {pll_inst1_CLKOUT1} -group {jtag_inst1_TCK}

# ---------------------------------------------------------------------------
# 2026-09-17 清理：删掉了原先的
#   create_generated_clock -name clk_samp -source pll_inst1_CLKOUT0 \
#       -divide_by 2 clk_div
# 原因：这条约束引用的是名为 `clk_div` 的网，而 sapphire_soc.v 里**根本
# 没有这个网**（Iteration 7 的双时钟方案板测失败已回退，现在是
# `wire clk_samp = clk;`，line 67）。该约束在引擎里匹配不到目标被静默忽略，
# 但会误导人以为采样域跑 50 MHz。已验证：outflow/sapphire_soc.timing.rpt
# 中 clk_samp / clk_div 出现次数均为 0，即它从未生效。
#
# 现在整个设计只有 pll_inst1_CLKOUT0 一个功能时钟（clk == clk_samp == 100 MHz），
# 时钟组只剩 CLKOUT1 / TCK 需要隔离。
#
# 新增的 adc_parallel_capture 输出 adc_clk_o 是从这个时钟分频出来的，
# 频率远低于 100 MHz（DIV>=4），且对端是外部 ADC 芯片，
# 不需要（也没有对端器件可）做 fabric 时序约束。
# ---------------------------------------------------------------------------
# ===========================================================================
# 2026-10-25 22:20  补 ADC SDO 源同步约束
#
# 背景：此前 SDC 里 set_input_delay / set_output_delay 数量均为 0，
#       ad_sdo_0_i / ad_sdo_1_i 到 sh_a / sh_b 的捕获路径完全未被分析。
#       布局器不知道数据何时有效，走线长度自由发挥 → A 路在某些布局下
#       超出 20 ns 采样窗口，出现随机位错误（MSB 丢失、读数按 2048 折叠）。
#
# SCK 由 CLKOUT0(100MHz) 寄存器产生、经计数器翻转得到 25 MHz（/4）。
# LTC2315-12 图 6：SCK 上升沿与 CS↓ 重合，SDO 在 SCK↓ 后 t4(最大 9.1ns) 有效。
# 采样点取 SCK↓ 后半周期（20 ns），故留给片内布线的预算是 20 - 9.1 = 10.9 ns。
# ===========================================================================
create_generated_clock -name ad_sck -source pll_inst1_CLKOUT0 -divide_by 4 [get_ports ad_sck_0_o]
set_input_delay -clock ad_sck -max 9.100 [get_ports {ad_sdo_0_i ad_sdo_1_i}]
set_input_delay -clock ad_sck -min 1.000 [get_ports {ad_sdo_0_i ad_sdo_1_i}]

# 本设计在 SCK 翻转后**第 3 个** CLKOUT0 周期才采样（等 30 ns 给 SDO 建立），
# 故 SDO 捕获是 3 周期关系。不声明 multicycle 的话工具会按 1 周期算，得出虚低的 slack。
set_multicycle_path -setup 3 -from [get_clocks ad_sck] -to [get_clocks pll_inst1_CLKOUT0]
set_multicycle_path -hold  2 -from [get_clocks ad_sck] -to [get_clocks pll_inst1_CLKOUT0]
