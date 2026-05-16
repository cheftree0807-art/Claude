# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 语言

始终使用简体中文与用户交流。代码注释、提交信息、文档均使用中文。

## 项目概述

Vivado 2018.3 FPGA 工程，目标器件 Xilinx Zynq-7000。实现基于 AXI4-Stream 的流式直方图均衡化，处理 8 位灰度图像（1280×720）。

## 架构

```
PS7 (ARM) → AXI Interconnect → AXI DMA → histogram_eq_top (AXI4-Stream) → DMA → DDR
```

- **用户 RTL**：`HE.srcs/sources_1/new/histogram_eq_top.v` — 自定义直方图均衡化 IP
- **Block Design**：`HE.srcs/sources_1/bd/HE/HE.bd` — Vivado IP Integrator 设计，连接 PS7、DMA、互联、复位和自定义 IP
- **MATLAB 参考**：`Histogram_Equalization.m` — 纯 MATLAB 算法实现，用于验证
- **测试图像生成**：`generate_bmp.m` — 生成 1280×720 渐变灰度 BMP

### 算法流水线（`histogram_eq_top.v`）

7 状态 FSM：`S_IDLE` → `S_HIST_RD` → `S_HIST_WR` → `S_CDF` → `S_MAP` → `S_MAP_CALC` → `S_PROC`

1. **直方图统计**：第一遍扫描，统计每个灰度级（0–255）的像素数
2. **CDF 计算**：直方图累加求和，每周期一个灰度级
3. **映射计算**：`(CDF[灰度] × 255) / 总像素数`，使用 DSP48 乘法 + 顺序除法器
4. **灰度映射**：第二遍扫描，查表替换每个输入像素

关键资源：BRAM 存储直方图/CDF（各 256×32 位），DSP48 用于乘法，32 位顺序除法器。

## 构建 / 开发

在 Vivado 2018.3 GUI 中打开工程：
```
vivado HE.xpr
```

Tcl 脚本（在 Vivado Tcl 控制台或通过 `vivado -source <脚本>` 运行）：

| 脚本 | 用途 |
|------|------|
| `clean_and_update.tcl` | 完整重建：重置 run、验证 BD、重新生成输出和 wrapper |
| `regenerate_bd.tcl` | IP 修改后重建 Block Design，更新编译顺序 |
| `update_bd.tcl` | 轻量更新：刷新 histogram_eq_top 模块，重新生成 BD 输出 |

修改 `histogram_eq_top.v` 后，运行 `update_bd.tcl` 或 `regenerate_bd.tcl` 在 Block Design 中刷新 IP。

## 重要文件路径

| 说明 | 路径 |
|------|------|
| 自定义 RTL（需编辑的） | `HE.srcs/sources_1/new/histogram_eq_top.v` |
| Block Design | `HE.srcs/sources_1/bd/HE/HE.bd` |
| 顶层 wrapper（自动生成） | `HE.srcs/sources_1/bd/HE/hdl/HE_wrapper.v` |
| 约束文件 | `HE.srcs/constrs_1/new/constraints.xdc` |
| 自定义 IP 封装元数据 | `HE.srcs/sources_1/bd/mref/histogram_eq_top/component.xml` |
| MATLAB 参考 | `Histogram_Equalization.m` |

## Git 注意事项

- `.gitignore` 排除了 Vivado 生成输出（`.runs/`、`.cache/`、`.hw/`、`.sim/`、`.sdk/`、`.Xil/`、日志、journal）
- `.claude/` 已加入 gitignore（Claude 本地配置）
- `HE.srcs/` 目录树（包括 IP 配置和 `ipshared/` HDL）被跟踪 — 重建工程需要这些文件
- 远程仓库：`git@github.com:cheftree0807-art/Claude.git`
