# Height Runtime Timing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在冻结v31有限高度控制外增加有界入口/处理/查询时序，保留真实传感器与控制合同。

**Architecture:** 调用原factory后包装注册callback实际访问的receive与tick；原spin observer钩子复用。透明publisher代理记录DDS查询，guard evidence仅加可选诊断字段。

**Tech Stack:** Python标准库、现有ROS2 rclpy运行时、pytest合成transport。

**Spec:** `docs/superpowers/specs/2026-11-12-height-runtime-timing-design.md`

## Global Constraints

- 不改v31/父adapter/spin源码、控制门、公式、预算、QoS或锁顺序。
- monotonic墙钟与thread_time CPU；不新增等待，不采图/点云，不上传。
- trace最多2048、慢span最多32，计数不因trace溢出丢失；重叠span明示。

## Review Focus

- 实际factory注册的lambda需命中包装receive；证明spin回调和observer都存在。
- DDS异常及阻塞返回仍原样触发零；记录不能吞原异常。
- watchdog与主线程的publish上下文不能串线。
- 计时证据满容量仍有界、gzip恢复与声明相符。
- 新脚本保持源码定位与子进程真实退出码，不启动仿真/裁判。

### Task 1：独立观测适配与证据

- [x] 写 `tests/test_height_runtime_timing.py` 实际factory+spin覆盖RED并保存输出。
- [x] 新增 `src/inspection_mission/inspection_mission/height_runtime_timing.py`，实现record/包装与main。
- [x] 新增脚本与边界/容量/证据/退出码专项，运行相关有限回归。
- [x] 记录冻结SHA、基线不变、测试输出，交根代理独审。

执行决策：根代理已指派本子代理就地实现并由另一代理独审；当前目录不是Git仓库，不创建worktree或提交。测试按根代理“必要专项，不重复大测试”的要求运行，不展开整仓回归。

完成记录：新增专项10项通过；只运行原height 42项、原spin节拍1项、原watchdog1项，相关44项通过。实现与测试哈希、冻结父模块对照以及RED/GREEN输出见 `runs/review/height_runtime_timing_20261112/manifest.json`。独审由根代理另派，当前未上传。
