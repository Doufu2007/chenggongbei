# Ground Relocated Reverse Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 在南侧实测预定位后完成三个有限负 X 请求并保存可信的输入、命令和停车证据。

**Architecture:** 新 guard 复用冻结传感器校验、prior 配对和证据字段，独立实现有界阶段状态机；新 adapter 直接用 guard factory 和最终 DDS 后预算核验。旧模块均不修改。

**Tech Stack:** Python、ROS2 Humble rclpy、Bash、pytest。

**Spec:** `docs/superpowers/specs/2026-11-12-ground-relocated-reverse-design.md`

## Global Constraints

- path `.35m` / radius `.25m` / rotation `5rad` / global `180s`，不在局部阶段重置。
- 单 turn `45s` / 总 turn `120s` / preposition drive `40s`（上传前修订）；原扫描门`.641m`及所有 fresh/DDS/局部停车门不变。
- 只 `/cmd_vel`；冻结旧源不改；上线由 root 独审后执行。

## Review Focus

- 最终 DDS 查询耗时后旧 decision 不能越过时间预算。
- 换阶段不得清空永久原点、累计路径/转角或总转向时间。
- 传感器未来配对、危险重复、非有限帧不可继续运动，首拒绝帧须完整保留。
- 预定位和脉冲均根据真实 odom 而非 nominal command 积分切换。
- publish 异常必须独立零尝试，其他发布器静默。

### Task 1：状态机与证据

**Files:** create `src/inspection_mission/inspection_mission/ground_relocated_reverse.py`; test `tests/test_ground_relocated_reverse.py`。

- [x] 写缺模块、完整阶段顺序、预算不重置、实测端点、首拒绝 scan 和门限反例；运行并保存 RED。
- [x] 实现 `RelocatedReverseLimits` / `GroundRelocatedReverseGuard`，复用 validated 输入、配对、raw records。
- [x] 跑专项 GREEN，核每阶段归零双时钟静稳与所有截止。

### Task 2：传输与启动脚本

**Files:** same module; create `scripts/run_ground_relocated_reverse.sh`。

- [x] 写 factory→callback→tick 集成和最终 DDS 查询跨时限/publish 异常反例，观察 RED。
- [x] 实现直接 factory adapter、当前时间发布前复核、独立零尝试；复制并明确重命名 launcher mode/revision/module/duration。
- [x] 跑专项和必要关联、py_compile、Git Bash `-n`；核冻结 SHA，生成实现说明，交独立审查。
