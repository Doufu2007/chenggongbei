# Ground Reverse Response Implementation Plan

> **For agentic workers:** 主控已指定本子代理执行独立模块；完成后交现有独立审查代理。保持 v28 冻结，不自行打包或上传。

**Goal:** 取得 yaw=+pi/2 时三次有限负 X 输入的真实响应。

**Architecture:** 新限值子类仅新增 2rad 累计旋转预算；新 guard 继承已审状态机，转向入口设置新目标。新 factory 复用已审实际 tick 并显式替换 guard，沿用原始证据与所有停止机制。

**Tech Stack:** Python、ROS2 Humble、pytest、既有遥测 receiver。

**Spec:** `docs/superpowers/specs/2026-11-12-ground-reverse-response-design.md`

## Global Constraints

冻结 direction/route/single/three/vertical；请求序列 (-.03,0)/(-.10,0)/(-.10,0)，目标 +pi/2，仅累计旋转上界改为 2rad；45s 转向、120s 全局、.18m 路径、.12m 起点半径、.010m 请求零、.04m 观测硬停、所有源龄/DDS/scan/归零机制不放宽；不发布飞行或计分。

## Review Focus

1. TURN 使用错误负目标、错误 initial_error 或在回调中被覆盖，必须在真实正向转动后才进入负 X 阶段。
2. factory 替换 guard 后订阅或 tick 仍引用旧 guard，必须通过真实 callbacks + tick 观察三个负 X 阶段。
3. 新 2rad 预算意外放大其他界，逐字段扩界必须被拒绝。
4. 第二次高档阶段未独立归零静稳或复用旧起点，必须记录独立段与逐段 source 反应。
5. 负 X 发布失败或 DDS 消失时仍运动，必须锁停、独立尝试归零且其他输出静默。

## Task 1：新模块、适配与最小启动器

文件：`src/inspection_mission/inspection_mission/ground_reverse_response.py`、`scripts/run_ground_reverse_response.sh`、`tests/test_ground_reverse_response.py`。

- [x] 写专项测试，覆盖五项 Review Focus、.010m 请求零、.04m/2rad/45s/120s 硬界及源配对；运行得到模块缺失的预期 RED。
- [x] 实现 `ReverseLimits`、`GroundReverseResponseGuard`、`create_reverse_response_node` 与 `main`；冻结父类不变。
- [x] 运行新专项、v28 方向专项及必要同步/传输关联检查；Git Bash `-n` 与 `py_compile`；不重复整仓检查。
- [x] 记录源码/脚本 SHA、RED/GREEN、实现报告，交独立审查；发现反例先 RED 再修复，主控之后负责真实练习。
