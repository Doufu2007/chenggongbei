# v31 Bounded Height Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在既有初始静态柱和固定硬界内测试局部高度反馈、持续HOLD与受限正trim下降。

**Architecture:** 新`HeightHoldGuard`继承冻结`VerticalResponseGuard`，独立limits允许事先批准的25s；薄ROS适配复用输入与watchdog，显式替换guard及发布接缝，保留延迟ground返回后校验。只读前向云是独立collector，不参与门控。

**Tech Stack:** Python、ROS2 Humble、pytest、现有gzip/SHA证据及Bash启动合同。

**Spec:** `docs/superpowers/specs/2026-11-12-bounded-height-hold-design.md`

## Global Constraints

- 新建`bounded_height_hold.py`、`run_bounded_height_hold.sh`、`test_bounded_height_hold.py`；v27/v29和地面源码不改。
- 严格按spec固定公式、.10/.02目标、3s驻留、HOLD8s、永久25s、总45s；原几何／运动／.20fresh门不放宽。
- 不自动重试、不发布任务或成绩声明、不前飞／穿窗；独审前不打包上传。

## Review Focus

- 3s由重复odom或越界后旧驻留拼接造成假HOLD完成：测试重复stamp和驻留复位。
- 阶段门收到NaN后被正常帧覆盖：测试阶段门非法scan／odom即时STOP。
- ground或DDS query耗时让旧命令过期：测试延迟、异常及仍独立UAV零。
- 低高度高速下降被误称阶段成功：测试高度／速度联合门，低高度未满足速度只中止归零。
- 25s被phase／PAUSE／回程重置或旧limits限制：测试跨阶段及回程预算，不修改冻结类。

### Task 1: 固定协议、反馈与证据

**Files:** Create `src/inspection_mission/inspection_mission/bounded_height_hold.py`; Test `tests/test_bounded_height_hold.py`.

**Interfaces:** `HeightHoldGuard(started_wall_s=...)`、`feedback_command(target_z,z)`、`HEIGHT_LIMITS`；guard沿旧update/step/record/evidence接口，新增phase元数据。

- [x] 写测试并运行，确认缺模块／协议而RED；覆盖固定参数、驻留、timeout、下降联合门、非法输入、永久预算、硬界及证据恢复。
- [x] 实现最小guard和固定limits，保留旧factory依赖合同；观测不能生成任务完成。
- [x] 运行专项确认GREEN，记录确切用例数（42项）。

### Task 2: 发布边界与启动合同

**Files:** Same new module; Create `scripts/run_bounded_height_hold.sh`; Test same专项。

**Interfaces:** `create_height_node(reporter=None, wall_clock=...)`；`main`，启动脚本仅source ROS及本队上传源码。

- [x] 写adapter／launcher反例，确认缺实现RED；覆盖正命令、独立零、延迟ground及DDSquery、真实命令计数／时刻和子进程真退出码。
- [x] 最小复用旧node工厂并替换guard及_publish，补固定反馈校验；创建新脚本不启动仿真或裁判。
- [x] 跑专项与必要依赖检查、py_compile、bash -n；记录任何未通过项（关联198通过，compile/bash均exit0）。

### Task 3: 只读collector与交独审

**Files:** 独立collector由root安排；新增`docs/bounded_height_hold_runtime_20261112.md`及`runs/review/bounded_height_hold_20261112/manifest.json`。

- [ ] 核对collector接口／真实topic后再挂入；若尚缺，明确主控制不依赖云、不能声称采齐。
- [x] 保存RED/GREEN、SHA及冻结源码核验，说明模型假设和平台验证仍待完成。
- [x] 交root安排独审；不上传、不打包、不声称平台成功（vertical_trim_review已接主协议独审）。

root已直接授权按此精确协议自主实现，采用同session执行；不重复请求用户审批。

## 执行裁定与进度

工作区不是git repository（`git branch --show-current`明确返回），本次按root指定共享目录创建独立新文件和review目录，不创建或移动现有工作树、不开分支、不提交git。Task1／2已完成；Task3主协议已交独审，collector待`flight_fov_diagnosis`完成axis独审后提供独立装配函数，当前未挂云且证据明确not_attached。

裁定：HOLD“位置±.005m”解释为z目标带，xy仍受原点漂移.04m硬界；等待低高度速度条件不能继续非零压低，任意近支撑高下降速度直接归零并记未完成。首次静稳按实际首次满足时刻保留（测试fixture为1.0s），不为配合旧预期延后起点。HOLD连续性必须逐odom复位；发布期间反馈变化触发永久归零而非悄悄换新命令。上述均在spec允许的保守方向，并有先RED后GREEN记录。
