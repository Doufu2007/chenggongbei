# v37 Bounded Cruise Height Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 独立固定初始静态柱升至绝对0.85m、双3s保持、下降回支撑，并有界保存完整前向云。

**Architecture:** 新guard与limits隔离冻结v31/v34；复用父层纯解析与factory和只读时序记录器。独立cloud collector只采样完整原始数据，结束且零输出后才编码。

**Tech Stack:** Python/pytest、ROS2 Humble、现有gzip/base64证据合同、Bash入口。

**Spec:** `docs/superpowers/specs/2026-11-12-bounded-cruise-height-v37-design.md`

## Global Constraints

- target绝对.85/hard绝对1.00m；90s运动、110s全局、2m路径；上升25/HOLD8/下降45/回零8s。
- .20源/arrival、.12clocklag、.04横漂、.05tilt、.15speed、.10angular、.641有限scan净距不放宽；+inf只在声明静态柱内适用。
- 冻结v31/v34、vertical/ground/telemetry字节不动；不上传、不飞窗、不发灯光/识别/分数/完成。
- cloud最多4帧、2MiB/帧、8MiB总量；完整原始bytes/header/layout/TF/priorpose，不截prefix，不拿未来pose。

## Review Focus

- 升高后+inf与低高度全有限轨迹应语义一致；NaN/-inf/近有限/非法布局仍拒绝。
- 阶段交接同tick低高度联合门、最终DDS迟返、watchdog均不能漏出非零。
- origin/路径/运动预算不得在HOLD或pause清零。
- cloud缺prior/TF或超限须记录真实原因；不得影响控制或持锁压缩大payload。
- 新schema/mode和完整分片必须可独立恢复，冻结代码哈希与启动退出码不变。

## Task 1: 完整原始点云采集（并行）

**Files:** 新增 `src/inspection_mission/inspection_mission/cruise_cloud_capture.py`、`tests/test_cruise_cloud_capture.py`。

**Interfaces:** `CruiseCloudCapture.capture(msg, *, phase, wall_s, trajectory, tf_records)`采集；`snapshot(encode=False)`仅元数据，`snapshot(encode=True)`在终止后编码完整bytes；四阶段slot为WAIT/ASCEND_CRUISE/HOLD_CRUISE/DESCEND_SUPPORT，内部可映射。

- [x] 写RED并保存：全bytes含尾哨兵、布局/上限拒绝、prior≤source≤.05/缺TF、四阶段容量和不重复占用。
- [x] 实现上述接口；metadata与错误有界，raw bytes完整复制，编码只由显式encode=True触发。
- [x] GREEN专项；记录collector回调和编码分离，不改任何父层。

## Task 2: 独立巡航高度协议与新scan语义

**Files:** 新增 `bounded_cruise_height.py`、`tests/test_bounded_cruise_height.py`。

**Interfaces:** `CruiseHeightLimits`、`CruiseHeightGuard(started_wall_s=...)`、`cruise_column_geometry(origin)`、`create_cruise_node(reporter=None,wall_clock=...,cpu_clock=...)`、`main(args=None)`。

- [x] 写RED：新limits严格固定、柱/硬顶、低高度全有限等价、+inf反例及坏值、HOLD/下降/预算/支撑/中断不恢复。
- [x] 实现独立guard，纯解析复用父层；完整scan结构与source合同保留，记录每帧+inf比例/finite最小值/起柱核验。
- [x] 工厂保留原两publisher/实际DDS最终边界与watchdog，附加独立时序及collector；终止清理后编码云并报告。
- [x] GREEN：真实factory相同输入输出/原软件回归、DDS迟返和异常零、完整分片恢复。

## Task 3: 入口、冻结与独审交付

**Files:** 新增 `scripts/run_bounded_cruise_height.sh`、`runs/review/cruise_height_v37_20261112/`、`docs/bounded_cruise_height_v37_20261112.md`。

- [x] 先写入口真退出码RED，新增只启动本队模块的入口；不启动仿真/裁判。
- [x] 运行新增专项与必要原height/vertical/watchdog回归、Bash语法和py_compile，保存RED/GREEN与冻结哈希。
- [x] 独立代理定点审查所有实际新文件，修复有证据问题；最终源码哈希与review对应。

执行方式与设计已由root明确授权。当前工作区由主代理并行使用；只写本计划列出的独立新文件，不提交/切换或复制其他代理的修改。
