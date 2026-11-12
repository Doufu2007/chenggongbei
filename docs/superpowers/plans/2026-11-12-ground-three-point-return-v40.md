# Ground Three Point Return v40 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 三地面点之后，在同一guard和永久账本内真实返航并双钟静稳，仍不飞行。

**Architecture:** 新增纯候选返航planner和独立guard；guard继承冻结v38，并通过冻结v36已受检的guard_factory构造。新增独立启动器，不修改父源码或setup.py。

**Tech Stack:** Python dataclasses、现有ROS适配、pytest、Bash启动器。

**Spec:** `docs/ground_return_flight_next_gate_20261112.md`；root本轮批准15m/40rad/1800s、turn1000s/drive650s及待平台验证边界。

## Global Constraints

- 新总cap15m/40rad/1800s，turn累计1000s/drive累计650s；其余冻结门不扩大。
- 同guard、session、初始pose、path/angle、总时间、恢复次数和累计暂停永久保持。
- 实际pose未通过原`.641m`候选门即拒绝；60种名义通过不是连续误差保证。
- return阶段不识别、不声明、不发布非零UAV；终点xy≤.02、yaw≤.025，源/墙双1s停稳。
- 不改v38/v36/父层源码；不操作浏览器或平台；源码只代表待验证能力。

## Review Focus

- 第三点声明前或重复handoff不得启动/重锚返航；Task2验证。
- return operations耗尽不能落入父类DECLARE_WAIT；Task2验证。
- 返航暂停、迟到DDS查询不得放行陈旧非零；Task2实际factory验证。
- 缺真实末pose、净空不足或预算不足必须停止且保留已归档三点；Task1/2验证。
- 零输出后才能记录本地返航完成；工厂发布失败不能成为成功；Task2验证。

---

### Task 1: 纯候选返航与独立预算

**Files:** Create `src/inspection_mission/inspection_mission/ground_return_plan.py`; Test `tests/test_ground_three_point_return.py`。

**Interfaces:** `plan_ground_return(actual_pose)->GroundReturnPlan`，含真实起点、home、operation、path/rotation/min-clearance；复用原`_AxisBuilder`。

- [x] 写失败测试：60种名义最大13.99/37.752432539827，ground_2南偏20mm拒绝，原点附近先在(.6,-.2)转朝南再倒向北。
- [x] `python -m pytest tests/test_ground_three_point_return.py -q`，观察缺功能失败。
- [x] 最小实现纯planner；不读私有配置、不增加ROS。
- [x] 重跑Task1测试，期望通过。

### Task 2: 同账本返航guard、受检factory和启动器

**Files:** Create `src/inspection_mission/inspection_mission/ground_three_point_return.py`, `scripts/run_ground_three_point_return.sh`; Extend本轮新test文件。

**Interfaces:** `ReturnThreePointLimits(AxisThreePointLimits)`、`GroundThreePointReturnGuard(GroundThreePointRecoveryBudgetGuard)`、`create_return_three_point_node(..., route_limits=None, worker=None)`、`main(args=None)`。

- [x] 写失败测试：新预算拒绝越界；第三点声明后仅归档一次并加载return；账本不重置；返航拒绝新视觉/声明；终点/双钟/坏帧门；真实factory使用新guard且UAV静默；DDS延迟、预算和发布失败保持零；启动器真退出码。
- [x] 执行并观察测试因功能缺失失败。
- [x] 新guard仅覆盖第三点handoff、return完成分支、return驻留、视觉入口和追加证据；其余原门保持。发布返回前不置完成；factory包装原零发布后确认本地完成。
- [x] 运行专项与必要父关联检查，期望全过，不重复项目全套。
- [x] 输出源码哈希、冻结依赖复核和实现报告，由root安排非作者审查。

## Execution ledger

- Pre-flight: Task2消费Task1的GroundReturnPlan，operation类型与冻结轴向guard一致，无冲突。
- Ruling: 目录不是git仓库，不能执行skill模板的git提交；使用文件SHA与冻结依赖快照供审查。
- Ruling: root已批准设计并明确当前代理直接实施，继续执行，无需再次请求计划确认。
- Ruling: 按root“只写必要边界/关联检查，不重复全套”限制验证范围；记录确切命令及结果。
- Task1: complete；3个缺planner断言先失败，最小实现后3/3通过；见task1_red.txt/task1_green.txt。
- Task2: complete；新增15项先失败（缺guard/launcher），实现后发现并按既有Axis上限修复单腿target_distance_m应为3而非默认.20；原父字段不变。正向整三点+返航与factory成功零发布均通过。
- Ruling: g0→g3→g2的实际合成累计转角27.766924321，加从真实末pose的返航12.542065101和原.10储备超过40rad；保留预算拒绝，不用名义角覆盖实际账本；另用g4→g3→g0证明正向完整合成反馈。
- Task2: 零发布迟返的freshness拒绝先出现RETURN_SETTLE/ready未清断言失败；将实时证据检查前置到attempt年龄拒绝前，进入原PAUSE，完成RED→GREEN。
- Verification: 22项v40专项+63项必要父关联，共85 passed in 50.43s；完整输出necessary_checks.txt。10个冻结依赖哈希逐字节一致。非作者审查由root安排。
