# Ground Three Point Axis Runtime Implementation Plan

> **For agentic workers:** Use the parent's assigned author and a different independent reviewer. Keep frozen sources unchanged.

**Goal:** 接通已审纯轴向planner和有符号反向段，而不改冻结计分源。
**Architecture:** 新v26子类覆写三个planner入口和局部reverse语义；用bounded factory直接创建新guard，重建真实tick/coordinator/send接线。
**Tech Stack:** Python、ROS2 Humble、现有视觉worker/telemetry。
**Spec:** `docs/superpowers/specs/2026-11-12-ground-three-point-axis-runtime-design.md`

## Global Constraints

- 新固定rotation28rad；path12.5m、global1200s、turn700s/45s、drive500s；其他原门不变。
- 冻结v25/v26/planner不改；不打包上传、不读私有裁判、不宣称实际成功。

## Review Focus

- 首点/entry/handoff三处都必须调用axis planner。
- reverse body yaw保留，正distance与负projection/负request只在当前段生效。
- TURN_SETTLE不得按forward重新瞄准reverse目标；切段清flag。
- coordinator必须持有新guard，过期epoch不能进入下一点或声明。
- 累计预算、DDS失联、危险source callback后停止吸收；真实tick发布负X。
- 全部描述性graph后最终actual DDS再检查；查询返回后重算当前预算/fresh/phase；publish异常STOP并独立零attempt。

## Task 1：RED与实现

- [x] 新建`tests/test_ground_three_point_axis.py`，先核新模块缺失RED。
- [x] 新建`src/inspection_mission/inspection_mission/ground_three_point_axis.py`，实现limits/guard/adapter/main。
- [x] 覆盖上述五类接缝、完整三点合成反馈及新预算不可扩大。
- [x] 新增`scripts/run_ground_three_point_axis.sh`，仅运行本队新模块，验证实际exit和LF。

## Task 2：冻结交审

- [x] 运行axis专项、旧three-point/single-point相关检查和receiver/telemetry合同检查；不跑整仓。
- [x] 记录新旧源SHA、RED/GREEN与局限报告，交独立审查；不打包上传。

## Task 3：独审阻断修复

- [x] 原样复现独审两项RED：later graph中DDS 1→0仍负X；负X publish抛错逃逸且未独立归零。
- [x] 六项新增回归先RED，重建独立adapter后GREEN；未改冻结旧源。
- [x] 原独立probe原样重跑5/5通过；另跑真实coordinator→dispatch→counts专项。
- [x] 关联179项通过，刷新manifest和实现报告，已交原独审人复审；作者不自称最终独审通过。
