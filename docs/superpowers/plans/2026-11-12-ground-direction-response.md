# Ground Direction Response Implementation Plan

> **For agentic workers:** Use subagent-driven-development for this isolated experiment. Keep scored source frozen.

**Goal:** 测量v26暴露的真实方向响应，给路线控制提供证据。
**Architecture:** 新guard复用既有传感器与预算门，新ROS适配器独立发布受限XY Twist；核心/gzip回传真实数据。
**Tech Stack:** Python、ROS2 Humble、现有遥测/receiver。
**Spec:** `docs/superpowers/specs/2026-11-12-ground-direction-response-design.md`

## Global Constraints

固定所有规格上界；不改冻结v25/route/三点源；不读私有裁判；不发布计分或完成。STOP吸收，预算跨阶段累计。主控裁定正常请求零阈值为实际.010m，保留.04m观测硬限及原.03m停止预留；未承诺物理停距。实际输出仅`/cmd_vel`，UAV全静默。

## Review Focus

重复stamp不得刷新活性或隐藏危险；PAUSE不得重置脉冲时间；发布异常仍尝试各通道归零；linear_y与实际命令记录一致；未知/缺失真实反馈不得推进阶段。

## Task 1：独立标定器与适配器

Files: 新增 `ground_direction_response.py`、`scripts/run_ground_direction_response.sh`、`tests/test_ground_direction_response.py`。

- [x] 先用真实guard回放v26证据建立方向不一致断言，并写规格边界/状态测试，运行得到新增模块缺失或预期行为缺失的RED。
- [x] 实现仅一次-.60rad转向和三次受限XY脉冲，逐阶段记录实际输入输出。
- [x] 覆盖Review Focus五类故障与每个运动/时间/累计硬界；运行专项与共享receiver关联检查。
- [x] 生成源码SHA、测试记录、实现报告；主控安排独立审查，不自行打包/上传。

## Task 2：平台练习与证据分析（主控执行）

- [ ] 独审通过后建立新隔离receiver，保留旧原始文件。
- [ ] 从已得分冻结v25包添加新模块，LF脚本仅运行本队代码，核SHA与ZIP完整性。
- [ ] 上传练习、保留平台ID/runId/结果与完整回传；分析真实差分响应及归零后停止，不推断官方分数。
- [ ] 根据证据制定路线修复；没有足够响应时报告具体未知，不猜测放宽门。
