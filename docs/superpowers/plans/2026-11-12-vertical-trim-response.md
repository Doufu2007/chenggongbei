# Vertical Trim Response Implementation Plan

> **For agentic workers:** Use subagent-driven-development for the isolated diagnostic; do not modify scored or reviewed frozen modules.

**Goal:** 在低高度取得近悬停正指令与负指令的真实响应，不假定插件模式。
**Architecture:** 新guard与薄node适配层继承已审Z实验，保留累计预算与异常归零；增加有限阶段及有符号命令记录。
**Tech Stack:** ROS2 Humble / Python / 现有telemetry。
**Spec:** `docs/superpowers/specs/2026-11-12-vertical-trim-response-design.md`

## Global Constraints

严格执行spec所有上界，旧v27/地面源不改，STOP吸收，阶段与PAUSE不清预算。不上传、不读裁判私有信息。

## Review Focus

有符号Z请求的校验/发布/计数一致；无有效新反馈不推进；低高度必须退出负向阶段；归零后是否真的回到支撑面以实测为准；任何publisher故障仍独立尝试归零且不恢复。

## Task 1

新增 `bounded_vertical_trim.py`、`scripts/run_bounded_vertical_trim.sh`、`tests/test_bounded_vertical_trim.py`。

- [ ] 建立RED：有符号阶段/永久预算/低高度拒绝及Review Focus故障反例。
- [ ] 最小实现固定协议；源码中阶段配置不可在运行时放大。
- [ ] 专项与关联测试通过；记录SHA/报告与本地测试的适用边界。
- [ ] 独立审查通过后由主控另建receiver、打包并进行独立练习；本代理不上传。
