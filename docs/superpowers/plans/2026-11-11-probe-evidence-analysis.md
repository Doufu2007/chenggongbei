# Probe Evidence Analysis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将本地诊断 JSONL 转换为可审计摘要及原始 JPEG 文件，明确不完整、无输入和非法证据。

**Architecture:** 独立离线工具读取接收器的 `received_at/payload/source` 包装，按 provenance 汇总普通事件和正常/分片 snapshot；真实 JPEG 经严格解码验证后原字节落盘。摘要保留原始样本和未知字段，不推断控制、坐标或裁判结算。

**Tech Stack:** Python 标准库、Pillow、pytest。

**Spec:** 父任务要求：新增 `tools/analyze_probe_evidence.py`，仅本地分析；输入 JSONL、输出摘要 JSON 与真实 image_sample JPEG，缺失/非法证据不得冒充成功。

## Global Constraints

- 不改 probe、controller、接收器，不发网络。
- 原 JPEG 字节及 topic/stamp/frame/hash 必须保留；输出文件名不使用 topic 路径。
- 空/未生成输入为 pending；非法 JSON/JPEG 给出原始行号，CLI 非零退出。
- 分片不能被当成完整 snapshot；未知字段及缺失事件明确列出。

## Review Focus

- 网络回调可能到达乱序，不把接收顺序推断为 ROS 产生顺序。
- 正常 snapshot 和 metadata/section/topic/sample 分片都要保留来源行号。
- 标记正确但解码失败的 JPEG 必须拒绝，不能仅检查头尾字节。
- 重复或非有限 JSON 数据不能静默覆盖。
- 抽签原始 payload 保留，不将静态资料或 topic 名推导为坐标。

## Task 1: 独立分析器与交付说明

**Files:** `tools/analyze_probe_evidence.py`、`tests/test_analyze_probe_evidence.py`、`docs/probe_evidence_analysis.md`。

**Interfaces:** `analyze_file(input_path, output_dir) -> dict` 写入 `summary.json`，CLI `python tools/analyze_probe_evidence.py INPUT --output DIR`；pending/collected/partial 返回 0，invalid 返回 2。

- [x] 先编写完整输入、分片、坏 JSON/JPEG、空/未运行输入与未知字段的行为测试，运行确认缺工具时失败。
- [x] 实现解析、provenance/缺失事件摘要、分片合并及真实 JPEG 验证和原字节保存。
- [x] 运行专项测试、本地历史输入分析，验证图像哈希和错误行号。
- [x] 写使用说明，交父代理运行包含本工具测试的最终全套验证。
