# 晟功杯：城市应急全域巡检

2026 年 iCAN 应急救援机器人挑战赛的 ROS 2 Humble 巡检项目，包含任务控制、视觉识别、传感器约束、诊断工具、测试、历史测评记录和材料草稿。

**当前状态：完整任务尚未通过平台验收。** v44 已获官方 **10 分**，耗时492.622秒；本队记录完成首点 `ground_1`，随后在第二点途中结束，尚未开始飞行。v45已修复末段低速与转向增益的耗时问题，并补充信号退出诊断，完整回归 **1866项通过**，22:29已上传练习平台（ID `d2b812b5-a832-49b4-940c-99a4d9cdf716`），等待真实验证。参见 [本局实测概要](runs/records/full_inspection_v44_public_summary.json) 和 [v45候选说明](docs/full_mission_v45_timing_plan_20261112.md)。

## 任务流程

| 阶段 | 任务 | 规则分值 |
| --- | --- | ---: |
| 地面 | 按本局公开抽签顺序完成三个点，返回指挥中心 | 30 |
| 窗口一 | 识别目标、下降并保持高度穿窗 | 10 |
| 空中巡检 | 识别两个目标并稳定灯光提示 | 30 |
| 窗口二 | 识别目标、下降并保持高度穿窗 | 10 |
| 返航 | 返回指挥中心并实际降落 | 10 |

本地合成场景和单元测试用于验证实现；正式任务成绩以官方平台裁判结果为准。

## 文件入口

- `src/inspection_mission/`：ROS 包、控制器、识别器、配置和图像模板。
- `scripts/run_full_inspection_v45.sh`：完整任务启动脚本；不启动平台的仿真或裁判。
- `tools/`：打包、诊断接收、日志分析、台账和材料生成工具。
- `tests/`：控制、视觉、时序、回传和完成判定测试。
- `docs/`：方案、接口核对、实现说明、失败诊断与历史进展。
- `runs/`：经过脱敏的历史测评、分析和审查记录。历史失败记录同样保留。
- `dist/`：历史源码包与公开启动脚本；原始与公开副本的 SHA256 见下述清单。
- `submission/`：技术文档草稿和开发演示。现有 MP4 为开发动画，**不是官方平台完整任务录像**。
- `external_resources/`：候选地图、参考材料、图像及来源清单；详情见 [第三方说明](THIRD_PARTY_NOTICES.md)。

完整入口：

```bash
bash scripts/run_full_inspection_v45.sh
```

依赖 ROS 2 Humble、`rclpy`、OpenCV、NumPy、PyYAML 及平台消息环境。平台已经启动仿真及裁判时，仅运行参赛程序；源码解压根目录应包含 `src/inspection_mission/`。

```bash
python -m pytest tests -q
python tools/mission_scenario.py
```

`mission_scenario.py` 使用人工地图和合成运动学模型；它不产生官方比赛成绩。v45完整本地回归 **1866 项通过**，报告在 `runs/review/full_v45_regression_20261112.xml`。首次公开归档的1811项测试和历史脚本脱敏后14项启动器检查保存在 `docs/github_snapshot_test_result.txt`。

## 公开副本与本机原始记录

为避免公开访问凭据，本仓库移除了临时回传地址、签名下载参数、账户字段、本机用户名路径及 PDF 作者元数据。传感器数值、模板、任务源码和原始成绩结论均保留。本机原件未修改。

[PUBLIC_SNAPSHOT_MANIFEST.json](PUBLIC_SNAPSHOT_MANIFEST.json) 为每个归档文件分别记录原始 SHA256、公开 SHA256 和脱敏标记。公开启动脚本通过环境变量接收诊断地址，默认关闭外部回传；其 SHA256 与历史实际上传脚本不同，不能拿公开副本冒充历史上传原件。源码 ZIP 未改动时仍保留原始哈希。

未放入 Git 的文件包括 Python 缓存、临时网页缓存、含账户界面的浏览器截图、下载的课程视频、工具 EXE 和大型可重新下载模型权重。CLIP 权重来源和 SHA256 在 `external_resources/openai_CLIP/MODEL_SOURCE.json`，Cloudflare 工具来源在 `tools/bin/cloudflared.source.json`。这些是可选探索/诊断资源，完整任务默认识别使用仓库内模板。

正式演示视频、逐项完整成绩和依据真实完整运行更新的最终文档仍需补齐。

