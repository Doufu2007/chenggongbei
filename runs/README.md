# 真实运行台账

`tools/run_records.py` 是纯标准库离线工具，不登录、不联网、不推进仿真、不读取浏览器或账号。此目录只保存有原始证据支持的记录；单元测试中的 `synthetic-*` 数据只存在于测试代码和临时目录，不能复制成实测台账。

## 命令

从仓库根目录执行，退出码 `0` 表示命令完成，`2` 表示输入、校验或本地文件操作失败。所有结果输出为 JSON，错误写入 stderr；命令成功不表示比赛任务成功。

```powershell
python tools/run_records.py --help
python tools/run_records.py hash dist/inspection_probe_src.zip
python tools/run_records.py hash dist/run_interface_probe_tab.sh

# 显式导入已经存在的真实探针；此命令不会改动原始证据，也不会覆盖已有输出。
New-Item -ItemType Directory -Path runs/records -Force
python tools/run_records.py import-probe docs/platform_probe_result_20261106.json --evidence-path docs/platform_probe_result_20261106.json --output runs/records/probe_20261106.json

python tools/run_records.py validate runs/records
python tools/run_records.py validate runs/records --verify-files --base-dir .
python tools/run_records.py summary runs/records
python tools/run_records.py compare runs/records --baseline baseline --candidate visual_alignment
python -m pytest tests/test_run_records.py -q
```

`validate`、`summary`、`compare` 接受一个或多个文件/目录。每个 JSON 文件可含单条记录或记录数组；目录会递归读取全部 `*.json`。规范台账放 `runs/records/`，原始响应放 `runs/evidence/`，统计输出放别的目录；不能把非台账 JSON 混入输入目录。空目录汇总返回 `task_attempts=0`、`completion_rate=null`，不会虚构完成率。

`--verify-files` 根据 `--base-dir`（默认当前工作目录）解析台账中的相对路径，核查证据文件存在，并对同时给出路径和 SHA-256 的源码包/脚本校验哈希。未知哈希可保留 `null`；此选项不把“缺少哈希”变成“已核验”。绝对路径也可使用。只检查文件，不运行它们。

## Schema v1

每条记录必须完整包含下表字段，不接受额外字段。缺失信息写 JSON `null`，不得从网页显示的“0 分”、任务状态或当前源码反推历史分数/哈希。

| 字段 | 类型与含义 |
| --- | --- |
| `schema_version` | 整数 `1` |
| `record_id` | 非空本地记录标识；缺少平台 runId 时也须唯一 |
| `runId` | 平台原始 runId，未取得时 `null`；空字符串不合法 |
| `kind` | `official_task`、`interface_probe`、`offline` |
| `evaluation_mode` | 平台任务/探针为 `practice` 或 `competition`；离线为 `null`。`official_task + practice` 表示官方平台上的练习完整任务尝试，不表示正式竞赛提交 |
| `condition_id` | 同条件配对试验标识或 `null`；同一条件的一次配对用同一标识，多次重复用不同标识 |
| `variant` | 方案名或 `null`，如已实际执行的 baseline/visual_alignment |
| `status` | 原始状态映射到 `submitted`、`queued`、`running`、`completed`、`success`、`failed`、`timeout`、`cancelled`、`unknown` |
| `full_mission_completed` | `true`/`false`/`null`；只有真实全阶段任务日志、官方裁判结果或录像支持时可填 `true`。分数或 `completed` 状态不能自动生成该字段 |
| `source`、`script` | 各为 `{"path": 本地路径或null, "sha256": 64位小写SHA256或null}`；必须对应当次实际上传的字节，不得把今天重新生成的文件哈希冒充历史版本 |
| `draw_order` | 通过允许的运行时公开观测得到的顺序字符串数组，未知为 `null`；不得从裁判私有答案、隐藏文件或真值取得 |
| `scores` | `{"items": {...}, "total": 数值或null, "manual": 数值或null}`，见下表 |
| `duration_s` | 原始耗时换算为秒，非负有限数或 `null` |
| `failure_reason` | 原始失败原因或经证据确认的归因；未知为 `null` |
| `evidence_paths` | 原始平台导出 JSON、公开用户日志、视频等本地文件路径数组；无证据时可为空，但不能认定有效结算或完成 |

自动计分满分 **90**，人工技术答辩/文档满分 **30** 单列，统计绝不把两者相加。`scores.items` 必须含所有八个键，未知逐项分数保留 `null`：

| 键 | 自动计分上限 |
| --- | ---: |
| `ground_1`、`ground_2`、`ground_3` | 各 10 |
| `window_1`、`window_2` | 各 10 |
| `aerial_1`、`aerial_2` | 各 15 |
| `landing` | 10 |

上限来源为仓库 `docs/referee_rules_v1.txt` 表 3。分数须为范围内有限数，布尔值不当数值接收；逐项全部已知时必须与已知 `total` 相等，部分已知时总分也不能超出剩余项可能范围。即使八项已知，工具仍不自动补写缺失的官方 `total`。

不要写密码、cookie、Authorization 或 access token。台账使用固定字段和本地证据路径，不接受证据 URL；日志内的账号信息应先脱敏。校验可发现常见凭据样式，不能代替人工检查原始证据是否含秘密。

## 统计口径

`task_attempts` 为所有 `official_task` 终态/未知态尝试，包含失败、空成绩和缺少 runId 的局。`submitted/queued/running` 单列 `pending_tasks`，在终态前不伪判失败。`interface_probe` 与 `offline` 排除出分母、得分和耗时，但在 `excluded_counts` 分别计数。`evaluation_modes` 单列练习/竞赛尝试数。

`valid_settled_runs` 要求 `status=completed/success`、官方自动 `total` 已知、平台 runId 已取得、有原始证据路径。有效的 **0 分**保留为 0；`null` 保持未知。`completed_runs` 还要求显式 `full_mission_completed=true`。完整任务完成率为 `completed_runs / task_attempts`；没有任务尝试时为 `null`。`unknown_completion` 单列未确定全流程完成情况的任务数。即使平台状态为 `completed`、总分为 90，未核实全流程完成仍不算已证实完成。

`minimum_valid_total` 仅取有效结算的已知自动总分。失败局的原始部分分数仍保留在记录中，但不会混入有效结算最低分。`valid_duration_s` 仅统计有效结算的已知耗时，给出 n、缺失数、最小值、中位数、P95、最大值；P95 使用 nearest-rank，小样本时可能等于最大值。失败原因包括原始失败文本和缺 runId/分数/证据、未证实全流程完成等派生原因。该项是“未证实完成的原因”统计，不能直接当成已定位的技术根因。

完全相同的 runId 副本只计一次；同 runId 的内容冲突会中止分析，必须回看原始证据并合并成一条权威记录。无 runId 时按 `record_id` 去重，不悄悄丢掉失败局。工具不能识别被人为改成不同 `record_id` 的同一缺失 runId 事件。

`compare` 只在相同 `condition_id` 下匹配 baseline/candidate，每个方案恰好一条。练习/竞赛模式不同、已知抽签顺序冲突不会配对；重复样本列 `ambiguous_conditions`，不任意选一局。失败伙伴仍保留在配对中，未知得分/耗时差为 `null`，不造零。分差和耗时差为 candidate 减 baseline。少于两个配对或少于两个有效分差时返回 `insufficient_pairs`；其余也仅为 `descriptive_only`，不推断显著性或优势。`condition_id` 应有可复核的同环境/同任务条件依据，未知抽签顺序不代表已经证明随机条件相同。

工具校验结构与本地文件/哈希，不能自动证明手工输入的分数、全阶段完成声明或条件可比性属实。平台终态、官方原始成绩、全阶段证据应由记录者逐项复核；当前已有的 2026-11-06 探针只有空成绩失败信息，不能转成官方任务成功记录。
