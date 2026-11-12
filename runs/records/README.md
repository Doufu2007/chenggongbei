# 2026-11-11 导入的本队历史记录

来源：`runs/evidence/20261111/evaluation_history.json`。该账号快照返回总数8、列表8条、8个不同runId；本目录逐条保存为schema v1，不生成新的运行。它们均为2026-11-05至2026-11-06的练习测评，原始 `status=failed`、`score=null`。

分类按本次整理约定保守保留：4条上传文件名为 `inspection_probe_src.zip` 的提交归 `interface_probe`，另外4条 `inspection_src.zip` 归 `official_task`。**kind只描述提交类别或当时意图，不证明脚本启动，更不证明执行过完整任务。** 其中三个任务类源码包搭配的是诊断脚本，因此这份历史分类的0/4是“保留所有任务类失败提交的保守统计”，不能作为已完成四场算法试验后的可靠性估计。

| runId | 上传源码包 | 上传脚本 | 台账类别 | 原始耗时（秒） |
| --- | --- | --- | --- | ---: |
| `run_20261105_231853_9250eb18` | inspection_src.zip | run_inspection.sh | official_task | 15.309 |
| `run_20261106_002014_40ef0cc3` | inspection_src.zip | run_probe.sh | official_task | 11.486 |
| `run_20261106_002558_9b17bc00` | inspection_src.zip | run_echo.sh | official_task | 21.105 |
| `run_20261106_004839_665ea9d0` | inspection_src.zip | run_probe_terminal.sh | official_task | 29.378 |
| `run_20261106_005323_885f0a05` | inspection_probe_src.zip | run_interface_probe_tab.sh | interface_probe | 43.072 |
| `run_20261106_013450_5c256bcb` | inspection_probe_src.zip | run_interface_probe_tab.sh | interface_probe | 43.334 |
| `run_20261106_013935_90be4399` | inspection_probe_src.zip | run_interface_probe.sh | interface_probe | 41.156 |
| `run_20261106_031856_3b37ac97` | inspection_probe_src.zip | run_interface_probe.sh | interface_probe | 41.166 |

平台原始提交 `note` 保留在证据JSON中，按runId定位；最新三条原备注分别说明“只读接口探针”“前台运行”和“只读启动诊断v3”，其余五条备注为空。提交备注是作者对提交目的的描述，不能替代用户编译/启动日志、传感器回调、机器人运动或裁判接受证据。

所有 `full_mission_completed`、逐项分数、总分、人工分、抽签顺序、condition_id/variant及历史source/script SHA-256均为 `null`。没有下载历史上传包，未用当前重新生成的文件冒充旧版本；`source.path`、`script.path`也保持 `null`。原始上传文件名见上表，带签名的下载URL不复制到台账。

```powershell
python tools/run_records.py validate runs/records --verify-files --base-dir .
python tools/run_records.py summary runs/records
python tools/run_records.py compare runs/records --baseline baseline --candidate visual_alignment
```

导入后结果：8条唯一记录，4条任务类失败提交、0条有效结算、0条证实完整完成，4条探针排除，未知全流程完成数4。完成率0/4，最低有效总分为 `null`，有效耗时样本n=0；原始失败原因均为“流程未检测到完整结束标志（未提取到成绩）”。所有condition_id/variant未知，配对数0、状态 `insufficient_pairs`。

2026-11-11同批历史/详情快照还返回 `serverDown=true`、`serverStatus=down`。这是查询时的平台服务状态，不能回推2026-11-06失败根因。审核回复说明测评构建/运行日志不展示，也不能据“无用户日志”判断脚本没运行或平台故障。

## 2026-11-12 当前增量

除上述8条历史外，目录另有本队v4、v5只读探针，当前共10条唯一记录、6条探针排除。v5 `run_20261111_222348_ef5c04b0` 已按真实终态更新为 `failed`、平台耗时112.172秒，所有成绩及 `full_mission_completed` 继续为null；加入原始回调、分析摘要、实际JPEG和终态证据路径，并保留上传文件原始SHA256。

v5的 `draw_order` 来自本轮允许读取的 `/referee/ground_tasks`：`ground_1,ground_4,ground_0`。它是公开任务顺序，不是图片类别答案，不构成全五点集合或ID到坐标的映射。两张JPEG是相机诊断缩小图，不是正式录像或得分证据。

本轮 `python tools/run_records.py validate runs/records --verify-files` 通过：10条、0重复、文件及已有哈希匹配。汇总仍为0条有效结算、0条证实完整完成、4条任务类提交完成情况未知；探针不纳入任务完成率，不能因exit0改变该口径。详情见 `docs/platform_probe_v5_20261112.md`。
