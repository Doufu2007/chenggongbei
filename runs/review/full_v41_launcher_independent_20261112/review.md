# v41 无启动回传：冻结包独立只读核验

结论：**未复现能够解释 `launcher_started` 也完全缺失的冻结脚本或 ZIP 错误。当前根因未定位，缺少远程脚本执行与回调投递证据，不能归因于地面、飞行或视觉算法。**

核验对象为记录 `d2d6ba08-ac43-42f7-a77f-a08eee68c53d`、`run_20261112_114542_50c9a8cd`。只读核查生产/提交文件；未启动仿真，未执行完整 launcher，未发网络请求，未改包、未上传。输出仅写入本审查目录。

## 冻结文件与语法

- `dist/run_full_mission_v41.sh`：2901 B，SHA256 `7eeb6f05399bdfd82b538322065fa0fb9a2e4404ac5576ac986716a729b50309`；与 manifest 和既有平台上传下载字节核验结果一致。无 BOM、CR、NUL，shebang 为 `#!/usr/bin/env bash`，Git Bash `bash -n` 退出 0。
- `dist/inspection_full_mission_v41_src.zip`：13378153 B，SHA256 `a2eaafc89bbfc8a185496e5df0f707f365652790d0747b548aa29cd8d9c6663d`；CRC 全通过，76 个条目全部逐项长度/SHA 与 manifest 一致，53 个 Python 源 AST 通过，包含预期 `src/inspection_mission/inspection_mission/full_inspection_runtime.py`，未发现越界路径条目。
- 原样提取内嵌 Python 后编译通过。仅用内存 mock HTTP opener 执行这段原样代码，`launcher_started` 与 `launcher_exit` 均构建合法 POST、Content-Type `application/json`、3 s timeout，URL 与冻结环境值相同；两事件均被精确 `tools/diagnostic_receiver.py::validate_event` 接受。该验证不接触真实 endpoint。
- v37/v38 的 launcher 函数与 EXIT trap 块和 v41 同结构，仅 mode/revision 与终端打印前缀不同。v37、v38 已有真实回传各包含 1 次 `launcher_started`、1 次 `launcher_exit`。未发现 v41 新增的 shell 引用或事件 schema 破坏。

## 因果边界

冻结脚本第 35 行调用 `diagnostic_event launcher_started`，第 37 行才取得 ROS setup 路径；其后才执行 ROS source、源码路径发现、PYTHONPATH 和完整 runtime。因此以下后段故障都**不能单独解释**已配置且可送达的启动回调也缺失：ROS setup 不存在、解包根目录不符、cv2/ROS Python 依赖失败、总桥初始化、地面返航门、空中视觉/照射或最终落地失败。

这并不证明 launcher 被执行，也不证明回调可从远程环境送达。当前 `callback_full_inspection_v41.jsonl` 不存在，而接收器只在完整合法 POST 被接受后才创建/追加文件；错误路径 404、格式拒绝、到达前失败均没有诊断 JSONL 记录。故文件缺失不能等价于“从未发起 HTTP”。

## 已复现的可观测性缺口

`diagnostic_event` 对 Python stderr 使用 `2>/dev/null`、命令使用 `|| true`，网络异常又被 `except Exception: pass` 吞掉，启动事件仅单次尝试。内存 mock 中让网络 opener 抛出 `OSError` 时，原样内嵌代码静默返回，符合该合同。该行为是已复现的诊断盲点，**不是本局网络失败已经成立的证据**。

若远程 Python 解释器在 ROS source 前不可用、DNS/TLS/出站网络失败、3 s 内未连通、接收路径不匹配或远端根本未执行脚本，都可能产生相同“无启动回传”；现有材料不足以在这些分支中选定一个。脚本显式绕过环境 HTTP 代理，与过去 launcher 相同；是否适配本次远程网络未知。

## 平台记录能说明的事实

`full_v41_detail_final.json` 报 `failed`、duration 291350 ms、score null、未检测完整结束标志；`hasLogs=true` 但 `tabLogs=[]/terminalLogs=[]`，没有可供本审查定位到脚本行或 ROS 阶段的日志。`serverStatus=down` 是该查询的状态；v37 的既有回传成功记录目前也显示 down，因此不能仅据此推断 v41 在算法执行时发生服务器崩溃。

据父任务提供的隧道记录，cloudflared 的 11:56 掉线晚于官方 11:53 更新，不能用这次较晚掉线直接解释本局全程零回传；同样，较晚才掉线也不能证明此前每个远程请求可达。

本地 `runs/records/full_inspection_v41_d2d6ba08.json` 仍为早先 queued 状态；其中 `runtime_started=false` 和“all phases attempted”备注均不能作为运行事实，最终平台详情和缺失回传应分别保留。

要进一步确定根因，需要实际 runner 的脚本调用/退出与原始 stdout/stderr，或请求投递层的可核对记录。当前只读证据没有可据以修改算法的因果依据。

机器可读核验：`metrics.json`。未变更任何冻结提交字节。
