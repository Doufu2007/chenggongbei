# v42 launcher：TDD 最小修复交付

仅新增 `scripts/run_full_inspection_v42.sh`、`tests/test_full_inspection_launcher_v42.py` 与本结果目录。未改 v41 dist 冻结文件、控制或视觉模块；未构建 dist、未上传、未启动仿真。

## 实际改变

`launcher_started` / `launcher_exit` 最多尝试三次，每次 urllib 请求 timeout 为 3 s，无额外 sleep。仅 HTTP 202 且响应 JSON 对象中的 `accepted` 为布尔 `true` 才接受；读取响应最多 4097 B，超过 4096 B 拒绝。无 endpoint 写死，仍由 `INSPECTION_DIAGNOSTIC_URL` 环境供给，保留 HTTPS 或 loopback HTTP 的原有约束。

诊断异常只输出 `event / attempt / exception / status` 四字段，绝不输出 URL、路径 token、响应体或异常原文。原始解释器 stderr 被压制；内嵌 Python 仅将安全字段转到 shell stderr，解释器本身失败时输出固定 `InterpreterError`。所有诊断最终返回 shell 成功，EXIT trap 保存并返回真正任务退出码。

ROS source 完成与源码定位完成后，各发送一次合法 heartbeat（分别 `startup_stage=ros_setup_complete/source_located`），每个 heartbeat 仅一次 3 s 请求。事件仍通过现有 receiver 的 `validate_event`，无需修改接受证据 schema。最多共八次启动/退出诊断 POST。

完整任务入口仍为：

```bash
python3 -u -m inspection_mission.full_inspection_runtime --ground-duration 1800 --flight-duration 900
```

ROS_SETUP、源码根目录发现顺序、可选 install setup 与 PYTHONPATH 合同保持 v41 原逻辑。重试提高可观测性；本修复不声称已确定 v41 无回传的网络根因。

## 红绿证据

先用同一专项测试加载**原样冻结 v41 的内嵌 sender**：11 失败、1 通过、2 未选，0.93 s。失败真实复现超时仅一次、错误 HTTP/JSON 未重试、缺少阶段标记等旧行为。排除两个完整 shell 基线案例，避免执行旧冻结远程 endpoint。详见 `v42_red.txt`。

新增 v42 后：**14 项专项通过，5.40 s**，见 `v42_green.txt`。覆盖 timeout、502/503、200、202+false/数字1/缺字段、错误 JSON、202+true 成功停止、异常原文和 URL canary 不泄露、现有 receiver schema、EXIT payload。

真实 Git Bash + loopback HTTP 测试持续返回 `202 / accepted=false`：启动和退出分别实收三次，两个阶段 heartbeat 实收，完整任务实际由无害 `python3` shim 捕获参数，任务退出 0 与 37 均原样保留。没有执行 ROS 或控制程序。

第一次绿测中的两个 shell 测试夹具因 Windows `C:/...` 被加入 Unix PATH 而未命中 shim；仅将测试 PATH 经 `cygpath` 转换、夹具换行固定为 LF 后，上述真实 shell 案例通过。`v42_green_initial.txt` 保留该失败，未将其计作生产缺陷。

接收器/telemetry/总桥联合：93 通过、2 失败、3 setup 错误，19.02 s。五处全部属于同时进行的 receiver `delivery_log` TDD RED（当前被测 `create_receiver` 尚无该参数/属性）；详见 `v42_related_green.txt` 与 `manifest.json` 列出的测试名。根代理已确认并负责 receiver GREEN 后的统一必要联合，本子任务按指示不重复运行或改动该模块，**不将该联合称为全绿**。

## 冻结与 SHA256

- 新脚本：4902 B，`aa3de78b5c110b06931e214b2c549751623dc1c93268bde31fb09d128e2978c2`。
- 专项测试：8501 B，`6eed011891a657848eed93c8b9c6d98a912463d05759d8512e223124ab75368a`。
- `bash -n` 通过。
- v41 的 76 个冻结 source 条目逐项长度/SHA 与现文件一致；dist v41 脚本仍为 `7eeb6f05399bdfd82b538322065fa0fb9a2e4404ac5576ac986716a729b50309`，ZIP 仍为 `a2eaafc89bbfc8a185496e5df0f707f365652790d0747b548aa29cd8d9c6663d`。

机器记录见 `manifest.json`。最终联合与非作者审查由根代理接手。
