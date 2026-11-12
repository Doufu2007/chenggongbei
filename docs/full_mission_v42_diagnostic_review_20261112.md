# 完整任务 v42 启动诊断核对

当前仍未取得完整平台计分。本次只修复已经复现的诊断盲点，不能据此认定 v41 的运行根因已经解决。

## 已核对的变更

- `scripts/run_full_inspection_v42.sh`：started/exit 最多各三次、每次三秒；仅 HTTP 202 且 JSON accepted 为 true 视为投递成功。新增 ROS source 完成、源码定位完成两个启动阶段 heartbeat；诊断错误不改变真实任务退出码。
- `tools/diagnostic_receiver.py`：可选 `--delivery-log`，默认关闭；独立记录接收、拒绝、证据写入和响应发送结果。新记录不写入原 accepted 证据文件，不保存完整路径、请求内容或原异常消息；上限 4096 条、每条 1 KiB。
- 根代理只读核对了重试、错误分类、原退出码、accepted 隔离和容量上限；未发现阻断项。原 v41 源码 ZIP、脚本和 76 条冻结文件不变。

## 验证边界

启动脚本、接收器、telemetry 和总桥联合检查 **103 passed / 19.77s**，见 `runs/review/full_v42_final_combined_20261112.xml`。包含真实 Bash 和本地 HTTP 夹具，未启动 Gazebo，也不是平台任务成功。

历史 receiver 设计 RED、并行 RED 以及一项旧离线夹具构造失配均保留；旧夹具按默认关闭账本修正后，受影响项通过。最终联合结果优先于这些中间记录。

## 本轮提交文件

- 源码仍为 `dist/inspection_full_mission_v41_src.zip`，13,378,153 B，SHA256 `a2eaafc89bbfc8a185496e5df0f707f365652790d0747b548aa29cd8d9c6663d`。
- 新启动脚本 `dist/run_full_mission_v42.sh`，5,011 B，SHA256 `7299ec1761e61737e7af545e401a2c01d0da515636d6d01f795ffa96b9729617`；bash 语法检查通过。
- 完整入口保持 `python3 -u -m inspection_mission.full_inspection_runtime --ground-duration 1800 --flight-duration 900`，控制和视觉逻辑未改。

Cloudflare 新隧道申请返回 429，未拿失败地址提交。备用 SSH 反向通道已通过公网 GET 405 及本地独立投递账本核对，未在真实 accepted 证据中写入测试事件。临时地址仅在本局脚本；不发布到本报告。

**12:50 后续更新：** 上述 GET 成功仅代表 12:35 当时可达。后续 invalid POST 健康检查未到达本地接收器，SSH 进程已经退出并报 `Connection reset`。当前通道不可依赖，不能把历史健康检查当成新局回传保障。官方新局仍在服务器离线自动重试队列，尚无 runId 或实际启动证据。

## 实测要求

继续提交完整五项任务；依据本局启动、进度、退出及裁判结果定位实际断点。唯有真实平台计分可确认对应任务完成；当前有效计分基线仍为 v25 首个地面点 10 分。完整任务通过前，不制作声称完整成功的正式视频或文档。
