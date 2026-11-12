# 全域巡检终态标记修复（2026-11-06）

## 证据与适用边界

最近一次平台记录 `run_20261106_031856_3b37ac97` 只有裁判启动日志，页面报“流程未检测到完整结束标志（未提取到成绩）”。这次提交的是只读诊断探针，本来就不会执行比赛任务或生成成绩，因此报错不能证明任务节点缺少完成消息是本次失败的根因。任务节点此前只有内部 JSON 心跳；收尾协议本身需要进一步核实。

平台论坛帖子声称需要向 `/inspection/mission_status` 发布 `mission_completed` 并正常退出。原帖地址：[巡检的结束条件是什么？](https://www.splenach.com/ros-simulation/forum/6999b2f6-69c7-4c53-bedf-e6a2001eefe5)。帖子的作者身份和该接口尚未得到官方文档或实际 ROS 消息证明，不能称为已确认的官方协议。官方判分细则要求在降落前声明 `landing_marker`；当前状态机保留该官方顺序，不采纳帖子中相反的声明时机。

## 修复

- 新增候选终态载荷 `mission_completed`，只在状态机完成所有地面点、窗口、空中点并通过 `LAND` 阶段后发布。该传输合同仍待平台验证。
- 将 `/inspection/mission_status` 与内部诊断心跳 `/inspection_mission/status` 分离，避免把 JSON 诊断内容误发给裁判。
- 发布终态后保留 0.5 秒传输等待时间，再关闭 ROS 节点；独立 `STEADY_TIME` 定时器使用 `time.monotonic()` 检查该等待时间，即使 `/clock` 停止也能触发退出。等待本身不能证明裁判已收到消息。失败或安全停车路径不会发布完成标记。
- 启动脚本和包结构保持平台要求：只启动参赛节点，不启动仿真或裁判。

## 验证

- 回归测试：`256 passed`
- `python -m compileall -q src tools tests`：通过
- `python tools/preflight.py .`：`PASS`
- 源码包：`dist/inspection_src.zip`；当前大小和哈希以重新构建后的 `dist/SHA256SUMS.txt` 为准。

## 尚未宣称的部分

已修复的是本队退出逻辑依赖仿真时钟的程序问题；候选终态消息是否被平台接收仍未验证。当前 `interface_profile.json` 为 `verified:false`，地图、相机、运动控制、识别确认和灯光反馈合同没有实际平台证据。因此本地测试和打包不能写成“平台测评成功”；重新上传前仍需完成真实接口绑定。
