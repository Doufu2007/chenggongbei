# v15 有界接近观察：冻结源码独立复核

日期：2026-11-12。对照 `docs/superpowers/specs/2026-11-12-ground-observation-route-design.md` 只读审查；未修改控制源、旧 raw 或已上传产物。

冻结 SHA256：

- `src/inspection_mission/inspection_mission/ground_bounded_approach.py`：`4290b53179c00cf5c52f37b3f8a91adb86539c3295248e9e93aea013390086a9`。
- `scripts/run_ground_bounded_approach.sh`：`f7ba69a544a58cb21991655717309a38cdb29f18c6e3a0201d017ab9390c4201`。
- 解析器共享实现 `tools/analyze_ground_survey_evidence.py`：`d95e063b491c96ffa0c6da0a9ad3b2ca9c7a612eeadf44e2da3b11efa95546d4`。
- 新入口 `tools/analyze_ground_bounded_approach_evidence.py`：`d9cdca8f07669cf70c2f674d08a8a723dfe8b9c1696e0bcb472e9750fb02a429`。

**结论：可进入当前实际 360 束平台环境的有界观察练习。** 本结论不代表官方任务完成、类别识别正确、控制器接受回执或取得比赛成绩。

## 独立验证

控制 34 项与真实模块事件/解析 9 项合计 **43 passed in 5.17 s**。未重复运行全套；主代理另记录全套 967 项通过与 ZIP 逐字节核对。

真实模块事件测试直接调用本地 `GroundBoundedApproachGuard`，明确标注 `synthetic_guard_fixture_not_platform`，只写 pytest 临时目录；没有将它混入平台 run。事件均可编码并经新 mode 入口解析。三个 capture phase 有不同、递增的曝光 stamp，各自 prior UAV pose 有效；四个 stage scan 保留自己的原始 payload、header、地面 prior pose 和差值，未借用终点 pose，也未改写 mode。

## 结构边界

1. `.06 m` 仍用于局部全环阈值 `.447+.114+.06+.02=.641 m`；全局原点峰值与累计路径各 `.30 m` 为独立字段。每帧 scan 前序 pose 的移动分配 `.03 m`，另留 `.03 m` 停车空间。新扫描重锚不重置全程原点、累计路径、角度或时间。
2. TURN 到角或越过后进入零速 TURN_SETTLE，墙钟与 odom 仿真时间均静稳满 1 秒才冻结 drive_origin/drive_yaw 并进入 DRIVE。前向实际投影达到 `.20 m` 后同样先零速驻留，再 CAPTURE。
3. 全局 90 秒从创建起；TURN/SETTLE 45 秒、DRIVE/SETTLE 55 秒、CAPTURE 5 秒均保留各自首次起点，暂停也消耗预算。适配器 watchdog 的 deadline 仍为创建时刻加全局预算，不在转向变直行时重启。
4. 回调硬停与 PAUSE 共用 RLock；硬 STOP 首因锁存，无法被旧 freshness reason 恢复。路径、峰值位移、倾角、累计角及阶段相关偏航/侧向/反向越界不会被下一正常样本覆盖。
5. STOP 已先于活动阶段预算检查处理，阶段超时后的零速驻留可满 2 秒进入 FINISHED；没有为了等全局 90 秒而一直停留 STOP。FINISHED 始终只是诊断状态。
6. CAPTURE 只有在 fresh、静稳、有效先前 UAV pose、正确 frame、当前曝光身份匹配且 stamp 递增时推进；第三张合格源图后才记录观察停止。缺图、编码失败或回传不全仍是 partial，不作任务成功。

## 本轮发现并复核的修复

- 初稿接受三个不同 stamp 的空 `data` 图，capture_count=3 并记录 `approach_observation_complete`，实际 capture 原图分片为 0。冻结版先验证源 header、shape、encoding、stride 与 bytes；独立重放相同空图后 count=0、保持 CAPTURE。
- 观察器遇到坏当前帧可能保留旧 exposure。冻结版增加 stamp/frame/wall 对当前回调身份的核对，不能借旧记录推进。
- 初稿 capture encoder 在异常处理外导入 cv2/numpy，缺依赖可令 additional_events 抛出，进而使适配器尚未发送核心证据就中断。冻结版将缺依赖记录为 `capture_image_dependency`，独立模拟 cv2 缺失后仍返回 metadata、控制 series 与 image_errors；没有抛出。

## 本轮明确的传输边界

合成合法 720 束长浮点 scan 的四阶段记录加三张 capture 元数据约 66,438 B，可能超过单事件 64KB。它是合成压力例，不是当前平台触发记录。当前 v14 实际每帧为 360 ranges 与 360 intensities，单 scan JSON 约 9.2KB；本轮只按当前实际 360 束环境放行，不扩大合同或扩展传输功能。若实际新回传缺 metadata/分片，必须标 partial，不能凭 capture_count 或退出码声称证据完整。

二维扫描、候选半径、外参边界及停车余量仍是本次有界工程合同；它们不构成任意三维障碍、真实制动距离或完整赛道安全证明。正式分类、识别回执与裁判分仍须分别取得真实平台证据。

## 上传后离线解析器缺图回归

控制源与启动脚本哈希保持上述冻结值。发现离线共享解析器原来只检查已收到的图像组，整组 `capture_1` / `capture_2` 事件全失时可能误记诊断 `collected`。先用回归复现 9 处失败，再仅修改共享解析器：对 `ground_bounded_approach` 按真实 `capture_count` 输出 `expected_capture_phases` 和 `missing_capture_phases`，且必须收到完整重组的 UAV native JPEG 才消除对应缺项。count=0 不要求三图，旧 survey / short-drive 规则不变。

新旧三类观察分析专项 **51 passed in 2.93 s**。共享解析器现 SHA256 `e9c08265d26e46f2c35e055df1978291659a56a5a1b7c078be67db3f6bb081d0`；上方 `d95e...` 为上传前审查时版本，保留作历史。新 v15 raw 用修后工具另存 `runs/analysis/20261112_approach_v15_capture_check/summary.json`，原 raw 和主代理首次分析目录均未覆盖；结果仍为 partial、capture_count=0、六张普通 native，不能把它们替代三张 CAPTURE 图。
