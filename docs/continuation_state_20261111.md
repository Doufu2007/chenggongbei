# 平台测试续接状态

更新：2026-11-11 23:20 +08。本文件用于续接工作，不是完成报告。

## 用户目标与授权

继续补全全域巡检，在官方平台实际练习、逐项计分、完整运行并生成真实视频和技术PDF。可自动修改代码、下载资源、登录本队账号并发起练习测评；不购买、不创建队伍、不消耗竞赛中心正式提交次数。不得伪造比赛结果、结束标志、ACK或官方录像，不读取裁判私有答案或Gazebo运行真值。用户无法取得官方工作空间，不重复索要。

## 当前实际平台记录

- v4：run_20261111_155937_44feef8e，已终态failed/score=null。21条真实回调证实shell和rclpy启动，收到odom/TF，退出码0；只读探针没有执行比赛任务。
- v5：run_20261111_222348_ef5c04b0，详情ID a1485e1e-27a0-45b0-bd2f-d3c313fdef6a，22:23上传，22:44最近查询仍queued/up，暂无回调。
- v5证据：runs/evidence/20261111/probe_v5_submission.json、probe_v5_detail_latest.json、probe_v5_artifacts.json。两份已上传文件为dist/inspection_probe_v5_src.zip、run_interface_probe_v5.sh，不覆盖重打包。10条台账的文件哈希校验已通过。
- v5为60秒只读诊断，最终直接发汇总、独立相机JPEG；普通快照单条，超限才分片；不控制机器人，不发布识别/完成消息。

## 活跃进程（先核查，失效后再重建）

- 平台会话PTY 67835：tools/platform_session.py，已登录，凭据仅在内存。JSON命令加回车，可用save保存完整证据、limit限制显示。
- 接收器PTY 84909：本地57010，callback_v5.jsonl，已保持原提交endpoint重启并延长为21600秒，约23:19启动。旧26483已停止。隧道不变，端到端GET405自检通过，不向实际事件文件写测试事件。
- 隧道PTY 92717：cloudflared，使用dist/callback_tunnel.yml空配置避免用户默认ingress截流。临时公网URL只在v5脚本中；提交前端到端自测202，测试事件已转存callback_v5_selftest.jsonl，未混入实际日志。
- 不使用旧PTY 51413、12983、72645。

## 本轮新增与待复核

- 诊断专项108通过；最新完整检查577 passed in 40.77s，JUnit为dist/current_tests.xml。开发包inspection_src.zip=58208B，构建保持未就绪，不上传正式竞赛入口。
- tools/analyze_probe_evidence.py及16项测试：读取receiver JSONL，输出summary和真实JPEG原字节。API analyze_evidence(input_path, output_dir)。正常/分片/坏图/空输入均覆盖，缺数据不猜。
- tools/create_submission_media.py新增--probe-evidence，材料引用真实回调、样本话题、退出码和哈希，不能推断任务得分。相关24项测试通过。PDF已按577项JUnit和v4回调重建并查看5页最新渲染。
- target_geometry.py/test_target_geometry.py：纯数学定位、下视悬停、照射评估，51专项通过，未接运动控制器/修改配置。独立审查发现的倾角和漂移误差边界均已先测试后修复，并由evidence_refresh复核通过。
- 计划：docs/superpowers/plans/2026-11-11-target-geometry.md。Task2实际控制接线还未做。
- evidence_refresh在极窄范围补查官方百度网盘实际链接，未找到新链接；19图仅解出微信二维码，讨论区16帖无附件/置顶，iCAN通知API返回403未绕过。详见docs/official_resource_link_20261111.md，不重复泛搜。

## 下一步顺序

1. 本轮代码修复、独立复核、最终全套、开发包及PDF草稿已完成，不重复执行；新代码或失败出现后再跑相应检查。
2. 保持v5接收器在线，间隔查询平台直到终态；上一轮曾排队36分钟，不因排队重复上传。
3. 用分析器提取真实相机/CameraInfo/TF、scan、抽签与公开订阅端合同，查明真实控制接口；不要先设verified=true。
4. 取得并记录首个真实地面点计分，再扩展三点、穿窗、空中照射与降落。
5. target_geometry目前只是数学合同，垂直空间观测、路径门禁、相机标定/外参和任务状态接线仍需解决。前视相机+下视灯必须分阶段定位与照射。
6. 用真实完整运行更新录像和正式PDF；当前旧MP4是人工运动学演示，不能作为官方效果证据。

## 保持明确的未完成项

当前真实有效计分完整运行数为0。地图ID映射、图库版本、控制帧、识别状态data与灯光反馈尚未绑定。不能把本地测试、只读探针退出或页面状态当任务通过，也不能宣称95%完成率。

## 最新外部状态

23:08前最后成功查询为queued/up、startedAt=null、durationMs=0、hasLogs=false。随后降低频率仍连续收到HTTP429（retryAfter为1至4秒），未绕过限流，未重复提交。`probe_v5_detail_latest.json`最近已被429响应覆盖；有效排队状态及查询时刻保存在 `probe_v5_wait_snapshot.json`，完整成功历史快照在 `evaluation_history_v5_wait.json`。后退避响应在 `probe_v5_detail_after_backoff.json`。约23:20 callback_v5.jsonl仍为空，不代表程序失败。恢复查询后另存带时间的快照，避免再覆盖最后成功记录。
