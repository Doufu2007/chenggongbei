# 完整飞行独立只读审查

审查者 ground_next_gate；2026-11-12。仅阅读本队源码、公开固定 world/launch 和本队传感器证据，未读私有裁判、隐藏贴图分配或运行真值。未改生产文件与作者测试。

## 门控结论

原源码 SHA256 `2299fa43df83970318f782b3c8511b961b83b7700a367ece1e1db612303e9a64` 上，真实 `create_flight_inspection_node` 工厂复现两处阻断：

1. 最终独立零命令 publish 耗时 .30s 后，`condition=flight_clock_stale`、odom/clock 墙龄 .30s，但仍发 `mission_completed` 并置完成。
2. 纯上升时障碍投影仅用 XY；机体 z=.90，点云传感器内点 (.20,0,.05)，世界点 (.32,0,1.13) 位于 1s 上升 .03m 的包络内；原 `path_clear=True`，真实 tick 发布 z=.04195 非零。

作者修为 `67cc2d18ac8643fe45f5c91173157c1e6c97d1b3eada1e784b54a0a5714be75c` 后，两反例均正确拒绝：最终零返回过期时不发完成；上升障碍不再发布非零。证据 `probe.py` / `metrics.json`。这次点云采用新的源 stamp，避免同 stamp 改内容被作者新门正确拒绝而掩盖扫掠验证。

作者同时修复同 stamp 刷新墙钟、落地缺 yaw 门；本人检查实现并运行 `python -m pytest tests/test_full_flight_inspection.py -q`，该 SHA **16 passed**。之后作者增加视觉拒识证据保存，需以最终接线 SHA 再运行相关专项。

## 候选几何与接口

- 用生产 `hover_for_downlight` 生成灯位后，完整 26 条名义相邻段 `static_swept_clear` 全通过；见 `static_nominal_segments.json`。这只证明静态候选几何，不证明动力学/实机路线。
- 窗口声明和降落声明编码与公开规则对应：`window_1` / `window_2` / `landing_marker`；识别话术经 `official_protocol.recognition_phrase`。该模块保留官方计分未核验标志。
- world/body 命令在真实 adapter 发布前按当前 yaw 转换；Z 使用已有实测 trim，1.282m 响应仍属候选外推。
- `/scan` 与 UAV 传感器挂在同一 `amphibious_drone/base_footprint`；不能因“地面雷达”名称认作第二台静止车。v34 原始 TF 58 对同 stamp 两原点最大差 (0,0,.0005)m，没有 .012m 固定差。
- 相机候选应为 (.390,0,.130)。公开 world 第299行与 launch static TF 配置均是 .130；v34 无相机 TF 实收记录，不能宣称已做平台标定。完整来源见 `camera_and_scan_sources.json`。

## 视觉发现与最小可行方向

在公开 a1 观察位 (3.08,-2.15,.9,pi/2)，.35×.30m 地板图、80°真实 pinhole K 投影约130×71px。每张公开参考独立合成，不使用本局分配。原 worker /真实 factory 10张均未检出。最初合成有 UV 镜像方向歧义；`uv_corrected_probe.py` 修为保持方向的透视后，原 worker **仍10张全空**，因此结论不依赖镜像输入。

离线 BEV 的原大尺寸参考仍全空。BEV 后将公开参考缩为物理比210×180或140×120，或等比最长边300/150，原 `TemplateMatcher` 的12内点/.55/四角门不改，全部10张恢复正确类；8方向共160个检出无错类。见 `bev_probe.py` / `bev_metrics.json`。这不是实景识别率，只用于确定生产修复路径。height_v34_analysis 负责独立 floor 视觉模块；本审查不修改其生产实现。

完整测试里的逐stage目标瞬移序列只验证协议顺序，不能当连续动力学、平台得分或完整飞行成功。

## 独立 floor 模块复核

`floor_plane_detector.py` SHA `c76cae66211925184bb8d921275aad7ae887275a66b63495255a5cdcd42b8cba`：完整姿态/K构造平面矩阵正确，不接受 point ID/地图目标中心；匹配中心/四角逆映射到原曝光像素，来源SHA和派生版本保留；跨类同面冲突继承原匹配器拒绝。

非作者80个合成变体59检出、0错类，检出中的最大世界定位偏差 .0072843m。与公开轴对齐板对应的0°/180°及镜像4组全40/40；90°/270°后强行把纹理重新拉伸至同一个 .35×.30m 矩形的扩展场景漏21例，详 `floor_independent_metrics.json`。按主代理收尾范围，这些额外拉伸朝向不作为本次新增阻断；不宣称任意旋转识别率100%。

门控+拒识证据最新 `full_flight_inspection.py` SHA `2a2d34bfc5fdddf2e62460a7b5fd362d9fb70d9be2fdc0ff17619d57b61cc813`：18项专项通过。`frame_response_probe.py` 通过真实factory核对yaw=pi/2的world/body逆变换一致，窗口下降trim发+.01195、悬停+.02695，灯false；不将其当动力学证明。

## 返航模式解包器最小补丁

主代理明确授权后，`tools/analyze_ground_three_point_evidence.py` 的 `MODES` 仅加入 `ground_three_point_return`；既有变体参数测试增加该模式。先跑此模式正确RED（旧解包器 `transport_complete=False`），再改模式集合，解析器16项全GREEN。未改变解压、SHA校验、来源保留或结算语义。

## 最终定点结论

完整飞行候选 `619d3dab44c0c47251d8a603946dbc84aa8f60367d1ea0b6e38b4898222209cd` 的真实factory输入图像→worker→生产localize→TargetFix（原误差 .05639512098081544m、stamp31.2）→一次识别上报→迁移→前次实际ON之后的独立反馈→照射2s→`AIR_NAV_2` 正控通过。原fix/stamp未变、21次ON、无错误。两原负控仍正确拦截。见 `factory_floor_light_metrics.json` / `metrics.json`。该正控等待worker时冻结虚拟墙钟，只证明接口与几何。

随后主代理实测发现原双尺度CPU耗时与 .5s 检测期限冲突；floor 缓存最小改为150单尺度，SHA `ce852363945a49c3056201a3ae0a66e358149851fafd909ffc3fb2a224c80748`。非作者 `async_wall_probe.py` 不await worker冻结源时间，真实wall推进5.734s、source推进5.675s后进入 `AIR_MOVE_TO_LIGHT_1`，恰一次识别上报，无stop/errors。首帧4.262s冷加载如实过期；后续warm .308/.207/.233/.192s，连续freshness门成功。详 `async_wall_metrics.json`。这补足异步时序验证，不修改 .5s 门，不保证另一台平台CPU的同等耗时。

在上述精确源码组合、现有公开静态世界/候选外参合同内，本次已列明确阻断均修复，独立定点审查允许继续完整平台实测。尚未取得本局平台完成或官方得分证据。
