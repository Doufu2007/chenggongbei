# approach 几何与曝光关联独立只读审查

日期：2026-11-12。审查开发中 `src/inspection_mission/inspection_mission/ground_bounded_approach.py`，对照 `docs/superpowers/specs/2026-11-12-ground-observation-route-design.md`。未修改控制源码、脚本或测试；未跑完整测试套件或操作平台。

本报告覆盖的源码 SHA256：`5618ff529ed564db4bf0a0be3fd9536e5b8f64b1cd987f353a941b756dd60954`。后续修订需重新核验，不能将本报告当作新版放行。

## 重要缺口：capture 身份与像素输入

1. **非法当前图可被配上旧曝光记录并计入 capture。** 继承 recorder 对非法 stamp 记录错误后返回，保留旧 `exposures[topic]['last']`；wrapper 随后读取这个旧 record，却将本次非法 msg 与它一起保存。独立脚本按真实状态顺序复现：在 DRIVE_SETTLE 最后一步前送合法 UAV 图，step 进入 CAPTURE 后送 `nanosec=1_000_000_000` 的图，`capture_count` 从 0 变为 1，保存 stamp 仍为旧 `15.35`，错误为 `invalid_image_stamp`。必须确认本次成功生成的 record 确实属于本次 msg，不能以旧 last 补新帧。
2. **不可编码的图也可满足三帧完成条件。** CAPTURE 中依次送三张合法递增 stamp、有各自前序 UAV odom、但 `width=0` 的图，得到 `STOP/approach_observation_complete`、`capture_count=3`；随后 `_capture_image_events` 产生 0 个图像事件和三条 `invalid_capture_image_shape_or_encoding`。接受前需验证 shape、encoding、stride、bytes 合法；最终编码/回传失败仍须明确 partial，不能等价于三张可用真实图。

两项已通知实现代理与主代理。均由离线合成消息复现，不表示真实平台已出现这两种异常。

## 已核验的几何与时间合同

- 每帧 scan 从自身 stamp 之前的真实 `/odom` 历史选择 pose。专项例：历史有 11.10 和 11.16，scan=11.15 时选 11.10，未借用未来 11.16；差 `.050s` 接受，`.051s` 无 anchor 并进入 PAUSE 零速。
- 底层有效返回门槛严格大于 `.447+.114+.06+.02=.641m`。ApproachLimits 的局部锚点 `.03m` 与停车分配 `.03m` 总和不超过 `.06m`；新 scan 只刷新局部锚点。
- `global_origin/pulse_origin` 只在初始启动建立，`turn_origin` 不随 scan 更新，`drive_origin` 在转后静稳建立；逐个合法 odom 累加路径，峰值由全局永久原点计算。独立往返例：位移到 `.02m` 再回原点并重锚，峰值仍 `.02m`、累计路径 `.04m`，没有回落或清零。
- `.30m` 全程路径及原点峰值检查独立于局部 scan 锚点；STOP 后仍记录合法 odom 的路径和峰值，便于后查停车漂移。`.03m` 停车分配是待实测预算，不是已证明的物理制动界。
- 起点、转后、中段、终点记录保留真实原 scan payload/header 和各自前序 pose；中段在 odom 投影达 `.10m` 时复制最新 scan，因此正常 odom→scan 回调顺序下样例保存 pose 为 `.095m`。该配对本身真实，但不能将其写成“精确 `.10m` 位置的 scan”。主代理确认此语义不阻塞平台实测。
- UAV 图通过 `/uav/odom` 历史关联，地面图通过 `/odom` 历史关联；各自前序差不超过 `.05s`，没有互借 pose。修复上述本次 msg 身份漏洞后，仍需在真实平台核验三张递增曝光和完整图像回传。

本审查没有发现重锚重置全程预算的路径；未审查另一代理负责的完整状态、并发和墙钟预算，也未声称平台得分或任务完成。

## 修复后独立复核

实现代理修复后冻结源码为 SHA256 `4290b53179c00cf5c52f37b3f8a91adb86539c3295248e9e93aea013390086a9`。对同样两条复现路径重新只读执行：坏 stamp 不再借旧 record，capture_count=0；三张 width=0 图均被拒绝，capture_count=0，状态仍为 CAPTURE 而非 observation_complete。错误原因均明确保留。上述两项重要缺口在该冻结版本中已修复；没有修改控制源，也没有用该本地复核代替平台实测。
