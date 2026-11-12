# v14 短直行后的候选地图/激光复核

日期：2026-11-12。本次仅离线读取本队v14公开`/odom`、`/scan`回传和固定公开候选world，不访问Gazebo真值、裁判文件、隐藏贴图或他队任务代码，不改任何控制源码。

## 结果及推进意义

**约2cm向前移动后，首末两帧激光仍与固定候选结构相容。** 两帧共720束均为有限有效观测，全部在预先保留的1mm容差内，最大绝对距离残差约3.475µm。没有拟合地图变换、雷达外参或选择更有利的参数。这支持继续使用同一静态几何假设设计下一层有限路线实验，当前“缺官方ZIP”不阻止以真实传感器核验后继续推进。

这次只移动约2cm、偏航基本不变，首末数据处于等待及停车后的静稳阶段；不能据此声称所有外参已独立标定、全场全部结构已确认、米级路径或空中三维扫掠安全，更不代表地面识别或官方任务计分成功。

## 来源与复现

| 来源 | 字节数 | SHA256 |
| --- | ---: | --- |
| `runs/evidence/20261112/callback_short_drive_v14.jsonl` | 210759 | `3929b4de07037746136e621517bf2060bbd2ab760116df21e0f737e3fc3078bf` |
| `external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world` | 27189 | `595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1` |

结果保存在`runs/analysis/20261112_short_drive_v14/scan_candidate_geometry.json`，包含完整原始pose、scan原header、每束观测/预测/残差/命中对象、替代假设残差及来源哈希。raw没有修改，mode保留为`ground_short_drive`，不是重新标记成旧诊断。

```powershell
python tools/validate_ground_scan_geometry.py runs/evidence/20261112/callback_short_drive_v14.jsonl --world external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world --expected-mode ground_short_drive --output runs/analysis/20261112_short_drive_v14/scan_candidate_geometry.json
```

固定候选公开源提交为`82b041043849a5d86e085390bf0df787366d98cd`，候选版本与当前官方版本仍不可等同。验证器只解析静态碰撞盒和所选rplidar安装位姿；任务板视觉材质不作为碰撞结构或类别答案。

## 真实配对、位移与误差

| 统计 | 首帧 | 末帧 |
| --- | ---: | ---: |
| 原odom stamp(s) | 2.960 | 10.180 |
| 原scan stamp(s) | 3.000 | 10.150 |
| scan−pose(s) | +.040 | −.030 |
| 比较束数/有效束数 | 360/360 | 360/360 |
| MAE(m) | 4.879×10⁻⁷ | 2.741×10⁻⁷ |
| RMSE(m) | 7.216×10⁻⁷ | 4.151×10⁻⁷ |
| 最大绝对残差(m) | 3.475×10⁻⁶ | 2.889×10⁻⁶ |
| 1mm以内束数 | 360 | 360 |

保留既有`.15s`端点配对上限，没有把原pose换stamp、插值或补造曝光姿态。这两帧不是逐scan同步的位姿样本。motion log显示开始动起时odom约t=4.02、停止条件触发约t=8.22；选取的首帧在动起前、末帧在停止后约2s。首末原twist线速均约10⁻⁵m/s，支持静稳端点解释，但不提供整个配对区间的严格速度误差界。

首末odom位置分别约`(.000027648,.000008336,.055000182)`与`(.020251620,.000163643,.055000070)`，两端距离约`.020224568m`。这与guard以首次动起原点计算的前向投影`.020213832m`是不同统计口径，不能互换；后者是约2.021cm短直行停止证据，前者只描述本次几何比较的采集端点。

射线实际命中了四面围墙、两个窗底座及侧柱。首/末命中计数分别包括北墙130/131、西墙114/113、南墙46/46、东墙9/9，以及窗口结构各61束。替代的x偏移+.1m假设RMSE为约.281/.311m，yaw+.05rad约.392/.393m；这些仅反驳已列替代假设，不是穷尽全局定位搜索或唯一性证明。

## 离线工具的小改动与验证

`validate_capture`新增显式`expected_mode`，默认仍为`bounded_ground_motion`，只额外支持`ground_short_drive`；CLI对应`--expected-mode`。所有回传事件若带mode，必须与显式期望一致，混入其他模式的生命周期或快照将拒绝。没有修改raw数据、复用多个运行或将short-drive改称bounded模式。

先新增5项回归，旧工具实际**5失败、11通过**；最小实现后该文件**16通过**。覆盖显式短直行支持、默认拒绝短直行、实际mode与raw字节保留、生命周期混mode拒绝、旧默认输出、未知mode拒绝及CLI运行。完整`python -m pytest -q --junitxml=dist/geometry_v14_full_suite.xml`为**917 passed in 48.98s**，JUnit于2026-11-12 02:25:09（+08:00）落盘。该结果仅覆盖当时工作区状态，不覆盖其后并行新增的approach实现；不为其他代理尚处TDD过程的新模块重复跑全套或修改其源码。

## 证据边界

- 两个端点、约2cm单方向平移的激励有限。即使从多个静态物体取得720束残差，也不能把它们当720个独立试验；地图到odom与雷达安装的组合仍可能存在不可辨识方向。
- 小于微米/微米级的数学残差只说明当前仿真观测与这份固定几何高度相容，不能当实际定位精度、物理标定误差界或安全裕度。
- 这里只测到激光平面命中的结构；相机安装、七类照片、ground_N消息语义、未扫到的高处障碍与窗洞高度另行核验。未用本次scan来宣称相机外参已独立确认。
- 当前短直行保守全环gate仍为`.447+.114+.06+.02=.641m`，起点北墙约.65m仅约9mm门禁余量；本工具和此次匹配没有放宽该阈值。米级路线需单独设计逐段扫掠门禁，不能将短直行通过等同于整条路线通过。
- 本结果的`score=null`、`task_completion=unverified`、`official_scene_version=unverified`保持原义，不生成成绩或结束标志。
