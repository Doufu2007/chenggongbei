# UAV 水平与上下包络最小修复

2026-11-12。此修改只处理完整 mission 的配置与最终运动几何门；没有修改飞控控制律、控制速度、单点/route/worker，没有打包、上传、启动仿真或发布任何消息。公开候选只保存在 `docs/candidates/uav_body_envelope_20261112.json`，根字段 `verified=false`。默认 `config/mission_plan.json` 的未知坐标、尺寸与 `interface_profile.json` 的未确认状态均未填补。

## 根因与配置合同

旧窗口校验使用 `robot_radius_m+clearance_m` 同时表示水平半径和原点上下距离。公开候选的完整旋翼水平半径约 .447m，而窗洞只有 .75m高，因此旧球模型无论选择什么高度都拒绝该洞。只改配置校验仍不够：旧点云圆柱与FOV包球也会把 .447m重新用作竖直余量。

本修复保留 `robot_radius_m` 为level水平外包络；新增完整可选字段：

```json
"uav_body_envelope": {
  "below_origin_m": 0.068,
  "above_origin_m": 0.166,
  "max_tilt_rad": 0.05
}
```

这是来源明确的候选数值，不是生产默认值。extents相对用于控制的机体/里程计原点，不是总机高、传感器高度或世界z。缺少整个字段时，配置和运行均保留旧保守球语义；出现 `null`、部分字段、额外字段、非有限/负extent或tilt超出[0,.15]直接拒绝。没有为未知字段赋0，没有缩小 .447m 半径。`clearance_m`继续显式提供。

显式包络要求每个窗口有有限正 `width_m`，表示穿越中心线处、垂直于其水平行进方向的真实净宽；路线必须以洞中心为中心线。校验同时保留上下位置误差与横向控制容差，不允许仅配置通过、却按既有容差推进时碰窗：

```text
effective_R = R + max(up,down) sin(max_tilt) + clearance
effective_D = down + R sin(max_tilt) + clearance
effective_U = up   + R sin(max_tilt) + clearance
bottom + effective_D <= crossing_z - window_z_tolerance
crossing_z + window_z_tolerance <= top - effective_U
2 (effective_R + position_tolerance) <= width_m
distance(approach.xy, exit.xy) > 2 (effective_R + position_tolerance)
```

stage的下降/穿越仍使用已有 `window_z_tolerance_m`，没有改变控制器容差默认。旧无显式包络的plan保持原几何合同。

## 公开来源和实体边界

来源为[固定公开 world](https://github.com/ranmengjia/ICAN/blob/82b041043849a5d86e085390bf0df787366d98cd/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world)，本地路径 `external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/worlds/inspection_task1.world`；SHA256：`595cbb95a395a2ca8923b79a0a5d3523e9198711059307d8b8cde6d47f35c9c1`。独立只读审读也得到同一哈希与边界。

`amphibious_drone` 在world第197行，model世界初始z=.055（199行）；`base_footprint`本身无额外pose（200行）。以下均相对该原点，保留SDF实际 `1.5708`，没有替成精确π/2：

| 几何 | world行 | 相对原点z范围（m） | 说明 |
| --- | --- | --- | --- |
| base collision/visual | 205–206 | [-.045,.045] | box .34×.22×.09 |
| flight body / arm visuals | 208–210 | [.0675,.1225] / [.1275,.1525] | 只有visual |
| 四rotor visuals | 211–214 | [.154,.166] | radius=.065，center=(±.27,±.27,.16) |
| emitter visual | 216 | [-.068,-.042] | 全可见包络最低实体 |
| camera body visual | 217 | [.104999926536,.155000073464] | 已计实际pitch=1.5708 |
| camera lens visual | 218 | [.115999988980,.144000011020] | 低于rotor最高边 |
| 四轮collision/tire | 339–344等 | [-.055000064281,.035000064281] | 已计实际roll=1.5708 |

全collision+visual的level候选z包络是[-.068,.166]；水平半径由rotor给出 `hypot(.27,.27)+.065=.446837661841`，继续向外取 .447。rotor、flight body、arms、emitter、camera都只有visual，Gazebo collision仅base+四轮；不能把 .447描述成Gazebo已有旋翼collision，也不能拿base_collision单独缩小机身。sensor optical原点和下视range原点不是实体几何边界。

窗口碰撞box在139–146、154–161行：bottom=.4、top=1.15、width=1.0、厚度=.16。仅属公开静态候选，未证明等于当前平台实例。

## 姿态和运行门

任意总倾角α≤.05rad，level圆柱可保守扩为：水平半径≤`.447+.166 sin(.05)=.455296542099`，向下≤`.068+.447 sin(.05)=.090340688664`，向上≤`.166+.447 sin(.05)=.188340688664`。三项都向外膨胀，没有只扩z却继续声称 .447覆盖所有姿态。

计clearance=.03后，水平R=.485296542099，下D=.120340688664，上U=.218340688664。计window_z_tolerance=.04后，允许crossing_z区间为[.560340688664,.891659311336]，.75在其内。**1m窗的position_tolerance最多 .014703457901m，所以默认 .08m必须拒绝。** 专项显式用 .01m验证可容纳的配置；它不是平台已能达到该精度的证据，不修改控制默认。

同步链路如下：

- `MissionController`只生成一次不可变的有效 `UAVEnvelope`，窗口与横向容差校验共用。
- `load_configuration`与实际node构造都把同一包络传入`RuntimeInputs`，node最终门还检查包络一致。
- `motion_path_clear`使用膨胀后水平R计算停止距离与横向corridor；ground与旧无包络路径保持原语义。
- UAV姿态门使用`min(profile.max_planar_tilt_rad, envelope.max_tilt_rad)`；越界会清除旧Pose2D与原始odom证据，不能沿用旧姿态。
- 显式点云使用完整四元数将body点变换到yaw相同的水平坐标，按独立上下边界及完整水平radius检查；不是丢掉roll/pitch。raw odom仍原样保留。
- 前视点云的停止端面使用四角分别验证横/竖FOV与range，不再用水平半径包成竖直球；使用`z/x`凸锥是球面elevation锥的保守子集。前视垂直、后退、yaw、超FOV、少于3个有效远样本仍拒绝。可选body点云也走独立上下界。
- 显式包络路径要求当前点云stamp与所用odom stamp相同；没有确认的pose/cloud skew误差上限时不允许近似。stamp不相同、旧数据或缺raw姿态均拒绝；没有改变旧路径的时间合同。

## 红绿与快照

先运行新专项，旧源码出现19失败/2通过，首个失败即 .447水平半径被当垂直余量；日志 `runs/review/uav_envelope_20261112/red.txt`。水平宽度/容差检查另先出现6失败/26通过，见`red_width.txt`。另外验证过接近水平半径的side-separation回归会先失败，再补为膨胀半径加实际容差。

最终命令：

```text
python -m pytest tests/test_uav_envelope.py tests/test_mission.py tests/test_runtime.py tests/test_mission_node.py tests/test_perception.py tests/test_pointcloud_profile.py tests/test_official_adapter.py tests/test_protocol.py tests/test_config.py tests/test_preflight.py tests/test_geometry_runtime.py -q --tb=short
```

最终 **296 passed in 8.06s**（含34个包络专项与配置/preflight/原始姿态证据回归），见`runs/review/uav_envelope_20261112/green_final.txt`。没有运行整个无关套件。新专项覆盖完整上下包络、未知/部分字段、旧保守拒绝、上下触界、window_z误差两端、水平半径不可缩水、1m窗默认容差拒绝、FOV、倾角越界清证据、倾斜点云去倾斜、stamp不一致、前视盲方向及prelaunch/node最终门。

| 文件（src下均为inspection_mission/inspection_mission） | 修复后SHA256 |
| --- | --- |
| `envelope.py` | `4c2c18fac681a134cacd5ba4337f941c62b2b3667f3af291467c738674746839` |
| `mission.py` | `f6142ef086ed8da35f120c3cbfb1be74304672df3e3adfa1088c9ec379c47822` |
| `mission_node.py` | `d63a5bd00015d823fb50975618a050e4117cd02d6b405cee8e83e9cac8b6f7b2` |
| `runtime.py` | `74069f278e2d85f5bd31846a4ad3f2b7c75056c5cdb0aaf953c3c833c21569c6` |
| `perception.py` | `58e32b3226f9711589b83e538d93699f66341d89732eb17ec3b4dbeb8ac2eb63` |
| `tests/test_uav_envelope.py` | `1a99e35de0b00b492ebeda3ee65c4c55124d134559b50d933ace971d7c0ecbbc` |
| `docs/candidates/uav_body_envelope_20261112.json` | `e1a9be19bb443d6b8d0d89f1ab86e19c5e2224534ef3f224531fa3ba8f237d31` |

## 尚未平台验证的边界

本修改没有平台飞行/穿窗实测，也没有确认当前平台的机体尺寸、odom控制原点、允许tilt、可达 .01m控制精度、点云/姿态同stamp、传感器FOV和range等合同。公开world只支撑工程候选；unit/preflight验证的是拒绝与数据流，不是接口实测。前视点云仍不能证明垂直或背向通道，点云样本也不等于完整自由空间。没有绕过任何未知合同，更未将候选profile设为verified。下一步须先确认这些输入合同及真实响应，才能把候选几何用于平台任务。
