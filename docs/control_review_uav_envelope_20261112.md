# UAV 包络补丁独立审查

2026-11-12。结论：**Approved，仅限局部几何修复。** 本次逐读了五个生产源文件、作者专项测试、候选文件和来源说明；未发现本补丁范围内需要阻塞合入的新缺陷。没有修改生产源码、运行 ROS、上传平台或访问裁判私有数据。296 项作者测试只作为既有证据，不重复计为本次独立通过数。

## 代码核对

- `envelope.py:21`：整个字段缺失保留旧球模型；完整显式字段拒绝布尔、NaN、无穷、负上下界和越界倾角。水平半径保持原值，并随允许倾角及 clearance 向外膨胀；上下分别膨胀，不用旋翼水平半径代替竖直尺寸。对 level 圆柱，水平分量上界为 `R+max(D,U)sin(a)`，竖直上下界分别不超过 `D+Rsin(a)`、`U+Rsin(a)`，三个公式保守。
- `mission.py:216`：先验证原始 `robot_radius_m/clearance_m`，再建立有效包络。窗口校验在上下分别计入 `window_z_tolerance_m`；显式 `width_m` 同时容纳水平包络和 `position_tolerance_m`。默认 .08m 横向容差对 1m 候选窗会拒绝，未被改小成默认 .01m。
- `mission_node.py:49`、配置加载和 node 构造：同一有效包络传入运行输入；最终门检查一致，stop distance/corridor 使用膨胀后的水平半径。
- `runtime.py:128`：姿态门取 profile 与包络的更严格倾角；坏姿态会清掉之前的有效 pose/raw evidence。显式点云路径要求实际当前 odom 与 cloud 同 stamp，未用到达时间替换源时间或引入未确认 skew。
- `perception.py:161`：完整归一化四元数变换到保持 yaw 的水平坐标，未丢弃 roll/pitch；cap 四角反变换到传感器后检查前视孔径、量程及近端界。前视垂直、后退和旋转未被开放。矩形锥的 `z/x` 条件是球面 elevation 孔径的保守子集。
- 候选明确标记 `verified=false`，包含 visual-only rotor/emitter 的外廓来源；未把视觉外廓冒充 Gazebo collision，未更新默认未绑定 profile。

## 独立探针

命令：

```text
python runs/review/uav_envelope_20261112/independent_probe.py
```

结果：PASS，证据为 `runs/review/uav_envelope_20261112/independent_metrics.json`。

1. 对真实旋转后的完整圆柱边界进行 **586,894 个点**核算：37 个水平倾转轴、11 个倾角、721 个圆周角、上下两圈，均落在派生包络内。此采样检验公式实现，精度范围不能外推为平台定位精度。
2. 四组组合 roll/pitch/yaw 使用独立欧拉四元数与逆刚体变换构造原传感器点：包络内障碍均拒绝，位于上包络之外的点不误挡，旋转矩阵正交误差小于 `1e-12`。
3. 四组姿态下，源 cloud stamp 与 pose 相差 **1ns** 都拒绝；没有同stamp时不会沿用之前的几何。
4. 默认 .08m 窗口横向容差拒绝；显式 .01m 候选配置可建立；布尔、无穷、NaN、负上下范围全部拒绝。

## 仍需单独解决的运行限制

**批准此局部修复不等于前视飞行已可运行。** 独立探针用真实 `motion_path_clear` 组合参数确认：候选水平有效半径约 `.485296542m`、平移请求 `.2m/s`、时效 `.5s` 时，实际 stop distance 为约 `.585296542m`。公开候选前视雷达安装在 `(0.12,0,0.18)`、竖直 FOV `1.1rad`，这个近端停止 cap 仍不能被完整孔径覆盖，最终门返回 false；将专项 stop distance 显式设为 `1.2m` 时可以放行，但这不能替代真实 tick。不能据专项较长距离用例宣称正常前飞已经打通，也不能为放行而删 FOV 门。

此外，严格同源 stamp 能否在实际订阅频率和时钟中成立、当前原点与尺寸、.01m 跟踪精度、上下净空、起降和照射都尚未平台验证。需继续用实时观测与明确静态几何假设建立受限动作合同；本审查没有改变这些限制，也没有取得新平台分数。

## 本次独立读取源 SHA256

| 文件 | SHA256 |
| --- | --- |
| `envelope.py` | `4c2c18fac681a134cacd5ba4337f941c62b2b3667f3af291467c738674746839` |
| `mission.py` | `f6142ef086ed8da35f120c3cbfb1be74304672df3e3adfa1088c9ec379c47822` |
| `mission_node.py` | `d63a5bd00015d823fb50975618a050e4117cd02d6b405cee8e83e9cac8b6f7b2` |
| `runtime.py` | `74069f278e2d85f5bd31846a4ad3f2b7c75056c5cdb0aaf953c3c833c21569c6` |
| `perception.py` | `58e32b3226f9711589b83e538d93699f66341d89732eb17ec3b4dbeb8ac2eb63` |

源哈希与作者最终 manifest 一致；后续源变化不在本次批准范围内。
