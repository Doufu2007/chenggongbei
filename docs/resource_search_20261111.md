# 城市应急全域巡检公开资源核验（2026-11-11）

## 本轮结果

找到了同平台的两个公开 GitHub 工作区副本，已保存接口说明、地图、静态场景、launch/config 和七类候选参考图；没有找到可从官方公开页面直接取得的 `demo_inspection_ws.zip` 或官方百度网盘分享链接。第三方副本不能认定为当前官方版本，未下载或执行任何预编译插件、裁判程序、他队主程序、模型或测评结果。

下载文件位于 `external_resources/ranmengjia_ICAN/` 和 `external_resources/wbxxmz_ros2-car-first/`。各自的 `SOURCE_MANIFEST.json` 记录原始 URL、固定 commit、Git blob SHA、SHA256、大小和失败条目。

## 公开来源与版本

| 来源 | 固定版本 | 保存内容 | 适用范围 |
| --- | --- | --- | --- |
| https://github.com/ranmengjia/ICAN | `82b041043849a5d86e085390bf0df787366d98cd`，提交时间 2026-10-24 | 11 文件：README、接口/地图/贴图来源/Nav2说明、地图 PGM+YAML、launch、Nav2配置、world | 同平台历史候选接口与可见静态几何 |
| https://github.com/wbxxmz/ros2-car-first | `48b3dac26b6f562ff21dd321db03fd6ffc91b492`，提交时间 2026-10-17 | 12 张成功下载的图像，2 张下载失败只记录错误 | 七类目标的候选参考图库 |

GitHub 元数据中两个仓库均为 `license=null`。公开可读不等于授予再发布许可，保存的素材应先用于本地核验；正式提交前应优先替换或确认官方授权来源。`窗口目标贴图来源.md` 单独标明两张无人机原图来自 Wikimedia Commons，CC0 1.0：`Spying quadcopter.jpg`（succo）和 `Quadcopter camera drone in flight.jpg`（Josh Sorenson）。该声明尚未对所有第三方图库图片逐张外部复核。

`ranmengjia/ICAN` 的 README 明确写道：“当前用户版暂不包含裁判系统。裁判系统源码计划在比赛开始前单独发放。”因此其环境本身不能证明当前官方计分合同或随机抽签实现。

## 已保存接口合同

原文：`external_resources/ranmengjia_ICAN/binary_install/share/ican_inspection_sim/docs/接口说明.md`。

| 方向/作用 | Topic | ROS 2 类型 | 文档说明 |
| --- | --- | --- | --- |
| 地面控制 | `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x`、`linear.y`、`angular.z` |
| 地面里程计 | `/odom` | `nav_msgs/msg/Odometry` | 地面行驶里程计 |
| 地面轮速 | `/wheel_speeds` | `std_msgs/msg/Float32MultiArray` | 四轮角速度 |
| 地面雷达 | `/scan` | `sensor_msgs/msg/LaserScan` | 二维扫描 |
| 地面图像 | `/camera/image` | `sensor_msgs/msg/Image` | 前向相机 |
| 地面内参 | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| 飞行控制 | `/uav/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x/y/z`、`angular.z` |
| 飞行里程计 | `/uav/odom` | `nav_msgs/msg/Odometry` | UAV 里程计 |
| 飞行图像 | `/uav/camera/image` | `sensor_msgs/msg/Image` | UAV 前向相机 |
| 飞行内参 | `/uav/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | UAV 相机内参 |
| 飞行点云 | `/uav/lidar/points` | `sensor_msgs/msg/PointCloud2` | 前向三维点云 |
| 旧灯光开关 | `/uav/light_on` | `std_msgs/msg/Bool` | 兼容保留 |
| 旧灯光状态 | `/uav/light_state` | `std_msgs/msg/Bool` | 逻辑状态回传 |
| 激光开关 | `/uav/laser_fire` | `std_msgs/msg/Bool` | `true` 发射，`false` 关闭 |
| 激光状态 | `/uav/laser_state` | `std_msgs/msg/Bool` | 激光状态回传 |
| 下视测距 | `/uav/laser/scan` | `sensor_msgs/msg/LaserScan` | 下方单线测距 |

Nav2 文档给出的 action 是 `navigate_to_pose` / `nav2_msgs/action/NavigateToPose`，输入 `/map`、`/scan`、`/odom`，输出 `/cmd_vel`。这些是第三方环境副本的接口文本，需要在当前官方环境通过 topic/type/action 实测核对。

## Frame 与静态场景来源

`地面Nav2导航说明.md` 给出链：`map -> odom -> base_footprint -> laser_link`。保存的 `nav2_ground.launch.py` 还定义：

- `base_footprint -> laser_link`：平移 `(0.08, 0, 0.08)`，无旋转。
- `base_footprint -> camera_link`：平移 `(0.16, 0, 0.11)`，无旋转。
- `base_footprint -> uav/base_link`：零变换。
- `inspection.launch.py` 的 `uav/base_link -> uav/camera_link`：平移 `(0.390, 0, 0.130)`，无旋转；受 `camera_rviz` 条件控制。
- `uav/base_link -> uav/laser_link`：平移 `(0, 0, -0.070)`，pitch `1.5708`。

world 中传感器 frame 分别为 `laser_link`、`uav/lidar_link`、`uav/laser_link`、`camera_link`、`uav/camera_link`。底盘与飞行插件都配置 `odometry_frame=odom`；这不能单独证明两条里程计的坐标始终可直接混用，仍须核对运行时 header/TF。

地图 `inspection_task1_map.yaml`：`resolution=0.025`，`origin=[-0.9,-3.5,0]`，`negate=0`，`occupied_thresh=0.65`，`free_thresh=0.25`，PGM 已成对保存。

`inspection_task1.world` 仅静态查看，其公开可见场景有以下模型坐标；没有读取任何随机抽签或裁判答案：

| 可见静态模型 | world pose x,y,z |
| --- | --- |
| command_center | `0, 0, 0.006` |
| ground_task_point_0 | `1.125, -1.35, 0.006` |
| ground_task_point_1 | `0, -1.35, 0.006` |
| ground_task_point_2 | `0, -2.70, 0.006` |
| ground_task_point_3 | `1.125, -1.925, 0.006` |
| ground_task_point_4 | `1.125, -0.775, 0.006` |
| window_1 | `1.925, -0.3275, 0` |
| window_2 | `1.925, -2.3725, 0` |
| aerial_task_point_1 | `3.080, -0.320, 0.006` |
| aerial_task_point_2 | `3.080, -1.355, 0.006` |

副本地面模型名从 0 编号，官方规则消息从 `ground_1` 等命名；不能直接按编号推定映射。副本底盘插件的物理最大线速度是 `0.8`，而官方规则是不得超过 `0.5m/s`；实现必须使用官方规则的更低限制。world 中底盘/飞行控制的 `cmd_timeout` 都是 `0.5` 秒，说明候选控制端需要持续刷新指令，但最终值以当前运行环境为准。

## 七类参考图映射

模板目录：`external_resources/wbxxmz_ros2-car-first/src/ican_inspection_examples/ican_inspection_examples/templates/`。

| 图像文件前缀 | 视觉核验类别 | 成功文件 |
| --- | --- | --- |
| collapsed_ruins | 坍塌废墟 | `_02.jpg` |
| landslide | 滑坡塌方 | `_01.jpg`、`_02.jpg` |
| trapped_people | 被困人员 | `_01.png`、`_02.jpg` |
| damaged_vehicle | 受损车辆 | `_01.png` |
| rescue_camp | 救援营地 | `_01.png`、`_02.jpg` |
| window_bird | 飞鸟 | `_01.jpg`、`_02.jpg` |
| window_target | 救援无人机 | `_1.jpg`、`_2.jpg` |

已将每类至少一张图组成 `.firecrawl/resource-20261111-seven-class-contact-sheet.jpg` 并实际查看，类别与图像内容一致。`collapsed_ruins_01.png` 和 `damaged_vehicle_02.jpg` 下载遭遇 IncompleteRead，未落不完整文件，manifest 留有错误。图库是公开第三方候选素材，尚无证据证明覆盖官方当前随机图片全集。

## 官方公开资源检查

- Firecrawl CLI `--status` 返回未认证；本轮使用不带凭据的公开 HTTPS 只读请求，缓存名统一以 `resource-20261111-` 开头。
- 官方课程列表 `https://www.splenach.com/api/courses?page=1&pageSize=200` 返回 177 门课程，包含 14 门 ICAN 课程。
- 巡检环境课程 `43296edb-1673-4643-8922-f13a7523f892` 的既有 2026-11-11 缓存中，`documentKey`、`documentContent`、`documentFileName`、lesson `content` 均为 null。
- 新查巡检基础 `f6c7cef3-8c18-4857-b86b-72e999a4f99b`、进阶一 `9d7eec93-b2ea-4f2c-9c3e-302ae24337f4`、进阶二 `3aa133be-c2ae-4ff8-98d3-6ed116b0c1ac` 也无文档/正文附件链接。未下载这些付费课程视频。
- 已检查站点公开配置 `/api/site-config`、公开导航配置 `/api/ros-simulation/nav-config` 和 `/api/ros-sidebar-labels`，无百度网盘分享链接。
- 检查现有官方公开页面与本轮文档缓存，未出现 `pan.baidu.com` 分享地址。这个负结果只覆盖所检查页面，不能说明官方从未发布网盘。
- GitHub 精确仓库名 `demo_inspection_ws` 的既有公开搜索缓存计数为 0；泛化查询发现上述两份工作区副本。未使用他队主程序、预训练识别器、随机任务配置或测评结果。

后续最有价值的来源仍是官方百度网盘中的原始工作区与图库；应从本队已获授权的官方公告/群资料/课程资料入口取得实际分享链接，再与本次候选文件的 SHA256、topic 和场地几何比对。
