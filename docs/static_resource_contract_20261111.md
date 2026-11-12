# 静态资源与接口合同复核（2026-11-11）

本轮只核查公开文档、已下载图片和静态 world；没有下载或运行裁判系统、他队任务程序，没有读取随机任务结果，也没有浏览器登录。

## 已核实的文件来源

`external_resources/ranmengjia_ICAN/SOURCE_MANIFEST.json` 已补入下列两张由主代理下载的场地图，仍使用原固定提交 `82b041043849a5d86e085390bf0df787366d98cd`。本地文件的大小、Git blob SHA1 均与此前缓存的该提交公开 Git 树一致，不能据此证明其为当前官方工作区版本。

| 文件 | 大小B | Git blob SHA | SHA256 |
| --- | ---: | --- | --- |
| `binary_install/share/ican_inspection_sim/materials/textures/ican_ground.png` | 624849 | `ad7aae3088dd1a8a5aa913cb0cb8031d886c7017` | `4d2e07a50ad3778cfc93bb355da9c291465d62da6d41e95fa096e2e0c401ddaa` |
| `binary_install/share/ican_inspection_sim/materials/textures/ican_layout_reference.png` | 5161919 | `5d9004df564223dbbbf9ddbf9224f27181449850` | `0ed119bd3a185a4cdaff148e1499a93b4c15119ebe1baaf0d2cded3200a63604` |

对应固定来源：

- https://github.com/ranmengjia/ICAN/blob/82b041043849a5d86e085390bf0df787366d98cd/binary_install/share/ican_inspection_sim/materials/textures/ican_ground.png
- https://github.com/ranmengjia/ICAN/blob/82b041043849a5d86e085390bf0df787366d98cd/binary_install/share/ican_inspection_sim/materials/textures/ican_layout_reference.png

另补存该提交的两份公开接口文档 `docs/topic_examples.md` 和 `docs/仿真环境与接口说明.md`，也登记在 manifest；仅含传感器、控制话题和用户侧启动示例，没有裁判编号映射。manifest 现有15条记录已逐条重新计算 SHA256 和 Git blob SHA，全部匹配。

## 地面点编号：可证实与未知

可证实：

- 已实际查看两张场地图。灰色地面任务点显示 `0、1、2、3、4`，与静态 world 中 `ground_task_point_0` 至 `_4` 的空间分布一致。
- 该 world 的静态模型坐标可以读取，但 world 没有 `ground_1 -> ground_task_point_0` 一类裁判消息映射。
- 官方《裁判系统判分细则V1.0》规定从5个地面点位随机抽3个，经 `/referee/ground_tasks` 发布，如 `ground_2,ground_4,ground_1`；到点声明示例为 `ground_2`，正文另用 `ground_X` 占位符。

仍未知：

- 没有公开合同把任意 `ground_X` 映射到该候选 world 的具体 `ground_task_point_N` 或 `(x,y)`。
- 官方正文没有穷举5个ID。它出现 `ground_1`、`ground_2`、`ground_4`，没有出现 `ground_0`、`ground_3`、`ground_5`；因此“官方一定从1开始、范围为1–5”本身也不能由这份正文严格证明。
- 不能据此采用“消息编号减1”、同号对应或其他排列作为已验证映射。

工程配置已移除 `ground_1` 至 `ground_5` 的固定枚举校验。`inspection_v1` 加载器只要求恰好5个非空、唯一且匹配 `ground_[0-9]+` 的ID；这是本队配置语法，不能当作官方完整编号枚举合同。`ground_0..4` 和非连续编号均可原样加载，不排序、不重编号、不推断坐标映射。运行时仍从配置ID集合严格接收3个不重复点，未知ID拒绝。默认模板保留 `ground_1..5`，但 `note` 已明确它们是未核实占位符，须依据官方工作区或实际公开抽签ID及实测坐标回填。

核查边界：官方正文 `docs/referee_rules_v1.txt` 的 SHA256 为 `45c6eaf7c81777af2c6f47a3cd677a3c98c3f3285e20bed5297f08df36d2931e`；公开课程来源为 https://www.splenach.com/courses/129d6e75-f19a-4c97-9ec6-0853695a4c48 。还核查了候选 world、地图说明、场地图、README和两份公开接口文档。未查看任何队伍任务程序或裁判实现来推定编号。

## 识别上报消息：已确认的字段

官方表1明确将 `/inspection/detections` 定义为发布 `std_msgs/msg/String`，用于地面、窗户和空中点的识别话术上报。ROS 2 Humble 的官方消息定义只有：

```text
string data
```

ROS官方定义来源：https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/String.msg ，本轮缓存 `.firecrawl/resource-20261111-std-msgs-string.json`，Git blob SHA `82c18e5532626e45d2510a87f4e3ab791fbd0eed`。

因此能确认的传输形式是一个字符串字段 `data`，其内容按官方表5包含相应类别和阶段话术，例如：

| 阶段 | 官方表5话术示例 | 匹配关键词 |
| --- | --- | --- |
| 地面点 | `发现坍塌废墟，地面巡航任务点确认，将巡航信息发送回指挥中心。` | `发现坍塌废墟` |
| 空中点 | `发现空中巡航任务点，识别目标为坍塌废墟，执行灯光提示，将巡航信息发送回指挥中心。` | `发现空中巡航任务点`＋`坍塌废墟` |
| 窗户 | `发现窗户上方目标为救援无人机，执行下降避让，穿越窗户。` | `发现窗户上方目标为救援无人机` |

表1中的通用示例使用了陆地清障话术；巡检应以表5的对应阶段话术和关键词为直接依据，不能从这个示例扩展出额外字段。

没有查到的合同：

- 未规定 JSON 载荷，也未定义 `point_id`、`target_id`、`class_id`、`confidence`、`center`、`bbox`、`timestamp` 等结构化字段。这不等于证明平台拒绝所有其他字符串，只表示公开资料没有给出它们的接受合同。
- 地面声明由另一个 `/referee/current_target` 的 `String.data` 发送，例如 `ground_2`；窗户和降落声明由 `/referee/cruise_event` 的 `String.data` 发送，例如 `window_1`、`landing_marker`。未找到证据要求将这些ID嵌入识别话术本身。
- V1.0规则本身未给出识别回执；另查官方测评注意事项（缓存 `.firecrawl/ros-evaluation-notes.txt`）明确列出 `/referee/recognition_status` 是保留的公开进度接口，但没有给出消息类型、字段、接受值和关联ID语义。可以只读发现实际类型/schema，不能据此假定它一定是 `String` 或含 `accepted` 字段。候选环境的 `/uav/laser_state` 是激光状态回传，不能当作识别或裁判接受回执。

这些是静态文档结论；当前平台的完整ID范围、坐标映射和任何额外接受回执仍需官方工作区或公开接口说明补足。

## 只读接口探针白名单核对

| 精确话题名 | 文档给出的类型 | 合同方向（本队视角） | 依据与诊断解释 |
| --- | --- | --- | --- |
| `/inspection/detections` | `std_msgs/msg/String` | 发布 | 官方V1.0表1：地面/窗户/空中识别话术；消息仅 `data` 字符串，阶段话术以表5为准。可加入只读探针发现类型与订阅端。 |
| `/referee/ground_tasks` | `std_msgs/msg/String` | 订阅 | 官方V1.0表1及巡检流程：例如 `data="ground_2,ground_4,ground_1"`；是公开允许接收的本场三点抽签顺序，必须按收到顺序，不排序。 |
| `/referee/current_target` | `std_msgs/msg/String` | 发布 | 官方V1.0巡检行：用户进入地面点后发送 `data="ground_2"` 等到点声明；名称本身不能证明它是裁判向用户发布的目标回执。 |
| `/referee/cruise_event` | `std_msgs/msg/String` | 发布 | 官方V1.0：`window_1`、`window_2`、`landing_marker` 窗户/降落声明。可加入只读探针。 |
| `/uav/laser_fire` | `std_msgs/msg/Bool` | 发布 | 官方V1.0及候选公开示例：`data=true` 开，`false` 关。可加入只读探针。 |
| `/uav/laser_state` | `std_msgs/msg/Bool` | 订阅 | 候选公开接口文档明确为下视激光状态回传；非V1.0计分表接口，当前平台桥接未知。可仅只读探测，不可提前认定已接通。 |
| `/referee/recognition_status` | 未给出 | 公开进度接口 | 官方测评注意事项列出名称，未定义类型和载荷。保留只读类型/schema发现。 |
| `/referee/events_public` | 未给出 | 订阅 | 官方测评注意事项说明为脱敏生命周期提示；未给消息类型，不可从事件原文推断随机答案或据此推进任务状态机。 |

只读探针不发布命令，因此命令话题的 `Publisher count=0` 不代表接口不可用；应同时记录是否存在官方订阅端、其QoS和实际类型。反之，话题名出现在图中也可能只是探针自己的订阅声明。官方测评注意事项正是要求区分这两者，并用实际回调计数/频率验证接收链路。

`/referee/detections` 属于陆地清障，不是本次巡检上报话题；`/delivery/detections` 属于物资投送。候选接口文档虽提到兼容 `/uav/light_on`、`/uav/light_state`，V1.0巡检规则使用的是 `/uav/laser_fire`，不应仅因旧兼容名称改用其他命令。
