# 网站测评 runner 契约与受支持诊断入口核查

核查日期：2026-11-06。范围：当前工作区已下载的网站前端 JS、测评指南缓存、官方注意事项转录和现有实测报告。只读核查，未调用提交、删除、报名、保存配置或其他外部写接口，也未访问他人记录、私有工作空间、隐藏接口或猜测下载地址。本文不包含登录凭据、用户标识或带签名的资源 URL。

## 可执行结论

1. 不需要官方示例工作空间即可完成**提交包静态诊断**：检查压缩包布局、脚本后缀、大小、禁用启动命令，并对自己的 ROS 包做本地编译/导入检查。前端的这些检查有清晰代码证据。
2. 网站已提供的运行后入口是自己的测评详情页及其终端标签。日志下载把已返回的当前标签内容生成本地 `.log`；没有发现独立获取编译日志、用户脚本 stdout/stderr 或退出码的前端请求。
3. “在线仿真”的代码执行器是浏览器中的简化 Python 风格文本解释/转换器；它不能运行上传的 `.sh`、`colcon`、`rclpy` 或 ROS2 launch。因此不能用其执行成功证明测评容器脚本可运行。
4. 现有记录只显示裁判启动日志，故仍不能区分脚本未运行、编译/加载失败、用户输出未回传这三种情况。无需继续尝试变体脚本来消耗测评次数；下一份有判别力的材料应是平台提供的该次运行编译结果、用户脚本 stdout/stderr、退出码和工作目录契约。

以上是已检查前端范围内的结论，不声称网站不存在其他官方支持渠道。

## 证据目录

| 证据 | 内容与定位 |
| --- | --- |
| `.baf13c38c0939482.js` | 练习测评页。单行压缩文件；`async function c` 是预检，`async function K` 是分片上传，`eX=async` 是提交处理器。 |
| `tmp_logchunks/239582d5d4bc9916.js` | 测评详情页。`function N` 选择日志来源，`evaluation/detail` 请求详情，`new Blob([s]` 实现日志下载。 |
| `tmp_logchunks/f8683864e80ba007.js` | 竞赛中心；`competition/eval` 的提交与列表，和练习测评独立。 |
| `tmp_logchunks/acb24c2f1e947599.js` | 在线仿真页；`Function("pub_twist","get_lidar","get_position","time",c)` 是代码执行器。 |
| `tmp_logchunks/8dfc04020a7396c7.js` | 仿真中心模型/传感器/场景编辑；`tO` 为场景加载按钮处理器。 |
| `.d6103d4ea3a88e06.js` | 导航把 `simulation`、`online-simulation`、`evaluation`、`competition` 分成独立栏目。 |
| `.evaluation.html` | 实际引用 `/_next/static/chunks/baf13c38c0939482.js` 等资源的 HTML。 |
| `.firecrawl/inspection-guide.pretty.json` | 已缓存公开巡检指南，`updatedAt=2026-10-11 01:24:58.745258+08`；本次复用缓存。 |
| `.guide2.png`、`.guide3.png` | 指南图：示例 `run_example_mission.sh`，以及删除仿真/裁判启动命令的示意。 |
| `.firecrawl/ros-evaluation-notes.txt` | 官方《ROS 仿真测评系测评注意事项》转录。 |
| `docs/platform_test_20261106.md`、`docs/platform_probe_foreground_20261106.md`、`docs/platform_probe_result_20261106.json` | 已授权账号先前的实测结论。 |

## 练习测评的上传契约

入口：`/ros-simulation/evaluation`。前端内置 `ican`、`delivery`、`inspection` 三种 `scenarioKey`，均 `dualUpload: true`。

| 项目 | 前端实际行为 |
| --- | --- |
| 源码文件 | 界面接受 `.zip,.tar,.gz`；预检后缀允许 `zip/tar/tar.gz/tgz`。练习页限制 `size <= 0x1400000`，即 20 MiB。 |
| 源码结构 | 对能解析的 ZIP 中央目录，要求至少存在精确布局 `src/<package>/package.xml`。不是 `workspace/src/<package>/package.xml`。 |
| 启动脚本 | 独立第二文件，`.sh`；三个内置场景均要求存在。 |
| 预检能力边界 | ZIP 预检读取目录名，不解压/编译源码；对于没有成功解析目录的文件不会据此证明包结构有效。它不是完整服务端校验。 |
| 提交请求 | `POST /api/ros-simulation/evaluation`，`multipart/form-data`。字段：`scenarioKey`、可选当前登录者 `userId`、`file`、`scriptFile`、`note`。前端存在登录 token 时带 Bearer 认证。 |
| 大包 | 源码超过 `0xc00000`（12 MiB）时先分片，每片 8 MiB，再在最终提交中以 `objectKey,srcFileName,srcFileSize` 替代 `file`。脚本仍为 `scriptFile`。 |
| 进行中限制 | 本场景已有 `submitted/queued/running` 时阻止再次提交，文案为同一场景同时只允许 1 条在测。 |
| 成功响应 | JSON `success=true` 时读取 `data.id`，跳转 `/ros-simulation/evaluation/log/<id>`。 |
| 服务端不可用 | 页面可见 `serverDown`、`pendingResubmitHint`、`resubmittedCount` 等状态；这说明前端支持排队/重推提示，不暴露 runner 内部日志路径。 |

已观察的分片上传接口为 `POST /api/ros-simulation/evaluation/upload`。同一地址以表单 `mode=init/chunk/complete/abort` 区分操作，使用 `uploadId`、`objectKey`、`partNumber` 和 `parts` 等字段。这里仅记录公开代码契约，本次未调用上传。

前端静态预检会提示：

- 脚本包含 `gzserver/gzclient/gazebo` 或 `ign gazebo`。
- 脚本包含 `.deb`、`apt install/apt-get install`、`dpkg -i` 等安装流程。
- 源码包含、脚本引用官方保留包：`ican_land_sim`、`ican_shoot_sim`、`ican_delivery_sim`、`ican_inspection_sim`、`ican_referee_system`、`ican_delivery_referee`、`ican_inspection_referee`。

前端把其中若干情况作为 warning，**不表示服务器允许它们**。官方注意事项明确：平台还会静态扫描及在容器运行时包装命令；仿真/裁判由平台负责；命中同类运行命令可能退出 126；无图形评测环境不应启动 RViz。

## 已观察的只读请求与详情输出

这些路径均直接出现在已下载前端中；本次没有对记录 API 发起请求。

| 请求 | 查询字段或作用 |
| --- | --- |
| `GET /api/ros-simulation/evaluation/guide?scenarioKey=<key>` | 公开场景指南，消费 `data.blocks`。缓存中巡检指南只有文字和三张示意图，没有 ZIP 附件块。 |
| `GET /api/ros-simulation/evaluation/access` | 当前用户场景资格、订阅/免费配额与标签；前端可带当前用户查询和认证。 |
| `GET /api/ros-simulation/evaluation` | 当前用户当前场景列表；`scenarioKey,page,pageSize=10`，可带当前用户查询和认证。 |
| `GET /api/ros-simulation/evaluation/detail` | 当前详情 `id`，可带当前用户查询和认证；响应形态为 `success,data`。 |

详情在 `submitted/queued/running` 时每 4 秒刷新一次。终态 `success/failed` 不再依此自动轮询。详情显示场景、源文件名/大小、脚本名、`runId`、备注、状态、失败原因、耗时、得分及 `scoreDetail`。

日志选择顺序由 `function N` 明确规定：

1. `tabLogs` 是非空数组时直接采用它；每项包含 `tab,label,content`。
2. 否则，若 `terminalLogs` 非空，用 `terminalLogs.join("\n")` 构造单个“终端日志”标签。
3. 否则无日志。界面再过滤 `content.trim()` 为空的标签。

这意味着 `tabLogs` 已存在时，不会把 `terminalLogs` 另外拼接进去。当前保存的实测响应是 `tabLogs` 只有裁判，`terminalLogs=[]`，故并非用户切错标签造成的遗漏。

“下载当前终端日志”出现条件是有非空终端且 `downloadEnabled === true`。实现为当前标签的完整 `content` → `Blob(text/plain;charset=utf-8)` → 浏览器本地 `.log`；文件名由标签名和 `runId` 构成。**没有额外 HTTP 请求，不会下载接口未返回的日志。** 页面显示最多最近 8000 行，但下载使用完整当前标签内容。

另一个容易误读的显示行为：终态 `score=null` 时，界面把分数回退为 0。因此 `failed + score=null` 的“0 分”不能当作评测产生了有效零分成绩。

## 指南实际能确认与不能确认的内容

缓存指南明确写道：只提交源码压缩包和启动脚本；启动脚本用于自己的源码和主程序；不需要附带启动仿真与裁判的命令。图片 3 也标记删除相应两条命令。

图片使用 `run_example_mission.sh` 和 `gnome-terminal` 多标签形式作为示例，并展示 source ROS Humble 与工作空间 `install/setup.bash` 的思路。但图片不是 runner 规范，**不能据此认定平台必须使用 gnome-terminal，或认定上传独立脚本在容器内保留示例相对目录结构**。

前端与已缓存指南均没有给出：解压/编译目录、执行脚本时的 cwd、脚本最终落盘路径、`install/setup.bash` 的准确位置、是否自动 source、允许的 shell/终端程序、超时与退出码回传规则。它们也没有给出巡检控制话题的消息类型、字段、坐标约定或地图/目标图库。

官方注意事项提供的受支持运行诊断方式是 `ros2 topic info` 查看发布者、`ros2 topic hz` 查看频率、在自己的回调中计数；不能仅凭 topic list 里出现名字判断可用。该方法仍需要脚本真实进入用户 ROS 域，且用户输出或自有记录可见。

## 竞赛中心不能与练习测评混用

`/ros-simulation/competition` 使用独立 `POST /api/ros-simulation/competition/eval`，源码/脚本字段是 `scenarioKey,file,scriptFile,note`，场景取队伍绑定值。列表同路径 GET，进行中约每 8 秒刷新。

竞赛页标签写源码 `zip/tar.gz <=50MB`，练习页实际代码限制 20 MiB。不能把一个入口的标签大小推广到另一个入口，也不能把练习日志诊断当作已经完成竞赛中心正式提交。

## 仿真中心与在线仿真的公开资源线索

导航中两者分别为 `/ros-simulation/simulation` 和 `/ros-simulation/online-simulation`，与测评栏目独立。

仿真中心前端加载机器人类别/模型、传感器定义、预定义场景、GLB 模型等资源；已观察引用包括 `robot-categories`、`robot-models`、`sensor-definitions`、`predefined-scenes`、`glb-models`、`scenes`。部分另有 `scenes/mine` 和保存配置等当前用户资源，本次没有访问。

这些前端资源用于 URDF/SDF/XML 解析、3D 场景与 GLB 展示、模型/传感器配置。已检查代码中没有看到把它们导出为 `demo_inspection_ws.zip`、巡检 ROS2 工作空间、裁判接口文档或目标图库的入口。

仿真中心“加载场景”处理器 `tO` 用 `setTimeout` 更新 React 状态并提示“已加载至 Gazebo”；该处理器本身没有运行容器调用。这个 UI 提示不能证明真正启动了官方评测环境。

在线仿真“执行代码”的具体实现更明确：

- 跳过 `import` 行，用正则把部分 `if/for/while/print/math` 等 Python 风格文本转为 JavaScript。
- 使用 `Function("pub_twist","get_lidar","get_position","time",c)` 在浏览器执行。
- `pub_twist` 收集前进/转向与时长，之后通过定时器更新页面；`get_position` 返回本地积分的位姿。
- `get_lidar` 回调使用 `Math.random()` 构造值；传感器展示还有随机回退值。
- 启动按钮主要设置运行状态与 3D 场景状态。已检查该页未出现 shell/ZIP 上传或通用 ROS2 runner 请求。

因此该页可用于其页面支持的控制演示，不能验证本项目启动脚本、ROS 包、DDS 白名单、巡检裁判接口或官方场景闭环。页面中的机器人/场景模型也不能自动等同于比赛用的地图、尺度或真实传感器定义。

## 在缺少官方工作空间时的下一步

当前可做且有证据支持的工作，是对自己的提交物做 ZIP 布局、脚本语法、LF 换行、禁用命令、依赖、ROS2 包安装/console entry/source 路径的本地验证；这些只验证提交物自身，不验证官方接口。若本机无 ROS2，可在独立 ROS2 Humble 环境完成自有包的编译与无场景导入检查。

平台侧应通过自己的详情页保留现有 `runId`、原始状态、失败原因及可下载标签日志。现有报告中最近前台探针运行约 41.2 秒后失败，且只有裁判三行启动日志；普通 shell 与 gnome-terminal 变体都未提供用户输出，继续更换包装方式无法消除上述歧义。

最小平台支持请求应包含该次运行标识，并索取：编译命令与结果；启动脚本 stdout/stderr 与退出码；解压/编译/执行工作目录；已编译 workspace 的 source 方法；用户终端日志如何进入 `tabLogs`。这组材料可以先解决 runner 问题，不必等待完整场地图和目标图库。之后再用公开消息接口与官方资料绑定巡检任务。

本次发现已有公开指南缓存可复用，未重新抓取；Firecrawl CLI 当前未认证。该状态没有被用来推断网站资源不存在。报告结论主要依据上述已下载可审计文件。
