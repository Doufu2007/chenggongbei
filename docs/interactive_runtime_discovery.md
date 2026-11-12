# 测评之外的公开交互入口核查

日期：2026-11-11。只读请求公开页面、导航配置和它们实际引用的前端JS；未使用账号、token或cookie，未请求`auth/me`、个人场景或创建/保存/启动云资源接口，未发消息。Firecrawl CLI状态为未认证，因此本轮使用不带凭据的公开HTTPS GET；仅此文档落盘。

结论：找到与`/evaluation`独立的两个交互页面，其HTML均HTTP 200，但本轮未找到能运行本队`.sh`/ROS2包、连接真实巡检传感器或取得官方裁判计分的独立终端/GUI合同。不能由测评服务down推断这些页面down；反过来，页面可加载也不能证明存在可用的ROS运行环境。

## 准确入口与当前导航状态

| URL | 2026-11-11公开GET结果 | 已观察合同 |
| --- | --- | --- |
| [仿真中心](https://www.splenach.com/ros-simulation/simulation) | HTTP 200，页面引用`8dfc04020a7396c7.js` | 机器人/传感器/场景配置与浏览器3D展示 |
| [在线仿真](https://www.splenach.com/ros-simulation/online-simulation) | HTTP 200，页面引用`acb24c2f1e947599.js` | 浏览器内简化Python风格代码转换和运动演示 |
| [公开导航配置](https://www.splenach.com/api/ros-simulation/nav-config) | HTTP 200、`success=true` | `simulation`、`online-simulation`与`evaluation`是独立nav_key |
| [公开侧栏配置](https://www.splenach.com/api/ros-sidebar-labels) | HTTP 200、`success=true` | `hiddenKeys=["online-simulation"]`；`testing.simulation.openAt`和`testing.online-simulation.openAt`均为`2026-11-12`；`isDeveloper=false` |

布局chunk`2f07c8f793a0e30e.js`的`RosTestingOperationProvider`读取`testing`字典。`guardOperation`先检查订阅状态，再对字典中存在的功能弹出“测试阶段，暂未开放”提示并阻断操作；`openAt`作为展示文本，没有在该处理器里按日期自动解锁。其路由表仍是simulation、online-simulation、algorithm、ai-assistant、courses、forum、evaluation、competition，未出现独立ROS终端路由。这里记录的是公开配置和前端行为，未核验账号内的订阅/开发者例外。

## 实际执行路径

仿真中心主chunk中，场景加载处理器`tO`用`setTimeout(..., 2000)`修改React状态、设置`Gazebo Fortress`文字并提示“已加载至Gazebo”；启动RViz的`tR`同样使用`setTimeout(..., 1500)`。所检查处理器没有启动容器、Gazebo进程或远程终端请求，页面日志还包含通过定时器追加的固定节点启动文本。传感器演示数据函数含`Math.random()`。这些文字不能当成真实Gazebo/ROS启动日志。

在线仿真主chunk将部分Python风格语句转成JavaScript，跳过import行，最终调用：

```javascript
Function("pub_twist", "get_lidar", "get_position", "time", c)
```

注入的`pub_twist`把命令加入本地数组，`time.sleep`推进本地时间并积分x/y/yaw，`get_position`返回该局部对象，`get_lidar`返回由`Math.random()`生成的五个值。因此这里执行的是页面支持的控制演示，不能运行`colcon`、`rclpy`、ROS2 launch或上传的shell脚本，也不等于当前官方巡检场景/传感器/裁判。

前端引用的API主要是以下资源合同，**本轮只读取代码，没有调用这些资源写接口或个人资源接口**：

- `/api/ros-simulation/robot-categories`、`robot-models`、`sensor-definitions`、`predefined-scenes`、`glb-models`、`scenes`：模型、场景和素材数据。
- `/api/ros-simulation/robot-configs`、`sensor-configs`及`scenes/mine`：配置/个人资源；存在GET、POST或DELETE前端分支，不能当成启动运行服务。
- `/api/ros-simulation/ai/code-assistant`、`ai/generate-robot`、`ai/robot-specs`、`ai/providers`：AI与模型配置，未见ROS runner合同。
- 3D依赖`52d8fe276283b308.js`引用`image-proxy`、`glb-models/presign`和AI步态分析；未出现shell、rosbridge或真实传感器WebSocket地址。

主chunk未出现WebSocket/rosbridge/xterm/noVNC/code-server。公共依赖`5decc908776f5c8b.js`中的三处`webSocket`命中是Next客户端初始化参数传递，不是机器人连接入口。布局中的`terminal`命中为图标定义。

## 可复核静态证据

两页面当前引用的核心JS与已有本地缓存逐字节一致：

| 公开chunk / 本地缓存 | 当前响应字节数 | SHA-256 |
| --- | ---: | --- |
| `/_next/static/chunks/8dfc04020a7396c7.js` / `tmp_logchunks/8dfc04020a7396c7.js` | 139217 | `cb49b75121e4c89ae0770ef8e38139fc11eba31f0f26888c287cc563432f04a1` |
| `/_next/static/chunks/acb24c2f1e947599.js` / `tmp_logchunks/acb24c2f1e947599.js` | 86571 | `f3f2037161ce6b971652e66ffbfe8af893a1130038e4aa8d7b5a200d1bdd988b` |

公开配置的现有本地副本为`.firecrawl/resource-20261111-nav-config.json`与`.firecrawl/resource-20261111-ros-sidebar-labels.json`，本轮实时读取的关键字段一致。以上仅覆盖实际页面引用的前端实现和公开导航，不声称网站不存在尚未发布或另行授权的真实ROS运行服务。当前可行结论是：这些页面没有提供已证实可替代练习测评的ROS终端；不能为了点击运行而绕过测试阶段门禁，也不能将浏览器演示计入官方任务完成率。
