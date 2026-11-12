# 官方巡检工作空间公开恢复核查（2026-11-06）

## 结论

截至 2026-11-06，未找到可公开、可核验、可直接下载的官方 `demo_inspection_ws.zip`，也未找到包含城市应急全域巡检地图、目标图库和控制接口的官方源码包。当前阻塞仍是“缺少官方工作空间/接口样例”，不能据此声称平台闭环已验证。

## 已核验的官方入口

1. [全域巡检环境配置课程 API](https://www.splenach.com/api/courses/43296edb-1673-4643-8922-f13a7523f892)
   - `chaptersCount=1`，只有一个环境配置视频。
   - `documentKey`、`documentContent`、`documentFileName` 均为空。
   - 章节 lesson 的 `content` 为空，视频对象只有带签名的 MP4 播放地址。
   - API 响应中没有 ZIP、附件、Git 仓库或工作空间下载字段。
2. [全域巡检课程页面](https://www.splenach.com/courses/43296edb-1673-4643-8922-f13a7523f892/video)
   - 页面脚本只请求课程详情和访问权限；没有发现公开工作空间附件入口。
   - 课程视频画面中出现本地目录 `/home/leea/demo_inspection_ws`，这是演示环境中的本地路径，不是可下载地址。
3. [官方判分细则 API](https://www.splenach.com/api/courses/129d6e75-f19a-4c97-9ec6-0853695a4c48)
   - 确认巡检话题和流程：`/referee/ground_tasks`、`/inspection/detections`、`/referee/current_target`、`/referee/cruise_event`、`/uav/laser_fire`。
   - 规则给出了话术与顺序，但没有地图坐标、观察位姿、相机外参、控制器接口或结束消息合同。

## 公开搜索记录

- GitHub Repository API 查询 `ican_inspection`、`demo_inspection_ws`、`shenggong ros`、`splenach` 均返回 0 个仓库。
- GitHub 对 `shenggong` 的结果是无关个人仓库或其他参赛队伍仓库；没有确认属于赛事主办/承办方的巡检源码。
- Bing 精确查询 `ican_inspection_navigation`、`ican_inspection_examples`、`ican_inspection_env_binary`、`demo_inspection_ws` 均返回“没有与此相关的结果”。
- Gitee API 对 `demo_inspection_ws`、`ican_inspection`、`shenggong`、`rosbot` 未返回可核验的官方仓库。
- 搜索到的公开仓库主要是其他参赛队伍的陆地清障练习工程；它们没有巡检工作空间，且没有可确认的授权。按赛事账号与代码权属规则，不复制、下载或并入本队代码/资产。

## 当前能安全使用的证据

- 现有源码可以依据 V1.0 公开话题和中文话术做静态接口适配与本地单元测试。
- 正式平台运行仍需要平台实际 ROS 图、机器人控制话题、地图/点位、相机接口和裁判结束判定。只读探针已证明裁判启动，但没有用户程序 stdout/stderr，也没有成绩结束标志。
- 在取得官方工作空间或平台老师提供的接口说明前，不应猜测坐标、私有随机抽签、灯光反馈或私有下载地址，也不应反复消耗正式测评次数。

## 后续解除阻塞所需的最小资料

请从赛事群文件、老师或平台课程后台取得以下任一项：

1. 官方 `demo_inspection_ws.zip`；或
2. 官方接口说明，至少包含机器人底盘/UAV 控制话题、地图与起始位姿、相机话题及坐标外参、目标图文件名/类别映射、识别确认和正常结束判定。

本报告仅记录公开核查结果；没有修改源码、上传网站、发送第三方消息或读取裁判私有随机化数据。
