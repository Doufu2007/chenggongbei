# 官方巡检工作空间下载来源核查

更新日期：2026-11-11。当前结论：官方审核明确表示工作空间等资料位于官方百度网盘；已检查的课程API仍未提供附件，尚未取得实际官方分享链接。另已找到并保存两份公开第三方环境候选，不能把候选认定为当前官方版本。

## 2026-11-11 最新账号内与公开来源证据

- `runs/evidence/20261111/support_post.json`：原求助帖`status=rejected`，审核时间2026-11-10 15:40:21 +08，说明原文为“工作空间等等文件都在官方的百度网盘连接中。测评侧的构建、运行日志等等内容都是不展示的。”资料渠道已有官方说明，具体分享地址在该回复中未给出。
- `runs/evidence/20261111/inspection_course.json`：课程仍为视频内容，`documentKey`、`documentContent`、`documentFileName`及lesson `content`为`null`。这只能证明所检查课程响应没有附件，不能反驳资料通过官方网盘或其他授权渠道分发。
- 公开候选来源为[ranmengjia/ICAN](https://github.com/ranmengjia/ICAN)（commit `82b041043849a5d86e085390bf0df787366d98cd`）与[wbxxmz/ros2-car-first](https://github.com/wbxxmz/ros2-car-first)（commit `48b3dac26b6f562ff21dd321db03fd6ffc91b492`）。本地保存接口/地图/静态场景/launch及七类候选参考图，固定版本与逐文件哈希见`external_resources/`中的`SOURCE_MANIFEST.json`；详细核查见`docs/resource_search_20261111.md`。
- 两份第三方副本未证明和当前官方环境一致，也未证明图库覆盖当前随机图片全集；未将他队主程序、预编译插件或裁判结果作为本队实测。来源和使用范围必须保留到候选配置与材料中。
- 今日`evaluation_history.json`和`latest_probe_detail.json`返回`serverDown=true`、`serverStatus=down`。服务状态不会使第三方资料变成官方资料，也不能回推历史失败根因。构建/运行日志不展示属于已获官方说明的显示边界，仍需通过允许的运行结果核实当前接口。

后续可复核依据仍是官方百度网盘中的实际原始工作区/图库，与当前平台允许观测的topic、类型、frame及有效计分。没有实际分享地址前，不从视频文件名猜测存储URL。

## 2026-11-06 本地来源核查（历史记录）

以下范围与结论是2026-11-06的本地静态核查，不覆盖上节后来取得的账号内回复和第三方候选。

结论：**当前本地证据中没有找到可访问的 `demo_inspection_ws.zip` 下载链接。** 视频画面能确认该文件名和示例目录存在，不能确认它的网络分发地址。以下是本次独立复核结果，不把“未找到”扩大为“官方没有提供”。

## 检查范围与结果

| 范围 | 结果 |
| --- | --- |
| `.firecrawl/` 的 JSON、HTML、文本，`tmp_logchunks/` 的 JS，根目录已下载 JS 与 `.evaluation.html`，共 40 个文本资源 | 搜索 `demo_inspection_ws`、`inspection_ws`、下载 URL/附件字段、压缩包扩展名及常见仓库/网盘域名。未找到巡检工作空间下载引用。 |
| 已下载前端中的压缩包名称 | 仅找到测评上传说明/类型中的 `.zip`、`src.zip` 等输入提示，没有 ZIP 下载地址。 |
| `tmp_logchunks/7cb4c908be122629.js` 课程中心 | 公开引用 `GET /api/ros-simulation/courses`，根据返回的 `courseId` 打开 `/courses/<id>/video` 或 `/courses/<id>`。该 chunk 没有附件字段、工作空间名或下载链接；本次未请求课程列表。 |
| `.firecrawl/inspection-guide.json` 与 `inspection-guide.pretty.json` | 同一巡检指南数据：3 个 text block、3 个 image block；只有 `图片1.png`、`图片2.png`、`图片3.png`，没有 file/ZIP 附件块。指南前端对非文字块按图片渲染。 |
| `.firecrawl/ros-evaluation-notes.txt` 及同内容副本 | 官方注意事项提供评测约束与诊断建议，没有工作空间分发地址。 |
| `course_transcript.txt`、`course_transcript_prompt.txt`、`course_transcript_repaired.txt` | 未发现 URL、网盘、仓库、附件或资料分发说明。前两份转录出现的“下载”在安装步骤上下文，不能据此构造工作空间地址；转录本身有明显识别噪声。 |
| `course_frames/000000.jpg` 与 `course_contact.jpg` | 开头画面显示 Home 下的 `demo_inspection_ws` 文件夹及 `demo_inspection_ws.zip` 文件。已查看缩略总览，没有可读网络下载地址或资料分发页面。 |
| 现有 `docs/platform_submission.md`、`docs/testing_report.md` | 先前人工页面观察记载课程作业为空、没有可见下载附件。它们是会话报告，不是下载来源。 |

## 已观察的合法入口

已记录的官方课程页面是：

https://www.splenach.com/courses/43296edb-1673-4643-8922-f13a7523f892/video

该 URL 来自现有项目报告；课程视频也已保存为 `course_inspection.mp4`。本次仅使用本地材料，未重新访问课程页。当前 `.firecrawl/` 没有该课程详情页的 HTML/JSON 快照，因而不能独立重验它此刻的附件列表。

缓存名为 `splenach-raw.html` 与 `splenach-competition-raw.html` 的两份文件字节相同，只有应用静态资源引用，不含课程 ID、附件或 ZIP 下载 URL；不能误把它们当作该课程完整内容快照。`gican-raw.html` 与 `gican-details-raw.html` 也彼此相同，均为 iCAN 应用壳，未含该工作空间引用。

## 可交付判断

- 能确认：官方教学视频演示的是名为 `demo_inspection_ws.zip` 的本地压缩包；它包含示例工作空间。课程中心允许从课程卡片进入实际课程页。
- 尚不能确认：该 ZIP 当前是否通过课程附件、赛事资料包、正式群文件或其他官方渠道分发；没有任何已观察 URL 可供合法下载。
- 所需最小外部补充：由老师/赛事方提供该压缩包的明确公开或对本账号授权的下载链接，或在实际课程页补齐附件；也可提供等价的公开接口说明、示例节点、地图和参考图资源。不要根据视频文件名猜测站点存储路径。

本次没有下载任何新网络资源，没有发送消息，也没有消耗测评次数。

## 2026-11-11 早期公开资料复核（后续状态见首节）

本轮目标收窄为先完成线上复赛：用户暂无实机、无新版规则和官方示例工作空间；不把实机采购或线下迁移作为当前前置条件。

- 已重新读取已知官方课程 API。全域巡检课程仍为一个章节、一个视频；`documents`、`documentKey`、`documentContent`、`documentFileName` 及 lesson `content` 均为空，没有发现示例 ZIP 附件。快照保存在 `.firecrawl/recheck-20261111-course.json`。
- 已重新读取现有 V1.0 判分细则 API，正文与本地历史副本一致；这只确认该已知课程未变，不能证明其他入口不存在新版规则。
- 此前求助帖的公开评论接口返回 `comments:[]`、`total:0`；帖子详情匿名读取返回 HTTP 403，不能核实账号内审核状态或私密回复。
- 两种浏览器自动化运行环境均在初始化时报告 `failed to write kernel assets`，未能操作现有登录会话。HTTP 默认代理连接被拒绝，改用不依赖代理的公开 HTTPS 请求后正常读取课程；未更改系统代理设置。
- 本轮未发送新消息、未登录新账号、未发起测评或消耗正式提交次数。

## 2026-11-06 后续在线核查

本轮重新检查已登录课程页及 `GET /api/courses/43296edb-1673-4643-8922-f13a7523f892`。课程内容只有一个章节、一个视频；`documentKey`、`documentContent`、`documentFileName` 和 lesson `content` 均为 null，没有工作空间附件字段或可见 ZIP 链接。页面的“部署”为 DSL/v86 演示虚拟机，“终端”为通用代码编辑器；它们不是 ROS2 巡检工作空间。没有在这些通用环境中安装或运行替代仿真。

同时在官方课程列表找到《应急救援机器人挑战赛裁判系统判分细则 V1.0》，并通过 `GET /api/courses/129d6e75-f19a-4c97-9ec6-0853695a4c48` 保存原文为 `referee_rules_v1_api.json`、`referee_rules_v1.html`、`referee_rules_v1.txt`。该资料确认抽签格式、识别话术、任务/窗口/降落声明和灯光命令，因此这些部分已经能够实施，不再依赖论坛帖子。它没有提供地图、目标图库、控制坐标系、识别确认 schema 或工作空间 ZIP 链接。

用户已明确回复目前无法取得工作空间下载链接。本轮未再次要求相同资料，也没有发送技术支持消息或提交新的测评。
