# 官方工作空间入口补查（2026-11-11）

本轮只补查尚未核验的原图二维码、平台赛事技术讨论区及 iCAN 原通知入口，未获得新的官方工作空间或百度网盘直链。该结论仅覆盖以下来源，不能据此断言官方未发布资源。

- 对现有 iCAN 原图 16 张及测评指南图 3 张做二维码解码。仅 `gican-img/img02.png` 解出微信链接 `http://weixin.qq.com/r/PxwVDZbEmsvlrdJu90kt`；其余未成功解码，不能视为已证明不存在二维码。逐图 SHA256 与结果保存在 `.firecrawl/official-link-20261111-qr-results.json`。
- 公开论坛首页取回 20/114 个帖子；赛事技术讨论分类 `48310089-d35a-4241-bb3e-cc6a19914d6e` 取回全部 16 个帖子。分类内本次返回的帖子均未置顶，`attachments` 均为空；正文唯一图片位于“每个赛项怎么才算真正结束？”帖中，文字上下文将其描述为结束标志对照表。未从这些正文取得工作空间或网盘链接。不能将论坛用户发帖直接当作官方规则。
- 论坛以“百度网盘”精确搜索，本次返回 `total=0`。原始响应与请求清单分别保存在 `.firecrawl/official-link-20261111-forum-search-baidu.txt`、`official-link-20261111-followups.json`。
- iCAN 原通知 `http://g-ican.com/competition/details?id=97` 返回前端壳。按原 HTML 明示的组件脚本 `http://g-ican.com/js/chunk-0b77c6aa.070cebba.js`，核实详情使用 `competitionId` 参数；按 app 脚本明示的 API host 和接口，公开 GET `https://api.g-ican.com/v1/api/competitionInfo/detail?competitionId=97` 返回 HTTP 403，未取得通知详情正文。组件下载与 API 响应的大小、SHA256、URL 记录在 `.firecrawl/official-link-20261111-final-fetch.json` 和 `official-link-20261111-notice-fetch.json`。

Firecrawl 本机仍未认证，本轮使用不带凭据的公开 HTTP GET；未加入群、未发送消息、未猜测存储路径、未尝试绕过 403。课程附件、站点配置与第三方候选工作空间的既有核验见 `docs/resource_search_20261111.md`。本轮收尾后停止扩大检索，优先使用真实平台传感器证据核对候选接口。
