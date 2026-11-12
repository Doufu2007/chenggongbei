# Third-party notices

本工程包含自行实现的任务状态机、控制器、识别适配、参数文件、诊断工具和测试。运行环境依赖 ROS 2、OpenCV、NumPy、PyYAML 和 cv_bridge，各组件遵循其各自许可证。

当前快照还保留以下第三方参考资产，不能把它们声明为本队原创，也不能据公开可访问推断拥有重新许可权：

- `external_resources/ranmengjia_ICAN/`：公开候选世界、地图、接口及图像资料，来源与固定提交、文件 SHA256 见该目录 `SOURCE_MANIFEST.json` 及补充清单。上游元数据未声明许可证；本目录记录来源和使用边界，不授予额外权利。候选几何尚不代表官方当前版本确认。
- `external_resources/wbxxmz_ros2-car-first/`：公开参考图像，固定版本与 SHA256 见该目录 `SOURCE_MANIFEST.json`。运行包中的 `assets/ground_templates*` 和 `assets/flight_templates` 保留逐图来源清单；现场随机图像类别仍须由本局传感器识别。
- `external_resources/openai_CLIP/`：独立视觉探索使用的 OpenAI CLIP 源码与 MIT 许可证，保留上游许可证和来源清单。大型模型权重未上传，官方来源与校验在 `MODEL_SOURCE.json`；它未接入默认完整任务识别器。

官方规则、课程相关文本用于记录接口和判分依据，其权利属于发布方。公开副本未包含课程视频及第三方工具可执行文件。项目没有新增整个仓库的统一再许可声明。

