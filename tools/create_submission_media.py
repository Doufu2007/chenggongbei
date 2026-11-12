"""Build evidence-bound PDF materials; never manufacture a platform recording.

Default use: python tools/create_submission_media.py --junit dist/current_tests.xml
Importing this module neither writes assets nor runs the artificial scenario.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape


ROOT = Path(__file__).resolve().parents[1]
FONT = Path("C:/Windows/Fonts/msyh.ttc")
BOLD = Path("C:/Windows/Fonts/msyhbd.ttc")


def file_evidence(path: Path) -> dict:
    path = path.resolve(strict=True)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "bytes": path.stat().st_size}


def read_junit(path: Path) -> dict:
    """Count leaf cases, rejecting contradictory report counters."""
    root = ET.parse(path).getroot()
    cases = list(root.iter("testcase"))
    if not cases:
        raise ValueError("JUnit contains no test cases")
    counts = {"tests": len(cases), "failures": 0, "errors": 0, "skipped": 0}
    for case in cases:
        status = [key for key, tag in (("failures", "failure"), ("errors", "error"),
                                      ("skipped", "skipped")) if case.find(tag) is not None]
        if len(status) > 1:
            raise ValueError("JUnit case has conflicting outcomes")
        if status:
            counts[status[0]] += 1
    for suite in root.iter("testsuite"):
        own_cases = list(suite.iter("testcase"))
        expected = {"tests": len(own_cases), "failures": 0, "errors": 0, "skipped": 0}
        for key, tag in (("failures", "failure"), ("errors", "error"), ("skipped", "skipped")):
            expected[key] = sum(case.find(tag) is not None for case in own_cases)
        for key, value in expected.items():
            if key in suite.attrib and int(suite.attrib[key]) != value:
                raise ValueError(f"JUnit {key} counter does not match cases")
    counts["passed"] = counts["tests"] - sum(counts[key] for key in ("failures", "errors", "skipped"))
    return {**counts, "evidence": file_evidence(path)}


def read_validation_record(path: Path) -> dict:
    """Accept supplied local records as attributed evidence, not official results."""
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("schema_version") != 1 or value.get("kind") != "local_validation":
        raise ValueError("validation record must be schema_version 1 / local_validation")
    scenario = value.get("offline_scenario")
    if scenario is not None:
        if (not isinstance(scenario, dict) or type(scenario.get("completed")) is not bool
                or any(type(scenario.get(key)) is not int or scenario[key] < 0
                       for key in ("steps", "reports"))
                or scenario.get("protocol") not in ("legacy", "inspection_v1")):
            raise ValueError("invalid offline scenario record")
    value["evidence"] = file_evidence(path)
    return value


def collect_validation(*, junit=None, validation_record=None, run_local_scenario=False,
                       probe_evidence=None, probe_output=None, probe_summary=None, probe_run_record=None) -> dict:
    result = {"generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
              "junit": read_junit(Path(junit)) if junit else None,
              "local_record": read_validation_record(Path(validation_record)) if validation_record else None,
              "readiness": None, "profile_verified": None, "probe_runtime": None,
              "probe_run_record": None, "inputs": []}
    if probe_evidence is None and (probe_summary is not None or probe_run_record is not None):
        raise ValueError("probe summary/record require raw --probe-evidence")
    if probe_evidence is not None:
        if probe_output is None:
            raise ValueError("probe evidence requires an analysis output directory")
        sys.path.insert(0, str(ROOT))
        from tools.analyze_probe_evidence import analyze_evidence
        probe = analyze_evidence(Path(probe_evidence), Path(probe_output))
        if probe["status"] == "invalid":
            raise ValueError("invalid probe evidence; inspect analysis parse_errors")
        if probe_summary is not None:
            supplied = json.loads(Path(probe_summary).read_text(encoding="utf-8"))
            if (supplied.get("schema_version") != 1
                    or supplied.get("source_evidence", {}).get("sha256") != probe["source_evidence"]["sha256"]
                    or any(supplied.get(key) != probe[key] for key in (
                        "record_count", "event_counts", "startup", "sample_topics", "images",
                        "reported_sample_counts", "sample_record_counts"))):
                raise ValueError("supplied probe summary does not match raw callback")
            result["inputs"].append(file_evidence(Path(probe_summary)))
        result["probe_runtime"] = {key: probe[key] for key in (
            "status", "official_scoring", "record_count", "event_counts", "startup",
            "sample_topics", "launcher_exit_codes", "missing_events", "images",
            "reported_sample_counts", "sample_record_counts", "missing_snapshot_fields")}
        result["probe_runtime"]["full_snapshot_count"] = probe["snapshots"]["full_count"]
        messages = {}
        for sample in probe["samples"]:
            data = sample["data"]
            messages.setdefault(data["topic"], data["message"])
        result["probe_runtime"]["sample_messages"] = messages
        if probe_run_record is not None:
            from tools.run_records import validate_record
            record = validate_record(json.loads(Path(probe_run_record).read_text(encoding="utf-8")))
            if record["kind"] != "interface_probe":
                raise ValueError("probe run record must be a diagnostic interface_probe")
            if not any((Path(raw) if Path(raw).is_absolute() else ROOT / raw).resolve()
                       == Path(probe_evidence).resolve() for raw in record["evidence_paths"]):
                raise ValueError("probe run record must reference the same callback")
            result["probe_run_record"] = {key: record[key] for key in (
                "runId", "status", "duration_s", "scores", "failure_reason")}
            result["inputs"].append(file_evidence(Path(probe_run_record)))
        if Path(probe_evidence).is_file():
            result["inputs"].append(file_evidence(Path(probe_evidence)))
        result["inputs"].append(file_evidence(Path(probe_output) / "summary.json"))
        for item in probe["images"]:
            result["inputs"].append(file_evidence(Path(probe_output) / item["file_name"]))
    for relative, key in (("dist/readiness.json", "readiness"),
                          ("src/inspection_mission/config/interface_profile.json", "profile")):
        path = ROOT / relative
        if path.is_file():
            value = json.loads(path.read_text(encoding="utf-8"))
            result["inputs"].append(file_evidence(path))
            if key == "profile":
                result["profile_verified"] = value.get("verified") is True
            else:
                result["readiness"] = value
    for relative in ("docs/referee_rules_v1.txt", "docs/protocol_gap_current.md",
                     "runs/evidence/20261111/support_post.json",
                     "docs/resource_search_20261111.md",
                     "docs/reference_vision_20261111.json",
                     "docs/zero_shot_vision_20261111.json"):
        path = ROOT / relative
        if path.is_file():
            result["inputs"].append(file_evidence(path))
    if run_local_scenario:
        sys.path.insert(0, str(ROOT))
        from tools.mission_scenario import demonstration_plan, run_scenario
        scenario = run_scenario(plan=demonstration_plan(protocol="inspection_v1"))
        result["local_record"] = {
            "schema_version": 1, "kind": "local_validation", "executed_now": True,
            "offline_scenario": {"protocol": "inspection_v1", "completed": scenario.completed,
                                 "steps": scenario.steps, "reports": len(scenario.reports),
                                 "fault": scenario.fault},
        }
        result["inputs"].append(file_evidence(ROOT / "tools/mission_scenario.py"))
    return result


def probe_video(path: Path) -> dict:
    """Read container metadata and decode a frame without rewriting the source."""
    if path.suffix.lower() != ".mp4" or not path.is_file() or not path.stat().st_size:
        raise ValueError("--video-input must be an existing nonempty MP4 recording")
    executable = str(ROOT / "ffmpeg.exe") if (ROOT / "ffmpeg.exe").is_file() else shutil.which("ffmpeg")
    if not executable:
        raise ValueError("ffmpeg is required to validate the platform recording")
    metadata = subprocess.run([executable, "-hide_banner", "-i", str(path)],
                              capture_output=True, timeout=30, check=False)
    details = metadata.stderr.decode("utf-8", errors="replace")
    duration = re.search(r"Duration: (\d+):(\d+):(\d+(?:\.\d+)?)", details)
    stream = next((line for line in details.splitlines() if "Video:" in line), "")
    dimensions = re.search(r"\b(\d{2,5})x(\d{2,5})\b", stream)
    if duration is None or dimensions is None:
        raise ValueError("recording has no verifiable duration or video stream")
    hours, minutes, seconds = map(float, duration.groups())
    duration_s = hours * 3600 + minutes * 60 + seconds
    if duration_s <= 0:
        raise ValueError("recording duration must be positive")
    decoded = subprocess.run([executable, "-v", "error", "-i", str(path), "-map", "0:v:0",
                              "-frames:v", "1", "-f", "null", "-"],
                             capture_output=True, timeout=30, check=False)
    if decoded.returncode:
        raise ValueError("recording video frame cannot be decoded")
    return {**file_evidence(path), "duration_s": duration_s,
            "width": int(dimensions.group(1)), "height": int(dimensions.group(2))}


def validate_official_inputs(record_paths: list[Path], video_path: Path | None) -> dict:
    if not record_paths or video_path is None:
        raise ValueError("official mode requires --run-record and --video-input from a real platform run")
    sys.path.insert(0, str(ROOT))
    from tools.run_records import summarize_records, validate_record
    records = []
    for path in record_paths:
        record = validate_record(json.loads(path.read_text(encoding="utf-8")))
        if record["kind"] != "official_task":
            raise ValueError("official mode rejects diagnostic and offline records")
        records.append(record)
    summary = summarize_records(records)
    if not summary.get("completed_runs") or not summary.get("valid_settled_runs"):
        raise ValueError("official mode requires an evidenced full mission and valid referee settlement")
    video = probe_video(video_path)
    known_animation = ROOT / "submission/城市应急全域巡检效果演示-开发验证版.mp4"
    if known_animation.is_file() and file_evidence(known_animation)["sha256"] == video["sha256"]:
        raise ValueError("artificial development animation cannot be used as a platform recording")
    matched_video = False
    for record in records:
        for key in ("source", "script"):
            source = record.get(key) or {}
            candidate = Path(source.get("path") or "")
            if not candidate.is_absolute():
                candidate = ROOT / candidate
            if (not candidate.is_file() or source.get("sha256") != file_evidence(candidate)["sha256"]):
                raise ValueError(f"official {key} evidence file or SHA256 is missing/mismatched")
        for raw in record.get("evidence_paths", []):
            evidence = Path(raw)
            if not evidence.is_absolute():
                evidence = ROOT / evidence
            if not evidence.is_file() or not evidence.stat().st_size:
                raise ValueError("official evidence path is missing or empty")
            if (evidence.resolve() == video_path.resolve()
                    and summarize_records([record])["completed_runs"] == 1):
                matched_video = True
    if not matched_video:
        raise ValueError("video must belong to the same valid settled complete run in evidence_paths")
    return {"summary": summary, "video": video,
            "records": [file_evidence(path) for path in record_paths]}


def make_flow(path: Path) -> None:
    from PIL import Image, ImageDraw, ImageFont
    image = Image.new("RGB", (1800, 720), "#10283d")
    draw = ImageDraw.Draw(image)
    regular = lambda size: ImageFont.truetype(str(FONT), size, index=0)
    bold = lambda size: ImageFont.truetype(str(BOLD), size, index=0)
    draw.text((70, 45), "城市应急全域巡检：任务顺序示意", font=bold(43), fill="#f2f7fb")
    draw.text((72, 110), "仅为方案流程图，无实景、坐标或裁判成功含义", font=regular(25), fill="#acc7d9")
    stages = [("地面任务", "订单 5 选 3\n点前识别、确认\n进入、声明"),
              ("返航起飞", "地面回指挥中心\n验证起飞条件"),
              ("窗口 1", "声明、识别\n确认后下降穿越"),
              ("空中任务", "两点识别确认\n稳定灯光照射"),
              ("窗口 2", "声明、识别\n确认后下降穿越"),
              ("返航降落", "返航、降落前声明\n实际落地、结算")]
    for index, (title, body) in enumerate(stages):
        x = 55 + index * 287
        draw.rounded_rectangle((x, 245, x + 252, 500), 18, fill="#1a3e55", outline="#5a9eb9", width=2)
        draw.text((x + 126, 285), title, font=bold(31), anchor="mm", fill="#f5f9fc")
        draw.multiline_text((x + 126, 342), body, font=regular(23), anchor="ma", align="center",
                            spacing=13, fill="#c2d9e6")
        if index < len(stages) - 1:
            draw.line((x + 257, 372, x + 280, 372), fill="#eab879", width=4)
            draw.polygon([(x + 280, 372), (x + 270, 365), (x + 270, 379)], fill="#eab879")
    draw.text((900, 595), "工程门禁：新鲜观测、稳定身份、安全净空；本地发布与裁判接受分别记录",
              font=regular(27), anchor="mm", fill="#eacba1")
    image.save(path)


def make_pdf(path: Path, flow: Path, validation: dict, *, official=None) -> None:
    from reportlab.lib import colors
    from reportlab.lib.enums import TA_CENTER
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.lib.units import mm
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image
    for name, font in (("MSYH", FONT), ("MSYH-Bold", BOLD)):
        if name not in pdfmetrics.getRegisteredFontNames():
            pdfmetrics.registerFont(TTFont(name, str(font), subfontIndex=0))
    ink = colors.HexColor("#183147")
    styles = {
        "title": ParagraphStyle("title", fontName="MSYH-Bold", fontSize=25, leading=36,
                                alignment=TA_CENTER, textColor=ink, spaceAfter=15),
        "sub": ParagraphStyle("sub", fontName="MSYH", fontSize=10.5, leading=18,
                              alignment=TA_CENTER, textColor=colors.HexColor("#617d90"), spaceAfter=13),
        "h": ParagraphStyle("h", fontName="MSYH-Bold", fontSize=15, leading=23,
                            textColor=colors.HexColor("#17627d"), spaceBefore=10, spaceAfter=9),
        "body": ParagraphStyle("body", fontName="MSYH", fontSize=9.4, leading=16,
                               textColor=ink, spaceAfter=7, wordWrap="CJK"),
        "small": ParagraphStyle("small", fontName="MSYH", fontSize=8, leading=13,
                                textColor=colors.HexColor("#527084"), spaceAfter=4, wordWrap="CJK"),
        "callout": ParagraphStyle("callout", fontName="MSYH", fontSize=10, leading=18,
                                  textColor=colors.HexColor("#865b25"), backColor=colors.HexColor("#fff3dd"),
                                  borderPadding=9, spaceBefore=7, spaceAfter=16, wordWrap="CJK"),
    }
    paragraph = lambda value, style="body": Paragraph(escape(str(value)).replace("\n", "<br/>"), styles[style])
    def table(rows, widths):
        item = Table([[paragraph(cell, "small") for cell in row] for row in rows],
                     colWidths=[width * mm for width in widths], hAlign="LEFT")
        item.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e7f0f4")),
            ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f6f9fa")]),
            ("GRID", (0, 0), (-1, -1), .35, colors.HexColor("#bacdd7")),
            ("VALIGN", (0, 0), (-1, -1), "TOP"),
            ("LEFTPADDING", (0, 0), (-1, -1), 8), ("RIGHTPADDING", (0, 0), (-1, -1), 8),
            ("TOPPADDING", (0, 0), (-1, -1), 7), ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
        ]))
        return item
    draft = official is None
    label = "技术方案草稿" if draft else "技术方案与真实平台运行记录"
    junit = validation["junit"]
    tests = (f"总计 {junit['tests']}；通过 {junit['passed']}；失败 {junit['failures']}；"
             f"错误 {junit['errors']}；跳过 {junit['skipped']}" if junit else "未传入 JUnit；本文不报告测试数量")
    local = validation["local_record"]
    scenario = local.get("offline_scenario") if local else None
    scenario_text = (f"{scenario['protocol']}：{scenario['steps']} 步、{scenario['reports']} 份识别报告；"
                     f"本地完成={str(scenario['completed']).lower()}" if scenario else "本次未运行，也未传入验证记录")
    completion = "未知" if draft else str(official["summary"].get("completion_rate"))
    video_status = "尚未取得" if draft else "已提供并校验容器元数据与证据关联"
    probe = validation.get("probe_runtime")
    messages = probe.get("sample_messages", {}) if probe else {}
    order = messages.get("/referee/ground_tasks", {}).get("data")
    order_text = (f"本轮公开抽签：{order}；这是任务顺序，不是图片答案或完整五点坐标映射。"
                  if isinstance(order, str) else "data 为 ground_X 的 CSV，5 选 3；本次未提供实际订单样本。")
    laser = messages.get("/uav/laser_state", {}).get("data")
    laser_text = (f"/uav/laser_state 已实收 Bool {str(laser).lower()}；本次回调证明接口存在，"
                  "不证明开启命令交付或照射命中。" if type(laser) is bool else
                  "/uav/laser_state 本次未提供实际样本，存在性和反馈语义仍需实测。")
    probe_text = (f"诊断记录 {probe['record_count']} 条；shell启动事件={probe['startup']['launcher_started']}；"
                  f"probe_started 事件{'已收到' if probe['startup']['probe_started'] else '未收到'}；"
                  f"退出码={probe['launcher_exit_codes']}；"
                  f"收到 {len(probe['sample_topics'])} 类公开消息，话题列表和计数见摘要。"
                  + ("完整诊断快照与真实消息证明探针节点运行；缺失事件仍按未收到保留。"
                     if probe.get("full_snapshot_count") and probe["sample_topics"] else "")
                  + f"分析状态={probe['status']}；official_scoring={probe['official_scoring']}。"
                  "此处仅是诊断证据，不推断编译成功、任务完成或裁判得分。"
                  if probe else "本次未指定回调证据；不从页面状态推断用户节点是否运行。")
    diagnostic = validation.get("probe_run_record")
    if diagnostic:
        total = diagnostic["scores"]["total"]
        total_text = "null" if total is None else str(total)
        probe_text += (f"平台记录：{diagnostic['status']}；成绩={total_text}；"
                       f"平台耗时={diagnostic['duration_s']} 秒。该耗时不是完整任务完成时间。"
                       f"runId={diagnostic['runId']}；原因={diagnostic['failure_reason']}。")
    story = [Spacer(1, 16 * mm), paragraph("“晟功杯”应急救援机器人挑战赛", "sub"),
             paragraph("城市应急全域巡检\n" + label, "title"),
             paragraph(validation["generated_at"], "sub"),
             paragraph("草稿用于说明当前实现和验证边界。官方任务、有效成绩和真实录像尚未完成，不能作为正式平台效果证明。"
                       if draft else "本文从显式提供的真实运行台账与录像构建；成绩和完成率仅按台账有效结算口径列示。", "callout"),
             table([["项目", "当前状态"], ["目标环境", "ROS2 Humble；官方巡检工作空间/等价接口仍待绑定"],
                    ["控制核心", "MissionController、ROS 适配、独立视觉节点与启动预检"],
                    ["接口配置", f"interface_profile.verified={str(validation['profile_verified']).lower()}；该值来自当前配置"],
                    ["自动化测试", tests], ["离线运动学", scenario_text],
                    ["官方任务完成率", completion], ["官方完整运行录像", video_status]], [42, 130]),
             Spacer(1, 8 * mm),
             paragraph("统计口径：人工地图、人工传感器和人工确认只验证本地控制流程。诊断探针退出、本地 COMPLETE 与页面默认分数均不构成官方完整任务成功。", "small"),
             PageBreak(), paragraph("1. 任务流程与实现", "h"),
             paragraph("地面订单由官方公开话题接收，从五个候选点抽取三个并按顺序执行。地面车在点外识别并报告，判定正确后进入点位并声明；随后返回指挥中心，再进行空中巡检。"),
             Image(str(flow), width=172 * mm, height=68.8 * mm),
             paragraph("图 1 方案流程示意。图中不包含真实场地坐标、随机目标答案或已获得分的暗示。", "small"),
             paragraph("感知与身份", "h"),
             paragraph("双相机分别处理输入。视觉模块具备 QR、ArUco 与参考图 ORB/RANSAC 匹配能力，使用当前帧、来源、时间戳和连续稳定证据；已支持同类别多张模板，同一表面保留最佳几何支持，不同类别仍按歧义规则拒绝。已找到含地图及七类图片的公开环境候选，版本尚未与当前官方环境核实。"),
             paragraph("原单模板离线基线未支持跨照片类别泛化；多模板能力已实现，但尚未完成独立保留照片复评。CLIP 整帧分类另有离线探索记录，未提供目标定位或激光对准证据，也未接入默认识别器。加入模板的照片不得继续充当保留测试图。", "small"),
             paragraph("控制与安全", "h"),
             paragraph("状态机区分接近、识别、等待确认、进入、声明及照射。默认 event_ack 等待明确接受；显式 transport_only 只记录本地 publish 成功，短等待及新鲜视觉/位姿后继续，不伪造 ACK，心跳保持 official_scoring=unverified。该策略能解除缺少回执接口的工程阻塞，尚未证明满足裁判“判定正确后”的条件。"),
             paragraph("障碍输入默认保留 LaserScan。显式 pointcloud 模式允许无 UAV 水平扫描，按已配置的传感器平移、坐标帧和视场检查运动方向；前向点云不能代替下视传感器，也不能证明垂直起降、后退或偏航方向净空。候选的下视三束扫描不被当作水平 LaserScan。地面阶段按地面所需端点启动，未受观测的飞行仍被拒绝。", "small"),
             paragraph("窗口与照射", "h"),
             paragraph("官方规则要求窗口先声明、识别正确后下降穿越；未规定固定下降距离。工程按真实下降、窗口几何、机体净空与定位容差校验，不能把本地演示尺寸称作官方门槛。空中任务要求先识别再照射，规则给出稳定照射 1-3 秒、飞行高度不低于 0.8 m；有效命中和反馈语义仍须实测。"),
             PageBreak(), paragraph("2. 官方已确认的接口边界", "h"),
             paragraph("依据官方《裁判系统判分细则 V1.0》表 1、表 5 与测评注意事项。规则为暂行版本；下表中的文档确认不代表消息已在本队 ROS 节点和裁判之间实际交付。"),
             table([["用途", "话题 / 类型", "已确认内容与剩余边界"],
                    ["地面订单", "/referee/ground_tasks\nstd_msgs/msg/String", order_text],
                    ["识别报告", "/inspection/detections\nstd_msgs/msg/String", "data 使用表 5 中文话术。类别须来自真实相机；不能轮流试探类别。"],
                    ["进入声明", "/referee/current_target\nstd_msgs/msg/String", "真实进入后发布 ground_X；交付与裁判接受仍待实测。"],
                    ["窗口 / 降落", "/referee/cruise_event\nstd_msgs/msg/String", "识别前 window_1/window_2；降落前 landing_marker。"],
                    ["灯光命令", "/uav/laser_fire\nstd_msgs/msg/Bool", "正确识别后开启、离开关闭；命中反馈与几何仍未确认。"],
                    ["公开位姿 / 控制", "/odom、/uav/odom、/tf\n/uav/cmd_vel", "前者来自测评须知；后者名称有规则依据。控制类型、帧、单位、起降仍需真实合同。"]], [27, 65, 80]),
             paragraph("识别确认不是已接通的接口", "h"),
             paragraph("测评须知列有 /referee/recognition_status；2026-11-11 本队只读探针已发现类型为 std_msgs/msg/String，但实际 data、接受值和关联语义仍未核实。内部 event_id / UUID 是工程关联手段；官方中文报告未证明可携带该 UUID，更未证明裁判会回传。transport_only 明确跳过 ACK 订阅并保留“裁判接受未知”，只有真实逐项计分和可见运行观察才能确认平台效果。"),
             paragraph("内部视觉结果使用 /inspection_mission/observations；官方 /inspection/detections 只发布中文话术，避免混用。"
                       + laser_text + "/inspection/mission_status 与 mission_completed 也不能据论坛文本直接当作官方终态合同。", "small"),
             PageBreak(), paragraph("3. 当前验证记录", "h"),
             table([["证据层级", "本次文档采用的记录"], ["自动化测试", tests],
                    ["离线运动学", scenario_text + "；人工场景，不计官方成功率。"],
                    ["平台启动与传感器", probe_text],
                    ["构建就绪性", (f"configuration_ready={validation['readiness'].get('configuration_ready')}；"
                                 f"platform_run_verified={validation['readiness'].get('platform_run_verified')}；"
                                 "原始构建记录保留在 manifest。" if validation['readiness'] else "未找到构建记录")],
                    ["官方完整任务", "官方任务完成率：" + completion],
                    ["真实录像", "官方完整运行录像：" + video_status]], [38, 134])]
    if probe:
        camera_rows = [["真实公开数据", "本次代表样本与限制"], ["任务顺序", order_text]]
        for topic in ("/camera/image", "/uav/camera/image"):
            message = messages.get(topic)
            if message:
                camera_rows.append([topic, f"{message.get('width')}×{message.get('height')}；"
                                    f"{message.get('encoding')}；frame={message.get('header', {}).get('frame_id')}。"
                                    "光学到机体外参仍需证明。"])
        for topic in ("/camera/camera_info", "/uav/camera/camera_info"):
            info = messages.get(topic, {})
            intrinsics = info.get("k")
            if isinstance(intrinsics, list) and len(intrinsics) == 9:
                camera_rows.append(["空中相机内参" if topic.startswith("/uav/") else "地面相机内参",
                                    f"fx={intrinsics[0]:.6g}；fy={intrinsics[4]:.6g}；"
                                    f"cx={intrinsics[2]:.6g}；cy={intrinsics[5]:.6g}。"
                                    "内参收到，不代表光学外参或目标尺寸已标定。"])
        scan = messages.get("/uav/laser/scan", {})
        if scan:
            camera_rows.append(["UAV测距样本", f"frame={scan.get('header', {}).get('frame_id')}；"
                                f"角域=[{scan.get('angle_min'):.6g},{scan.get('angle_max'):.6g}]；"
                                f"ranges={scan.get('ranges')}。无效读数不作为高度或空间净空证明。"])
        cloud = messages.get("/uav/lidar/points", {})
        if cloud:
            camera_rows.append(["UAV点云样本", f"frame={cloud.get('header', {}).get('frame_id')}；"
                                f"width={cloud.get('width')}；point_step={cloud.get('point_step')}。"
                                "诊断只保留数据短前缀，不能重建完整点云或验证通道净空。"])
        camera_rows.append(["灯光反馈", laser_text])
        if probe["reported_sample_counts"]:
            counts_text = "；".join(f"{topic}={count}" for topic, count in sorted(probe["reported_sample_counts"].items()))
            camera_rows.append(["快照累计回调", counts_text + "。与保留的代表样本数分开，不按采样墙钟时长推断仿真频率。"])
        story += [paragraph("公开传感器实收", "h"), table(camera_rows, [38, 134])]
    if probe and probe["images"]:
        story += [PageBreak(), paragraph("相机诊断图像", "h"),
                  paragraph("以下为本队探针从真实相机源图生成的缩小诊断 JPEG。分析器保存 JPEG 原始字节，不重编码。"
                            "这两张静态图不构成完整录像、七类识别准确率、穿窗效果、照射命中或裁判得分证据。", "callout")]
        for item in probe["images"]:
            image_path = next(Path(value["path"]) for value in validation["inputs"]
                              if value["sha256"] == item["sha256"])
            story += [Image(str(image_path), width=65 * mm, height=65 * mm),
                      paragraph(f"{item.get('topic')}；诊断图 {item.get('width')}×{item.get('height')}；"
                                f"frame={item.get('frame_id')}；源图时间戳={item.get('stamp')}。", "small"),
                      paragraph(f"SHA256: {item['sha256']}", "small"), Spacer(1, 4 * mm)]
    if probe and probe["images"]:
        story.append(PageBreak())
    story += [paragraph("证据索引与可重建性", "h"),
              paragraph("文档旁的 .manifest.json 保存完整输入绝对路径、SHA256、验证记录和生成时刻。"
                        "测试数量来自指定 JUnit 的实际 testcase；有失败和跳过时按原值保留。"
                        "回调、已匹配的既有摘要、诊断台账和 JPEG 分别保留证据哈希。未提供的检查不标为通过。", "small")]
    inputs = list(validation["inputs"])
    if junit:
        inputs.append(junit["evidence"])
    if local and local.get("evidence"):
        inputs.append(local["evidence"])
    for evidence in inputs:
        evidence_path = Path(evidence["path"])
        try:
            display_path = evidence_path.relative_to(ROOT).as_posix()
        except ValueError:
            display_path = evidence_path.name
        story.append(paragraph(f"{display_path}\nSHA256: {evidence['sha256']}", "small"))
    if official:
        story.append(paragraph("真实录像元数据：" + json.dumps(official["video"], ensure_ascii=False), "small"))
    story += [PageBreak(), paragraph("4. 平台验收待完成项", "h"),
              paragraph("官方 2026-11-10 审核答复说明，工作空间文件位于官方百度网盘链接中，测评侧构建和运行日志不展示。现阶段应依靠裁判有效逐项计分与允许的可见运行观察验证，不能继续把日志不可见视为等待平台开放的功能。仅修改 verified=true 不能替代真实验证。"),
              paragraph("仍需核实公开候选与官方环境的版本对应，验证相机/控制/传感器合同、识别流程、灯光命中及正常结算；还需补齐垂直飞行方向的真实观测方案。当前候选前向点云模式只支持有观测的方向，不能据此宣称全任务可飞行。"),
              paragraph("完成次序：绑定公开资料和真实观测；验证单点识别与进入声明；逐项完成窗口、空中照射和降落；记录裁判有效结算；拍摄完整任务录像，再按真实逐局台账统计重复运行。当前不提供官方得分、最低分或 95% 成功率。"),
              paragraph("来源见本页证据索引及随附 manifest；测试数量只取本次指定 JUnit。官方规则入口：https://www.splenach.com/api/courses/129d6e75-f19a-4c97-9ec6-0853695a4c48", "small")]
    def footer(canvas, document):
        canvas.saveState()
        canvas.setStrokeColor(colors.HexColor("#bfd0da"))
        canvas.line(18 * mm, 13 * mm, 190 * mm, 13 * mm)
        canvas.setFont("MSYH", 8)
        canvas.setFillColor(colors.HexColor("#617d90"))
        canvas.drawString(18 * mm, 8 * mm, "城市应急全域巡检 | " + label)
        canvas.drawRightString(190 * mm, 8 * mm, str(document.page))
        canvas.restoreState()
    document = SimpleDocTemplate(str(path), pagesize=A4, leftMargin=19 * mm, rightMargin=19 * mm,
                                 topMargin=16 * mm, bottomMargin=21 * mm,
                                 title="城市应急全域巡检 - " + label, author="参赛队伍")
    document.build(story, onFirstPage=footer, onLaterPages=footer)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("draft", "official"), default="draft")
    parser.add_argument("--pdf", type=Path, help="output PDF; draft is the default mode")
    parser.add_argument("--junit", type=Path, help="current pytest JUnit XML; no count is assumed")
    parser.add_argument("--validation-record", type=Path, help="schema 1 local_validation JSON")
    parser.add_argument("--probe-evidence", type=Path, help="raw local read-only callback JSONL; does not prove a scored mission")
    parser.add_argument("--probe-summary", type=Path, help="optional existing analysis; must match the raw callback")
    parser.add_argument("--probe-run-record", type=Path, help="diagnostic ledger referencing the same callback; remains draft evidence")
    parser.add_argument("--run-local-scenario", action="store_true", help="run the artificial inspection_v1 scenario now")
    parser.add_argument("--run-record", type=Path, action="append", default=[], help="real official run ledger JSON")
    parser.add_argument("--video-input", type=Path, help="original platform MP4 listed in run evidence")
    args = parser.parse_args(argv)
    try:
        if args.mode == "draft" and (args.run_record or args.video_input):
            raise ValueError("use --mode official to validate platform run/video inputs")
        if args.validation_record and args.run_local_scenario:
            raise ValueError("choose --validation-record or --run-local-scenario")
        official = validate_official_inputs(args.run_record, args.video_input) if args.mode == "official" else None
        default_name = "城市应急全域巡检技术方案-草稿.pdf" if official is None else "城市应急全域巡检技术方案-平台验证版.pdf"
        output = (args.pdf or ROOT / "submission" / default_name).resolve()
        if output.suffix.lower() != ".pdf":
            raise ValueError("--pdf output must use the .pdf extension")
        validation = collect_validation(junit=args.junit, validation_record=args.validation_record,
                                        run_local_scenario=args.run_local_scenario,
                                        probe_evidence=args.probe_evidence,
                                        probe_summary=args.probe_summary,
                                        probe_run_record=args.probe_run_record,
                                        probe_output=output.with_suffix(".probe"))
        output.parent.mkdir(parents=True, exist_ok=True)
        flow = output.with_suffix(".flow.png")
        make_flow(flow)
        make_pdf(output, flow, validation, official=official)
        manifest = output.with_suffix(".manifest.json")
        record = {"schema_version": 1, "kind": "submission_materials", "mode": args.mode,
                  "validation": validation, "official": official, "pdf": file_evidence(output),
                  "generator": file_evidence(Path(__file__)), "flow": file_evidence(flow),
                  "note": "No animation or platform recording is generated by this tool."}
        manifest.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"pdf": str(output), "manifest": str(manifest), "mode": args.mode}, ensure_ascii=False))
        return 0
    except (ValueError, OSError, ET.ParseError, subprocess.SubprocessError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
