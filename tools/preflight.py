"""Static checks matching the public inspection evaluator rules."""

from __future__ import annotations

import ast
from dataclasses import dataclass, field
from pathlib import Path


FORBIDDEN = (
    "gzserver",
    "gzclient",
    "gazebo",
    "ign gazebo",
    "ros2 launch ican_",
    "ros2 run ican_",
    "ican_land_sim",
    "ican_shoot_sim",
    "ican_delivery_sim",
    "ican_inspection_sim",
    "ican_referee_system",
    "ican_delivery_referee",
    "ican_inspection_referee",
    "apt install",
    "apt-get",
    "dpkg",
    ".deb",
)
MAX_SRC_ZIP_BYTES = 20 * 1024 * 1024


@dataclass
class PreflightReport:
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def _forbidden_access(text: str) -> bool:
    """Catch literal interface/file accesses without flagging a reject list.

    This is a limited lint, not a proof that arbitrary Python is compliant;
    dynamic interface configuration is additionally checked at runtime.
    """
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = node.func.attr if isinstance(node.func, ast.Attribute) else node.func.id if isinstance(node.func, ast.Name) else ""
        if name not in {"create_subscription", "create_client", "open", "Path", "read_text", "read_bytes"}:
            continue
        for arg in node.args + [kw.value for kw in node.keywords]:
            if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                value = arg.value.lower()
                if "/gazebo/" in value or value in ("/referee/events", "/referee/delivery_order") or "randomization" in value or "randomized_result" in value:
                    return True
    return False


def run_preflight(root: Path) -> PreflightReport:
    report = PreflightReport()
    package_xml = root / "src" / "inspection_mission" / "package.xml"
    script = root / "scripts" / "run_inspection.sh"
    if not package_xml.exists():
        report.errors.append("missing src/inspection_mission/package.xml")
    if not script.exists():
        report.errors.append("missing scripts/run_inspection.sh")
    if script.exists():
        script_text = _read(script).lower()
        for token in FORBIDDEN:
            if token.lower() in script_text:
                report.errors.append(f"forbidden token in startup script: {token}")
        if "ros2 run inspection_mission mission_node" not in script_text:
            report.errors.append("startup script does not launch mission_node")
        if "ros2 run inspection_mission vision_node" not in script_text:
            report.errors.append("startup script does not launch vision_node")
    src_dir = root / "src"
    if src_dir.exists():
        for path in src_dir.rglob("*"):
            if path.is_file():
                if "__pycache__" in path.parts or path.suffix in (".pyc", ".pyo"):
                    continue
                lowered = path.name.lower()
                if lowered.endswith((".pt", ".onnx", ".engine", ".deb")):
                    report.errors.append(f"large/binary model artifact in src: {path.relative_to(root)}")
                if path.suffix == ".py":
                    try:
                        if _forbidden_access(_read(path)):
                            report.errors.append(f"forbidden interface/file access in source: {path.relative_to(root)}")
                    except SyntaxError:
                        report.errors.append(f"invalid Python source: {path.relative_to(root)}")
    zip_path = root / "dist" / "inspection_src.zip"
    if zip_path.exists() and zip_path.stat().st_size > MAX_SRC_ZIP_BYTES:
        report.errors.append(f"source ZIP exceeds 20MB: {zip_path.stat().st_size} bytes")
    if not (root / "README.md").exists():
        report.warnings.append("README.md is missing")
    return report


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("root", nargs="?", default=".")
    args = parser.parse_args()
    report = run_preflight(Path(args.root).resolve())
    for warning in report.warnings:
        print(f"WARN: {warning}")
    for error in report.errors:
        print(f"ERROR: {error}")
    print("PASS" if report.ok else "FAIL")
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

