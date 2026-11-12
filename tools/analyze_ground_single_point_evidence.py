"""Reassemble explicit single-point evidence without inferring referee acceptance.

The original source mode, native bytes, prior poses, match diagnostics, one-shot
attempts and raw recognition-status text remain observations with no score.
"""
from __future__ import annotations

try:
    from . import analyze_ground_survey_evidence as observations
except ImportError:
    import analyze_ground_survey_evidence as observations


MODE = "ground_single_point"


def analyze_file(input_path, output_dir):
    return observations.analyze_file(input_path, output_dir, expected_mode=MODE)


analyze_evidence = analyze_file


def main(argv=None):
    return observations.main(argv, expected_mode=MODE)


if __name__ == "__main__":
    raise SystemExit(main())
