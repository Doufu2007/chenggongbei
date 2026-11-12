"""Reassemble explicitly identified ground route observations without relabelling.

Route metadata, four source scan records and capture phases remain observations.
Only this explicit mode permits the larger bounded transport series. This tool
does not infer control acceptance, recognition, route completion or scoring.
"""
from __future__ import annotations

try:
    from . import analyze_ground_survey_evidence as observations
except ImportError:
    import analyze_ground_survey_evidence as observations


MODE = "ground_observation_route"


def analyze_file(input_path, output_dir):
    return observations.analyze_file(input_path, output_dir, expected_mode=MODE)


analyze_evidence = analyze_file


def main(argv=None):
    return observations.main(argv, expected_mode=MODE)


if __name__ == "__main__":
    raise SystemExit(main())
