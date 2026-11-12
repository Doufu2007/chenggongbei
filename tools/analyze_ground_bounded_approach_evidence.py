"""Reassemble real bounded-approach observations without changing source mode.

Approach metadata, scan records and capture phases remain source observations.
This selects only the explicit expected mode and its bounded transport limits;
it does not infer target recognition, geometry, accepted control or scoring.
"""
from __future__ import annotations

try:
    from . import analyze_ground_survey_evidence as observations
except ImportError:
    import analyze_ground_survey_evidence as observations


MODE = "ground_bounded_approach"


def analyze_file(input_path, output_dir):
    return observations.analyze_file(input_path, output_dir, expected_mode=MODE)


analyze_evidence = analyze_file


def main(argv=None):
    return observations.main(argv, expected_mode=MODE)


if __name__ == "__main__":
    raise SystemExit(main())
