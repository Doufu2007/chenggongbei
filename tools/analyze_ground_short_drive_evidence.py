"""Reassemble real ground_short_drive evidence, without changing its mode.

The shared observation transport retains its survey_* section names. Source
bytes, lifecycle, DRIVE states, metadata, image headers and scoring uncertainty
remain untouched. This wrapper selects only the expected mode and output kind.
"""
from __future__ import annotations

try:
    from . import analyze_ground_survey_evidence as observations
except ImportError:
    import analyze_ground_survey_evidence as observations


MODE = "ground_short_drive"


def analyze_file(input_path, output_dir):
    return observations.analyze_file(input_path, output_dir, expected_mode=MODE)


analyze_evidence = analyze_file


def main(argv=None):
    return observations.main(argv, expected_mode=MODE)


if __name__ == "__main__":
    raise SystemExit(main())
