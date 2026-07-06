#!/usr/bin/env python3
"""Create non-legal patent review scaffolds for Autoware patent triage.

The generated Markdown is for IP-professional review. It does not decide
infringement, validity, freedom to operate, or non-infringement.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Candidate:
    database: str
    publication: str
    assignee: str
    title: str
    url: str
    relevance: str


def load_candidates(path: Path) -> list[Candidate]:
    with path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {"database", "publication", "assignee", "title", "url", "relevance"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing required CSV columns: {', '.join(sorted(missing))}")
        return [Candidate(**{key: row.get(key, "") for key in required}) for row in reader]


def candidate_table(candidates: list[Candidate]) -> str:
    lines = [
        "| Database | Publication | Assignee | Title | Why it may be relevant | URL |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for candidate in candidates:
        lines.append(
            "| "
            + " | ".join(
                [
                    candidate.database,
                    candidate.publication,
                    candidate.assignee,
                    candidate.title,
                    candidate.relevance,
                    candidate.url,
                ]
            )
            + " |"
        )
    return "\n".join(lines)


def render_report(feature: str, candidates: list[Candidate]) -> str:
    sections = [
        "# Autoware Patent Candidate Review Scaffold",
        "",
        "> Non-legal technical triage only. This scaffold does not conclude infringement,",
        "> non-infringement, validity, invalidity, or freedom to operate.",
        "",
        "## Feature under review",
        "",
        feature,
        "",
        "## Candidate patent documents",
        "",
        candidate_table(candidates),
        "",
        "## Claim-comparison worksheet for IP-professional review",
        "",
        "| Candidate publication | Claim element | Autoware evidence to compare | Technical similarity notes | Difference / uncertainty | Reviewer disposition |",
        "| --- | --- | --- | --- | --- | --- |",
        "| TBD | TBD | File/function/topic/parameter/message | TBD | TBD | Needs IP review |",
        "",
        "## Monitoring plan",
        "",
        "- Re-run the saved search queries on a scheduled cadence.",
        "- Alert only when a newly found publication/application matches the saved feature keywords, assignee filters, or classifications.",
        "- Include database, query, publication/application number, assignee, title, URL, and reason for alert.",
        "- Route alerts to a human reviewer; do not auto-label them with a legal conclusion.",
    ]
    return "\n".join(sections) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--feature", required=True, help="Autoware feature under review")
    parser.add_argument("--candidates-csv", required=True, type=Path, help="CSV with database,publication,assignee,title,url,relevance columns")
    parser.add_argument("--output", type=Path, help="Markdown output path. Defaults to stdout.")
    args = parser.parse_args()

    report = render_report(args.feature, load_candidates(args.candidates_csv))
    if args.output:
        args.output.write_text(report, encoding="utf-8")
    else:
        print(report, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
