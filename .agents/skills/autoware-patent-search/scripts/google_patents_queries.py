#!/usr/bin/env python3
"""Compatibility wrapper that emits Google Patents search URLs for Autoware IP triage."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from patent_cross_search_queries import build_google_queries  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--feature", required=True, help="English feature phrase to search")
    parser.add_argument("--jp", help="Japanese feature phrase to search")
    parser.add_argument("--cpc", action="append", default=[], help="CPC code hint, repeatable")
    parser.add_argument("--assignee", action="append", default=[], help="Optional assignee filter, repeatable")
    args = parser.parse_args()

    for plan in build_google_queries(args.feature, args.jp, args.cpc, args.assignee):
        print(plan.query)
        print(plan.url)
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
