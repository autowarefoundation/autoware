#!/usr/bin/env python3
"""Generate Google Patents URLs and J-PlatPat query plans for Autoware IP triage."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from urllib.parse import quote_plus


DEFAULT_CONTEXT = [
    '"autonomous vehicle"',
    '"automated driving"',
    '"trajectory planning"',
    '"vehicle control"',
]

JPLATPAT_URL = "https://www.j-platpat.inpit.go.jp/"


@dataclass(frozen=True)
class QueryPlan:
    database: str
    query: str
    url: str | None = None
    note: str | None = None


def google_url_for(query: str) -> str:
    return f"https://patents.google.com/?q=({quote_plus(query)})"


def build_google_queries(feature: str, jp: str | None, cpc: list[str], assignee: list[str]) -> list[QueryPlan]:
    queries: list[str] = [feature]
    for context in DEFAULT_CONTEXT:
        queries.append(f"{feature} {context}")
    if jp:
        queries.extend([jp, f"{jp} 自動運転"])
    for code in cpc:
        queries.append(f"{feature} CPC={code}")
    for name in assignee:
        queries.append(f'{feature} assignee="{name}"')
        if jp:
            queries.append(f'{jp} assignee="{name}"')
    return [QueryPlan("Google Patents", query, google_url_for(query)) for query in queries]


def build_jplatpat_queries(feature: str, jp: str | None, cpc: list[str], assignee: list[str]) -> list[QueryPlan]:
    base_terms = [term for term in [jp, f"{jp} 自動運転" if jp else None, feature] if term]
    queries: list[str] = list(base_terms)
    for name in assignee:
        for term in base_terms:
            queries.append(f"{term} / 出願人・権利者: {name}")
    for code in cpc:
        queries.append(f"{jp or feature} / 分類: {code}")
    note = "Enter these terms in J-PlatPat patent/utility model search; record filters and result identifiers because stable deep links may be session-dependent."
    return [QueryPlan("J-PlatPat", query, JPLATPAT_URL, note) for query in queries]


def build_queries(feature: str, jp: str | None, cpc: list[str], assignee: list[str], databases: list[str]) -> list[QueryPlan]:
    plans: list[QueryPlan] = []
    selected = {database.lower() for database in databases}
    if "google" in selected or "google-patent" in selected or "google-patents" in selected:
        plans.extend(build_google_queries(feature, jp, cpc, assignee))
    if "jplatpat" in selected or "j-platpat" in selected:
        plans.extend(build_jplatpat_queries(feature, jp, cpc, assignee))
    return plans


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--feature", required=True, help="English feature phrase to search")
    parser.add_argument("--jp", help="Japanese feature phrase to search")
    parser.add_argument("--cpc", action="append", default=[], help="CPC/IPC/FI classification hint, repeatable")
    parser.add_argument("--assignee", action="append", default=[], help="Optional assignee/company filter, repeatable")
    parser.add_argument(
        "--database",
        action="append",
        choices=["google", "google-patent", "google-patents", "jplatpat", "j-platpat"],
        default=None,
        help="Target database to include, repeatable. Defaults to the only supported targets: Google Patents and J-PlatPat.",
    )
    args = parser.parse_args()

    databases = args.database or ["google", "jplatpat"]
    for plan in build_queries(args.feature, args.jp, args.cpc, args.assignee, databases):
        print(f"Database: {plan.database}")
        print(f"Query: {plan.query}")
        if plan.url:
            print(f"URL: {plan.url}")
        if plan.note:
            print(f"Note: {plan.note}")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
