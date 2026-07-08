#!/usr/bin/env python3
"""Generate BigQuery SQL for Google Patents Public Data Autoware IP triage.

The filename is retained for compatibility with existing skill instructions. The
primary output is reproducible Standard SQL for the Google Patents Public Data
BigQuery dataset. Optional Google Patents search URLs are emitted only as
human-review links.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import re
from urllib.parse import quote_plus


GOOGLE_PATENTS_PUBLICATIONS_TABLE = "`patents-public-data.patents.publications`"
DEFAULT_CONTEXT = [
    "autonomous vehicle",
    "automated driving",
    "trajectory planning",
    "vehicle control",
]
TEXT_FIELDS = [
    "title_localized",
    "abstract_localized",
    "description_localized",
    "claims_localized",
]
CPC_BROAD_RE = re.compile(r"^[A-HY][0-9]{2}[A-Z]$", re.IGNORECASE)

CPC_BROAD_RE = re.compile(r"^[A-HY][0-9]{2}[A-Z]$", re.IGNORECASE)


def google_cpc_filter(code: str) -> str:
    """Return Google Patents CPC filter syntax for a classification hint.

    Broad subclasses (for example B60W) use Google Patents metadata-style
    ``cpc:`` syntax, which includes child classifications. More specific CPC
    groups use field syntax with ``/low`` so children remain included instead of
    matching only the exact classification.
    """
    normalized = code.strip().upper()
    if CPC_BROAD_RE.fullmatch(normalized):
        return f"cpc:{normalized}"
    return f"CPC={normalized}/low"


def google_assignee_filter(name: str) -> str:
    """Return Google Patents assignee metadata filter syntax."""
    cleaned = name.strip()
    if not cleaned:
        return ""
    if re.search(r"\s", cleaned):
        return f'assignee:"{cleaned}"'
    return f"assignee:{cleaned}"


@dataclass(frozen=True)
class QueryPlan:
    database: str
    query: str
    url: str


@dataclass(frozen=True)
class BigQueryPlan:
    source: str
    module: str
    sql: str
    human_review_urls: list[str]


def sql_string(value: str) -> str:
    """Return a BigQuery Standard SQL string literal."""
    return "'" + value.replace("'", "''") + "'"


def like_pattern(value: str) -> str:
    """Return an escaped LIKE contains pattern for BigQuery."""
    return "%" + value.lower().replace("%", r"\%").replace("_", r"\_") + "%"


def google_cpc_filter(code: str) -> str:
    """Return Google Patents URL CPC filter syntax for a classification hint.

    Broad subclasses (for example B60W) use Google Patents metadata-style
    ``cpc:`` syntax, which includes child classifications. More specific CPC
    groups use field syntax with ``/low`` so children remain included instead of
    matching only the exact classification.
    """
    normalized = code.strip().upper()
    if CPC_BROAD_RE.fullmatch(normalized):
        return f"cpc:{normalized}"
    return f"CPC={normalized}/low"


def google_assignee_filter(name: str) -> str:
    """Return Google Patents URL assignee metadata filter syntax."""
    cleaned = name.strip()
    if not cleaned:
        return ""
    if re.search(r"\s", cleaned):
        return f'assignee:"{cleaned}"'
    return f"assignee:{cleaned}"


def google_url_for(query: str) -> str:
    return f"https://patents.google.com/?q=({quote_plus(query)})"


def normalized_terms(*term_groups: str | None) -> list[str]:
    terms: list[str] = []
    for term in term_groups:
        if term and term.strip() and term.strip() not in terms:
            terms.append(term.strip())
    return terms


def feature_terms(feature: str, keywords: list[str], jp: str | None) -> list[str]:
    """Return terms that must drive candidate retrieval for the reviewed feature."""
    return normalized_terms(feature, *keywords, jp)


def context_terms() -> list[str]:
    """Return broad autonomous-driving context terms used only as support signals."""
    return normalized_terms(*DEFAULT_CONTEXT)


def text_term_condition(term: str) -> str:
    field_conditions = []
    pattern = sql_string(like_pattern(term))
    for field in TEXT_FIELDS:
        field_conditions.append(
            f"EXISTS (SELECT 1 FROM UNNEST(p.{field}) txt "
            f"WHERE LOWER(txt.text) LIKE {pattern})"
        )
    return "(" + " OR\n      ".join(field_conditions) + ")"


def text_query_condition(terms: list[str], indent: str = "    ") -> str:
    return "(\n" + indent + ("\n" + indent + "OR ").join(text_term_condition(term) for term in terms) + "\n  )"


def text_score_expression(terms: list[str]) -> str:
    if not terms:
        return "0"
    return " +\n  ".join(f"IF({text_term_condition(term)}, 1, 0)" for term in terms)


def cpc_condition(cpc: list[str]) -> str:
    codes = [code.strip().upper() for code in cpc if code.strip()]
    if not codes:
        return ""
    parts = [
        f"EXISTS (SELECT 1 FROM UNNEST(p.cpc) c WHERE STARTS_WITH(c.code, {sql_string(code)}))"
        for code in codes
    ]
    return "(" + " OR ".join(parts) + ")"


def assignee_condition(assignees: list[str]) -> str:
    names = [name.strip() for name in assignees if name.strip()]
    if not names:
        return ""
    parts = [
        "EXISTS (SELECT 1 FROM UNNEST(p.assignee_harmonized) a "
        f"WHERE LOWER(a.name) LIKE {sql_string(like_pattern(name))})"
        for name in names
    ]
    return "(" + " OR ".join(parts) + ")"


def date_condition(field: str, value: str | None, operator: str) -> str:
    if not value:
        return ""
    return f"p.{field} {operator} {value.replace('-', '')}"


def build_bigquery_sql(
    module: str,
    feature: str,
    keywords: list[str],
    jp: str | None,
    cpc: list[str],
    assignee: list[str],
    publication_after: str | None = None,
    publication_before: str | None = None,
    limit: int = 50,
) -> str:
    """Build a reproducible Google Patents Public Data SQL query."""
    required_feature_terms = feature_terms(feature, keywords, jp)
    if not required_feature_terms:
        raise ValueError("At least one feature-specific term is required via --feature, --keyword, or --jp")
    broad_context_terms = context_terms()
    where_clauses = [text_query_condition(required_feature_terms)]
    for optional_condition in [
        cpc_condition(cpc),
        assignee_condition(assignee),
        date_condition("publication_date", publication_after, ">="),
        date_condition("publication_date", publication_before, "<="),
    ]:
        if optional_condition:
            where_clauses.append(optional_condition)

    where_sql = "\n  AND ".join(where_clauses)
    cpc_select = "ARRAY(SELECT c.code FROM UNNEST(p.cpc) c LIMIT 10) AS cpc_codes"
    assignee_select = "ARRAY(SELECT a.name FROM UNNEST(p.assignee_harmonized) a LIMIT 10) AS assignees"
    return f"""-- Google Patents Public Data candidate search for Autoware module: {module}
-- Technical triage only; results are candidates for human IP-professional review.
SELECT
  p.publication_number,
  p.application_number,
  p.publication_date,
  p.filing_date,
  (SELECT text FROM UNNEST(p.title_localized) LIMIT 1) AS title,
  (SELECT text FROM UNNEST(p.abstract_localized) LIMIT 1) AS abstract,
  {assignee_select},
  {cpc_select},
  ({text_score_expression(broad_context_terms)}) AS broad_context_match_count,
  CONCAT('https://patents.google.com/patent/', p.publication_number) AS google_patents_url
FROM {GOOGLE_PATENTS_PUBLICATIONS_TABLE} AS p
WHERE {where_sql}
ORDER BY broad_context_match_count DESC, p.publication_date DESC
LIMIT {limit};"""


def build_google_queries(feature: str, jp: str | None, cpc: list[str], assignee: list[str]) -> list[QueryPlan]:
    """Build optional Google Patents URL queries for human review links."""
    queries: list[str] = [feature]
    for context in DEFAULT_CONTEXT:
        queries.append(f"{feature} {context}")
    if jp:
        queries.extend([jp, f"{jp} 自動運転"])
    for code in cpc:
        cpc_filter = google_cpc_filter(code)
        queries.append(f"{feature} {cpc_filter}")
    for name in assignee:
        assignee_filter = google_assignee_filter(name)
        if not assignee_filter:
            continue
        queries.append(f"{feature} {assignee_filter}")
        if jp:
            queries.append(f"{jp} {assignee_filter}")
    return [QueryPlan("Google Patents", query, google_url_for(query)) for query in queries]


def build_queries(
    module: str,
    feature: str,
    keywords: list[str],
    jp: str | None,
    cpc: list[str],
    assignee: list[str],
    publication_after: str | None = None,
    publication_before: str | None = None,
    limit: int = 50,
    include_urls: bool = False,
) -> BigQueryPlan:
    """Build the Google Patents Public Data query plan for a module."""
    sql = build_bigquery_sql(
        module,
        feature,
        keywords,
        jp,
        cpc,
        assignee,
        publication_after,
        publication_before,
        limit,
    )
    urls = [plan.url for plan in build_google_queries(feature, jp, cpc, assignee)] if include_urls else []
    return BigQueryPlan("Google Patents Public Data via BigQuery", module, sql, urls)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module", required=True, help="Autoware module/package/directory/component being reviewed")
    parser.add_argument("--feature", required=True, help="English technical feature phrase extracted from the module")
    parser.add_argument("--keyword", action="append", default=[], help="Additional English technical keyword, repeatable")
    parser.add_argument("--jp", help="Japanese technical keyword phrase to search")
    parser.add_argument("--cpc", action="append", default=[], help="CPC/IPC classification hint, repeatable")
    parser.add_argument("--assignee", action="append", default=[], help="Optional assignee/company filter, repeatable")
    parser.add_argument("--publication-after", help="Optional publication_date lower bound, YYYY-MM-DD")
    parser.add_argument("--publication-before", help="Optional publication_date upper bound, YYYY-MM-DD")
    parser.add_argument("--limit", type=int, default=50, help="Maximum BigQuery rows to return")
    parser.add_argument(
        "--include-review-urls",
        action="store_true",
        help="Also print optional Google Patents search URLs for human review; BigQuery SQL remains primary.",
    )
    args = parser.parse_args()

    plan = build_queries(
        args.module,
        args.feature,
        args.keyword,
        args.jp,
        args.cpc,
        args.assignee,
        args.publication_after,
        args.publication_before,
        args.limit,
        args.include_review_urls,
    )
    print(f"Source: {plan.source}")
    print(f"Module: {plan.module}")
    print("SQL:")
    print(plan.sql)
    if plan.human_review_urls:
        print("\nOptional Google Patents human-review URLs:")
        for url in plan.human_review_urls:
            print(url)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
