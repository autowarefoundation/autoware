import sys
from pathlib import Path
from urllib.parse import parse_qs, unquote_plus, urlparse

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))


from patent_cross_search_queries import (  # noqa: E402
    build_bigquery_sql,
    build_google_queries,
    build_queries,
    google_assignee_filter,
    google_cpc_filter,
)


def decoded_query(url: str) -> str:
    raw = parse_qs(urlparse(url).query)["q"][0]
    return unquote_plus(raw)


def test_google_cpc_filter_uses_metadata_for_broad_subclass():
    assert google_cpc_filter("B60W") == "cpc:B60W"


def test_google_cpc_filter_uses_low_suffix_for_specific_groups():
    assert google_cpc_filter("G05D1/00") == "CPC=G05D1/00/low"


def test_google_assignee_filter_uses_metadata_prefix():
    assert google_assignee_filter("Toyota") == "assignee:Toyota"
    assert google_assignee_filter("Toyota Motor") == 'assignee:"Toyota Motor"'


def test_optional_google_query_urls_apply_metadata_and_classification_filters():
    plans = build_google_queries(
        "lane change path generation with collision check using predicted objects",
        "車線変更 経路生成 衝突判定 予測物体",
        ["B60W", "G05D1/00", "G08G1/00"],
        ["Toyota", "Denso", "Waymo"],
    )
    decoded_queries = [decoded_query(plan.url) for plan in plans if plan.url]

    assert "(lane change path generation with collision check using predicted objects cpc:B60W)" in decoded_queries
    assert "(lane change path generation with collision check using predicted objects CPC=G05D1/00/low)" in decoded_queries
    assert "(lane change path generation with collision check using predicted objects CPC=G08G1/00/low)" in decoded_queries
    assert "(lane change path generation with collision check using predicted objects assignee:Toyota)" in decoded_queries
    assert "(車線変更 経路生成 衝突判定 予測物体 assignee:Denso)" in decoded_queries


def test_build_bigquery_sql_searches_public_data_text_cpc_and_assignee_fields():
    sql = build_bigquery_sql(
        module="planning/behavior_path_planner/lane_change",
        feature="lane change path generation with collision check using predicted objects",
        keywords=["behavior planning"],
        jp="車線変更 経路生成 衝突判定 予測物体",
        cpc=["B60W", "G05D1/00"],
        assignee=["Toyota Motor"],
        publication_after="2018-01-01",
        limit=25,
    )

    assert "patents-public-data.patents.publications" in sql
    assert "planning/behavior_path_planner/lane_change" in sql
    assert "title_localized" in sql
    assert "abstract_localized" in sql
    assert "description_localized" in sql
    assert "claims_localized" in sql
    assert "STARTS_WITH(c.code, 'B60W')" in sql
    assert "STARTS_WITH(c.code, 'G05D1/00')" in sql
    assert "LOWER(a.name) LIKE '%toyota motor%'" in sql
    assert "p.publication_date >= 20180101" in sql
    assert "LIMIT 25" in sql


def test_build_queries_is_bigquery_first_with_optional_review_urls():
    plan = build_queries(
        module="planning/behavior_path_planner/lane_change",
        feature="lane change path generation with collision check using predicted objects",
        keywords=["behavior planning"],
        jp="車線変更 経路生成 衝突判定 予測物体",
        cpc=["B60W"],
        assignee=["Toyota"],
        include_urls=True,
    )

    assert plan.source == "Google Patents Public Data via BigQuery"
    assert "FROM `patents-public-data.patents.publications` AS p" in plan.sql
    assert plan.human_review_urls
    assert all(url.startswith("https://patents.google.com/?q=") for url in plan.human_review_urls)


def test_bigquery_cli_options_do_not_reintroduce_removed_database_paths():
    script = SCRIPT_DIR / "patent_cross_search_queries.py"
    help_text = script.read_text()
    forbidden_database_name = "J-" + "PlatPat"
    forbidden_cli_alias = "j" + "platpat"

    assert forbidden_database_name not in help_text
    assert forbidden_cli_alias not in help_text.lower()
    assert "--database" not in help_text
    assert "--module" in help_text
    assert "--include-review-urls" in help_text


def test_bigquery_where_requires_feature_terms_not_broad_context_alone():
    sql = build_bigquery_sql(
        module="planning/behavior_path_planner/lane_change",
        feature="lane change path generation",
        keywords=["collision check", "predicted object"],
        jp="車線変更",
        cpc=["B60W"],
        assignee=[],
        limit=10,
    )

    where_sql = sql.split("\nWHERE ", 1)[1].split("\nORDER BY", 1)[0]
    select_sql = sql.split("\nWHERE ", 1)[0]

    assert "lane change path generation" in where_sql
    assert "collision check" in where_sql
    assert "predicted object" in where_sql
    assert "車線変更" in where_sql
    assert "autonomous vehicle" not in where_sql
    assert "automated driving" not in where_sql
    assert "broad_context_match_count" in select_sql
    assert "autonomous vehicle" in select_sql
    assert "STARTS_WITH(c.code, 'B60W')" in where_sql