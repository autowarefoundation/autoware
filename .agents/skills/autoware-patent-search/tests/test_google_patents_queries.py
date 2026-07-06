import sys
from pathlib import Path
from urllib.parse import unquote_plus, urlparse, parse_qs

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from patent_cross_search_queries import build_google_queries, google_assignee_filter, google_cpc_filter


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


def test_google_query_urls_apply_metadata_and_classification_filters():
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
