-- Google Patents Public Data candidate search for Autoware module: planning/behavior_path_planner/lane_change
-- Technical triage only; results are candidates for human IP-professional review.
SELECT
  p.publication_number,
  p.application_number,
  p.publication_date,
  p.filing_date,
  (SELECT text FROM UNNEST(p.title_localized) LIMIT 1) AS title,
  (SELECT text FROM UNNEST(p.abstract_localized) LIMIT 1) AS abstract,
  ARRAY(SELECT a.name FROM UNNEST(p.assignee_harmonized) a LIMIT 10) AS assignees,
  ARRAY(SELECT c.code FROM UNNEST(p.cpc) c LIMIT 10) AS cpc_codes,
  (IF((EXISTS (SELECT 1 FROM UNNEST(p.title_localized) txt WHERE LOWER(txt.text) LIKE '%autonomous vehicle%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.abstract_localized) txt WHERE LOWER(txt.text) LIKE '%autonomous vehicle%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.description_localized) txt WHERE LOWER(txt.text) LIKE '%autonomous vehicle%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.claims_localized) txt WHERE LOWER(txt.text) LIKE '%autonomous vehicle%')), 1, 0) +
  IF((EXISTS (SELECT 1 FROM UNNEST(p.title_localized) txt WHERE LOWER(txt.text) LIKE '%automated driving%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.abstract_localized) txt WHERE LOWER(txt.text) LIKE '%automated driving%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.description_localized) txt WHERE LOWER(txt.text) LIKE '%automated driving%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.claims_localized) txt WHERE LOWER(txt.text) LIKE '%automated driving%')), 1, 0) +
  IF((EXISTS (SELECT 1 FROM UNNEST(p.title_localized) txt WHERE LOWER(txt.text) LIKE '%trajectory planning%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.abstract_localized) txt WHERE LOWER(txt.text) LIKE '%trajectory planning%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.description_localized) txt WHERE LOWER(txt.text) LIKE '%trajectory planning%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.claims_localized) txt WHERE LOWER(txt.text) LIKE '%trajectory planning%')), 1, 0) +
  IF((EXISTS (SELECT 1 FROM UNNEST(p.title_localized) txt WHERE LOWER(txt.text) LIKE '%vehicle control%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.abstract_localized) txt WHERE LOWER(txt.text) LIKE '%vehicle control%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.description_localized) txt WHERE LOWER(txt.text) LIKE '%vehicle control%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.claims_localized) txt WHERE LOWER(txt.text) LIKE '%vehicle control%')), 1, 0)) AS broad_context_match_count,
  CONCAT('https://patents.google.com/patent/', p.publication_number) AS google_patents_url
FROM `patents-public-data.patents.publications` AS p
WHERE (
    (EXISTS (SELECT 1 FROM UNNEST(p.title_localized) txt WHERE LOWER(txt.text) LIKE '%lane change path generation with collision check using predicted objects%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.abstract_localized) txt WHERE LOWER(txt.text) LIKE '%lane change path generation with collision check using predicted objects%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.description_localized) txt WHERE LOWER(txt.text) LIKE '%lane change path generation with collision check using predicted objects%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.claims_localized) txt WHERE LOWER(txt.text) LIKE '%lane change path generation with collision check using predicted objects%'))
    OR (EXISTS (SELECT 1 FROM UNNEST(p.title_localized) txt WHERE LOWER(txt.text) LIKE '%behavior planning%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.abstract_localized) txt WHERE LOWER(txt.text) LIKE '%behavior planning%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.description_localized) txt WHERE LOWER(txt.text) LIKE '%behavior planning%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.claims_localized) txt WHERE LOWER(txt.text) LIKE '%behavior planning%'))
    OR (EXISTS (SELECT 1 FROM UNNEST(p.title_localized) txt WHERE LOWER(txt.text) LIKE '%safety margin%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.abstract_localized) txt WHERE LOWER(txt.text) LIKE '%safety margin%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.description_localized) txt WHERE LOWER(txt.text) LIKE '%safety margin%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.claims_localized) txt WHERE LOWER(txt.text) LIKE '%safety margin%'))
    OR (EXISTS (SELECT 1 FROM UNNEST(p.title_localized) txt WHERE LOWER(txt.text) LIKE '%車線変更 経路生成 衝突判定 予測物体%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.abstract_localized) txt WHERE LOWER(txt.text) LIKE '%車線変更 経路生成 衝突判定 予測物体%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.description_localized) txt WHERE LOWER(txt.text) LIKE '%車線変更 経路生成 衝突判定 予測物体%') OR
      EXISTS (SELECT 1 FROM UNNEST(p.claims_localized) txt WHERE LOWER(txt.text) LIKE '%車線変更 経路生成 衝突判定 予測物体%'))
  )
  AND (EXISTS (SELECT 1 FROM UNNEST(p.cpc) c WHERE STARTS_WITH(c.code, 'B60W')) OR EXISTS (SELECT 1 FROM UNNEST(p.cpc) c WHERE STARTS_WITH(c.code, 'G05D1/00')) OR EXISTS (SELECT 1 FROM UNNEST(p.cpc) c WHERE STARTS_WITH(c.code, 'G08G1/00')))
  AND (EXISTS (SELECT 1 FROM UNNEST(p.assignee_harmonized) a WHERE LOWER(a.name) LIKE '%toyota%') OR EXISTS (SELECT 1 FROM UNNEST(p.assignee_harmonized) a WHERE LOWER(a.name) LIKE '%toyota motor%'))
  AND p.publication_date >= 20180101
ORDER BY broad_context_match_count DESC, p.publication_date DESC
LIMIT 25;
