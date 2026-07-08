# Sample module-based patent review report: lane change planning

This is sample output for validating the `autoware-patent-search` MVP workflow. It is technical triage only and is not a legal opinion about infringement, validity, freedom to operate, or non-infringement.

## 1. Module/package name

`planning/behavior_path_planner/lane_change` (example module target)

## 2. Files/classes/functions/ROS nodes/topics reviewed

Sample placeholders for a real module review:

- README/package documentation: `README.md`
- Launch/config/parameters: lane-change safety threshold and collision-check parameters
- Source/classes/functions: lane-change path generation, predicted-object collision check, safety evaluation
- ROS interfaces: route/lanelet inputs, predicted objects, candidate path output, behavior-planning status topics

## 3. Technical feature summary

Lane-change path generation that evaluates candidate paths against predicted objects and applies collision-check or safety-margin logic before accepting a lane-change maneuver.

## 4. Inputs and outputs

- Inputs: ego pose/velocity, route/lanelet map, predicted objects, candidate lane-change paths, safety thresholds
- Outputs: accepted/rejected lane-change path, behavior-planning status, optional diagnostic or debug markers

## 5. Algorithmic steps or behavior logic

1. Generate candidate lane-change paths from current lane to target lane.
2. Estimate object interactions using predicted object trajectories.
3. Apply collision or safety-margin checks over the candidate path horizon.
4. Accept, reject, or defer the lane-change maneuver based on safety logic.

## 6. Why the feature may be patent-sensitive

The feature combines autonomous-driving behavior planning, path generation, object prediction, and collision/safety decision logic. These are common areas for patent filings, so matching patent documents should be treated as candidates for human IP review.

## 7. English patent search keywords

lane change, path generation, collision check, predicted objects, behavior planning, autonomous vehicle, trajectory planning, safety margin

## 8. Japanese patent search keywords

車線変更, 経路生成, 衝突判定, 予測物体, 行動計画, 自動運転, 安全余裕

## 9. CPC/IPC hints

- B60W
- G05D1/00
- G08G1/00

## 10. BigQuery SQL queries generated for Google Patents Public Data

See `examples/lane_change_bigquery.sql`. The SQL is structurally validated in this repository, but it has not been executed in this environment.

## 11. Candidate patent documents and assignees

| Publication | Assignee | Title | Status |
| --- | --- | --- | --- |
| TBD from BigQuery result | TBD | TBD | candidate for IP review |

## 12. Technical reason for relevance

Placeholder candidates should be reviewed for overlap with lane-change path generation, predicted-object collision checks, safety margins, and autonomous-driving behavior-planning decision logic.

## 13. Risk level

Watch

## 14. Reason for risk level

This sample describes a patent-sensitive technical area, but candidate patent documents have not been retrieved or reviewed in this placeholder report.

## 15. Recommended next human IP review action

After configuring a Google Cloud project with BigQuery access, run a BigQuery dry run or a small `LIMIT 10` query, then collect candidate publication records, and have an IP professional review the technical overlap. Use "needs human IP review" or "not enough evidence" as dispositions; do not make legal conclusions.
