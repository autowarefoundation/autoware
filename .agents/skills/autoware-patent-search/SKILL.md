---
name: autoware-patent-search
description: Review existing Autoware modules for patent-sensitive autonomous-driving technical features, generate Google Patents Public Data BigQuery searches, and produce non-legal candidate patent review reports. Use when asked to evaluate Autoware modules, packages, directories, or components for IP/patent review candidates; generate patent search keywords in English/Japanese; generate BigQuery SQL for Google Patents Public Data; produce non-legal patent-sensitivity reports for planning, control, fallback/MRM, route planning, perception-to-planning interactions, maps, traffic signals, infrastructure, pedestrians, or other vehicle-interaction behavior.
---

# Autoware Patent Search

## Core rule

Do not make a legal conclusion about infringement, validity, freedom to operate, or non-infringement. Treat the output as technical triage for IP counsel or patent professionals. Say that results are candidates for review, not determinations. If the user asks for automated infringement judgment, convert it into a non-legal claim-comparison worksheet with a human-review disposition; do not output an infringement/non-infringement verdict.

Use the product/project name **Autoware** consistently.

## MVP scope

The MVP is **module-based**, not PR-diff-based. Evaluate an existing Autoware module, package, directory, or component and identify potentially relevant third-party patent documents for human IP review.

Suggested MVP targets include:

- planning, behavior planning, trajectory planning, obstacle avoidance, lane change, and stop decision
- vehicle control
- fallback, MRM, safety monitor, fail-operational, degradation, and emergency stop behavior
- perception-to-planning interfaces and prediction-to-planning interactions
- route planning, maps, traffic signals, infrastructure, pedestrians, and other vehicle-interaction behavior

Google Patents Public Data queried through **BigQuery** is the primary retrieval mechanism. Do not require a Google Patents-specific API, do not automate or scrape Google Patents web pages, and do not use non-Google patent databases. Google Patents search URLs may be generated only as supporting links for human review.

## Workflow

1. Accept an Autoware module, package, directory, or component as input.
2. Inspect available module evidence, including README files, source files, launch/config files, parameters, ROS nodes, topics, classes, functions, package manifests, and tests.
3. Extract patent-sensitive technical features from the existing implementation. Prioritize:
   - behavior, trajectory, motion, route, or lane planning
   - vehicle control
   - fallback behavior, MRM, fail-operational behavior, degradation, emergency stops, or safety monitors
   - interactions with vehicles, pedestrians, traffic signals, maps, V2X, infrastructure, prediction, or perception outputs
4. Extract inputs, outputs, algorithmic steps, behavior logic, thresholds, state machines, optimization objectives, data associations, map queries, and message topics.
5. Generate bilingual search terms using `references/search-taxonomy.md` and optionally the helper script:
   ```bash
   python .agents/skills/autoware-patent-search/scripts/patent_cross_search_queries.py \
     --module "planning/behavior_path_planner" \
     --feature "lane change path generation with collision check using predicted objects" \
     --keyword "behavior planning" \
     --jp "車線変更 経路生成 衝突判定 予測物体" \
     --cpc B60W --cpc G05D1/00
   ```
6. Generate reproducible BigQuery Standard SQL for Google Patents Public Data. Use combinations of English technical keywords, Japanese technical keywords, CPC/IPC hints, assignee/company filters, publication/application metadata, and title/abstract/description/claims text where available.
7. Run the SQL when BigQuery access is available. If BigQuery access is unavailable, record that limitation and provide the reproducible SQL query instead. The example SQL in this repository has been structurally validated, but it has not been executed in this environment.
8. As the next validation step, a human reviewer should configure a Google Cloud project with BigQuery access and run either a BigQuery dry run or a small `LIMIT 10` query before relying on candidate retrieval results.
9. Use optional Google Patents search URLs only as human-review links. Do not treat URLs as the main retrieval source.
10. Read candidate patent titles, abstracts, snippets, and claims at a high level and compare technical concepts only. Do not overstate similarity.
11. For claim charts, generate a **claim-comparison scaffold** only:
    - identify claim elements from candidate patent text at a high level
    - map each element to Autoware files, functions, parameters, topics, or behavior evidence when available
    - record differences, assumptions, and uncertainty
    - leave the final disposition as "Needs IP review" or "Not enough evidence" rather than infringement/non-infringement
12. For continuous monitoring, save BigQuery SQL queries, cadence, alert routing, and deduplication keys. Alerts should say "new candidate for IP review" and must not use legal conclusion language.
13. Produce the required report format below.

## Requirements and definition-of-done co-design

When the user has not finalized requirements, suggest this starting point and refine it with them:

- **Purpose**: find patent documents worth IP-professional review for existing Autoware modules.
- **Primary patent data source**: Google Patents Public Data through BigQuery. Do not add any additional patent database unless the user explicitly changes scope in a later request.
- **Supporting links**: Google Patents web URLs may be generated for human review, but candidate retrieval is BigQuery-first.
- **Scope**: module behavior and directly affected planning/control/safety/perception-to-planning interfaces.
- **Out of scope**: legal opinions, automated infringement/non-infringement conclusions, validity opinions, freedom-to-operate sign-off, Google Patents browser automation, web scraping, or non-Google patent databases.
- **Evidence captured**: module files/classes/functions/nodes/topics reviewed, exact BigQuery SQL, whether the SQL was only structurally validated or actually executed, date searched, filters, candidate publication/application identifiers, assignees, dates, titles/abstract snippets, CPC/IPC hints, optional Google Patents links, and technical reason for relevance.
- **Definition of done**: every patent-sensitive feature in the selected module has English/Japanese keywords, CPC/IPC hints if inferable, reproducible BigQuery SQL, candidate results or a no-candidate explanation, optional claim-comparison scaffold when candidates exist, risk level, and next human IP review action.

## Required report format

For each Autoware module, output:

1. Module/package name
2. Files/classes/functions/ROS nodes/topics reviewed
3. Technical feature summary
4. Inputs and outputs
5. Algorithmic steps or behavior logic
6. Why the feature may be patent-sensitive
7. English patent search keywords
8. Japanese patent search keywords
9. CPC/IPC hints
10. BigQuery SQL queries generated for Google Patents Public Data
11. Candidate patent documents and assignees
12. Technical reason for relevance
13. Risk level: High / Medium / Watch / Low
14. Reason for risk level
15. Recommended next human IP review action

If no relevant patent-sensitive technical feature is found, state that no candidate feature was identified and explain the module files/functions reviewed.

## Candidate language

Use candidate-only language such as:

- "candidate for IP review"
- "potentially relevant patent document"
- "technical overlap candidate"
- "needs human IP review"
- "not enough evidence"

Do not state or imply infringement, non-infringement, validity, invalidity, or FTO clearance.

## Risk level guidance

- **High**: Existing module contains or exposes a planning/control/fallback algorithm with specific decision logic, optimization, prediction interaction, map/traffic interaction, or safety/MRM behavior and close technical-overlap candidates are found.
- **Medium**: Existing module has meaningful autonomous-driving behavior logic, thresholds, state transitions, or object/trajectory handling and plausible candidates are found.
- **Watch**: Module touches patent-sensitive interfaces or parameters, but the behavior is generic, incomplete, or candidate overlap is weak.
- **Low**: Documentation, build, CI, dependency, formatting, tests, or non-AD behavior with no apparent autonomous-driving technical algorithm change.

## Patent matching, claim-comparison, and monitoring automation

- **BigQuery retrieval**: generate and, when available, run Google Patents Public Data Standard SQL against `patents-public-data.patents.publications`. Search localized title, abstract, description, and claims text where available, along with CPC/IPC, assignee, publication/application dates, and publication/application identifiers. In this repository, generated SQL examples are structurally validated but not executed; next validation is a human BigQuery dry run or small `LIMIT 10` query after Google Cloud project setup.
- **Optional human-review links**: include Google Patents web links such as `https://patents.google.com/patent/<publication_number>` in candidate tables or optional search URL sections. Do not scrape these pages.
- **Automated matching**: rank candidates by technical overlap signals such as shared feature terms, CPC/IPC hints, assignee scope requested by the user, and overlap between claim concepts and Autoware behavior evidence. Treat ranking as triage only.
- **Claim-comparison scaffold**: use `scripts/patent_review_scaffold.py` after collecting candidates into CSV. The worksheet may compare claim elements to Autoware evidence, but final legal disposition must remain for human/IP-professional review.
- **Continuous monitoring**: save the exact BigQuery SQL generated by `scripts/patent_cross_search_queries.py`, run it on a cadence such as weekly or monthly, deduplicate alerts by publication/application number, and alert only with "new candidate for IP review" language while avoiding legal conclusion wording.

## Search tips

- Start from module evidence: README descriptions, package manifests, launch/config parameters, ROS topics, classes, functions, and algorithm-specific constants.
- Search English and Japanese terms separately and together; Japanese technical terms remain useful in Google Patents Public Data localized text fields.
- Prefer broad technical terms first, then add implementation-specific words such as state machine, cost function, drivable area, occupancy grid, predicted path, stop line, emergency stop, or MRM.
- Search by CPC/IPC only as a supplement; classifications are hints, not proof that a result is relevant.
- Record the exact BigQuery SQL used in the final report when the user asks for traceability.

## Resources

- `references/search-taxonomy.md`: module-based bilingual terms and CPC/IPC hints for Google Patents Public Data review.
- `scripts/patent_cross_search_queries.py`: legacy-named helper that generates BigQuery SQL for Google Patents Public Data plus optional human-review Google Patents URLs.
- `scripts/google_patents_queries.py`: compatibility wrapper that emits optional Google Patents review URLs only.
- `scripts/patent_review_scaffold.py`: generate a non-legal candidate table, claim-comparison worksheet, and monitoring plan from reviewed candidate patent records.
- `examples/module_review_lane_change.md`: sample module-based output report with candidate placeholders.
- `examples/lane_change_bigquery.sql`: sample BigQuery SQL generated for a lane-change module review.
- `examples/patent_review_sample/`: test-only workflow documentation/sample data; not an Autoware runtime module.
