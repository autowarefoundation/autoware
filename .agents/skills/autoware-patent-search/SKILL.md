---
name: autoware-patent-search
description: Review Autoware pull requests or code changes for patent-sensitive autonomous-driving features, search the two target patent databases, Google Patents and J-PlatPat, generate non-legal claim-comparison scaffolds, and outline monitoring alerts. Use when asked to check Autoware changes for IP/patent review candidates, generate patent search keywords in English/Japanese, search patents in Google Patents and J-PlatPat, compare candidate assignees for user review, draft claim charts for attorney review, configure patent monitoring, or produce a non-legal patent-sensitivity report for planning, control, fallback/MRM, route planning, perception-to-planning interactions, maps, traffic signals, infrastructure, pedestrians, or other vehicle-interaction behavior.
---

# Autoware Patent Search

## Core rule

Do not make a legal conclusion about infringement, validity, freedom to operate, or non-infringement. Treat the output as technical triage for IP counsel or patent professionals. Say that results are candidates for review, not determinations. If the user asks for automated infringement judgment, convert it into a non-legal claim-comparison worksheet with a human-review disposition; do not output an infringement/non-infringement verdict.

Use the product/project name **Autoware** consistently.

## Workflow

1. Identify changed Autoware functionality from the PR diff, files, package manifests, class/function names, launch files, parameters, message types, and ROS node names.
2. Decide whether the change introduces or materially modifies a patent-sensitive autonomous-driving technical feature. Prioritize:
   - behavior, trajectory, motion, route, or lane planning
   - vehicle control
   - fallback behavior, MRM, fail-operational behavior, degradation, emergency stops, or safety monitors
   - interactions with vehicles, pedestrians, traffic signals, maps, V2X, infrastructure, or prediction
3. Extract inputs, outputs, algorithmic steps, thresholds, state machines, optimization objectives, data associations, map queries, and message topics.
4. Generate bilingual search terms using `references/search-taxonomy.md` and optionally the helper script:
   ```bash
   python .agents/skills/autoware-patent-search/scripts/patent_cross_search_queries.py --feature "lane change path generation with collision check" --jp "車線変更 経路生成 衝突判定"
   ```
   The older `google_patents_queries.py` wrapper remains available for compatibility.
5. Search only the two target patent databases, **Google Patents** and **J-PlatPat**, using multiple query variants. When internet access and either target database UI/API is available, actually open the database results, capture candidate patent identifiers, titles, assignees, publication/application dates, abstracts/snippets, and URLs. If access is blocked, record that limitation and provide reproducible queries instead:
   - **Google Patents** for broad international cross-searching, English/Japanese keyword variants, CPC hints, and assignee filters.
   - **J-PlatPat** for Japan-focused patent searches. Use Japanese terms first, then English technical terms where useful. Because J-PlatPat URLs and session behavior can change, record the exact keywords and filters entered even when a stable deep link is unavailable.
   - Combine broad domain terms with implementation-specific words such as state machine, cost function, drivable area, occupancy grid, predicted path, stop line, emergency stop, or MRM.
   - Add CPC/IPC hints from `references/search-taxonomy.md` when the feature is clear.
6. Assignee/company handling:
   - Do **not** decide which company's IP is infringed.
   - Do **not** limit the search to one company by default.
   - If the user provides companies, add them as assignee filters and report candidates for the user to inspect.
   - If the user wants to decide the company risk themselves, present a candidate-assignee table with database, query, publication/application number, assignee, title, and why it might be relevant.
7. Read patent abstracts/claims at a high level and compare technical concepts only. Do not overstate similarity.
8. For claim charts, generate a **claim-comparison scaffold** only:
   - identify claim elements from candidate patent text at a high level
   - map each element to Autoware files, functions, parameters, topics, or behavior evidence when available
   - record differences, assumptions, and uncertainty
   - leave the final disposition as "Needs IP review" or "Not enough evidence" rather than infringement/non-infringement
9. For continuous monitoring, define saved queries for Google Patents and J-PlatPat, cadence, alert routing, and deduplication keys. Alerts should say "new candidate for IP review" and must not use legal conclusion language.
10. If requirements, scope, or definition of done are unclear, propose a lightweight review charter before doing a full search. Include scope, the two target databases, excluded conclusions, evidence to capture, and completion criteria.
11. Produce the required report format below.

## Requirements and definition-of-done co-design

When the user has not finalized requirements, suggest this starting point and refine it with them:

- **Purpose**: find patent documents worth IP-professional review for Autoware technical changes.
- **Databases**: only Google Patents and J-PlatPat are in scope. Do not add any additional patent database unless the user explicitly changes the target scope in a later request.
- **Scope**: changed Autoware behavior and its directly affected planning/control/safety interfaces.
- **Out of scope**: legal opinions, automated infringement/non-infringement conclusions, claim charts presented as legal analysis, validity opinions, or freedom-to-operate sign-off.
- **Evidence captured**: exact queries, database used, date searched, filters, result links or identifiers, candidate assignees, publication/application dates, relevant abstracts/snippets, and technical reason for relevance.
- **Definition of done**: every patent-sensitive changed feature has bilingual keywords, Google Patents and J-PlatPat search attempts, candidate results or a no-candidate explanation, optional claim-comparison scaffold when candidates exist, risk level, monitoring recommendation, and next human review action.

## Required report format

For each relevant change, output:

1. Technical feature summary
2. Changed packages, files, classes, functions, and ROS nodes
3. Inputs and outputs
4. Algorithmic steps
5. Why the feature may be patent-sensitive
6. English patent search keywords
7. Japanese patent search keywords
8. Google Patents and J-PlatPat searches performed, with exact queries/filters used
9. Candidate patent documents and assignees for user/IP review, if any
10. Possible patent classification hints, if inferable
11. Risk level: High / Medium / Watch / Low
12. Reason for the risk level
13. Non-legal claim-comparison scaffold, if requested or if strong candidates exist
14. Continuous-monitoring query/alert recommendation, if requested
15. Recommended next review action

If no relevant patent-sensitive technical feature is found, state that no candidate was identified and explain the changed files/functions reviewed.

## Risk level guidance

- **High**: New or materially changed planning/control/fallback algorithm with specific decision logic, optimization, prediction interaction, map/traffic interaction, or safety/MRM behavior.
- **Medium**: Meaningful modification to an existing autonomous-driving algorithm or node behavior, including thresholds, state transitions, or object/trajectory handling.
- **Watch**: Refactor, parameter, interface, visualization, or tooling change that touches patent-sensitive modules but does not clearly alter core behavior.
- **Low**: Documentation, build, CI, dependency, formatting, tests, or non-AD behavior with no apparent autonomous-driving technical algorithm change.

## Patent matching, claim-comparison, and monitoring automation

- **Actual database search**: perform live searches in Google Patents and J-PlatPat when available. Record target database, query, filters, date, result count if visible, candidate publication/application numbers, assignees, titles, links, and why each candidate is technically relevant.
- **Automated matching**: rank candidates by technical overlap signals such as shared feature terms, CPC/IPC/FI/F-term hints, assignee scope requested by the user, and overlap between claim concepts and Autoware behavior evidence. Treat ranking as triage only.
- **Claim-comparison scaffold**: use `scripts/patent_review_scaffold.py` after collecting candidates into CSV. The worksheet may compare claim elements to Autoware evidence, but final legal disposition must remain for human/IP-professional review.
- **Continuous monitoring**: save the exact Google Patents and J-PlatPat queries generated by `scripts/patent_cross_search_queries.py`, run them on a cadence such as weekly or monthly, deduplicate alerts by target database plus publication/application number, and alert only with "new candidate for IP review" language while avoiding legal conclusion wording.

## Search tips

- Use URLs produced by the helper script as starting points, then refine manually in each database.
- Search English and Japanese terms separately; Japanese patent documents may have English machine translations, but Japanese technical terms often reveal different results.
- Prefer broad technical terms first, then add implementation-specific words.
- Search by CPC/IPC only as a supplement; classifications are hints, not proof that a result is relevant.
- Record the exact search queries used in the final report when the user asks for traceability.

## Resources

- `references/search-taxonomy.md`: bilingual terms, database notes, and CPC/IPC hints for Autoware autonomous-driving review.
- `scripts/patent_cross_search_queries.py`: generate Google Patents URLs and J-PlatPat query plans from feature phrases, Japanese terms, CPC codes, and optional assignees.
- `scripts/google_patents_queries.py`: compatibility wrapper that emits Google Patents URLs only.
- `scripts/patent_review_scaffold.py`: generate a non-legal candidate table, claim-comparison worksheet, and monitoring plan from reviewed candidate patent records.
