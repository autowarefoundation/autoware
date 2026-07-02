You are performing a patent-risk screening for an autonomous driving software repository.

Scope:
- Focus only on planning, behavior planning, trajectory planning, control, fallback, and MRM-related changes.
- Do not make a legal conclusion.
- Do not say that the code infringes a patent.
- Identify technical features that should be reviewed by IP counsel.

Task:
1. Inspect the repository changes.
2. Identify patent-sensitive technical features.
3. Extract technical elements.
4. Generate English and Japanese patent search keywords.
5. Suggest likely patent search classifications if inferable.
6. Assign a preliminary risk level: High, Medium, Watch, Low.
7. Explain why human IP review is or is not needed.

Output in the following structure:

risk_level: High / Medium / Watch / Low
patent_review_needed: true / false
technical_domain:
feature_summary:
changed_files:
technical_elements:
search_keywords_en:
search_keywords_ja:
candidate_patent_queries:
human_review_points:
reason:
