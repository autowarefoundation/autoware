# Patent review workflow sample data

This directory contains test-only documentation and sample configuration data for
validating the `autoware-patent-search` module-based BigQuery workflow.

These files are **not** an Autoware runtime module and should not be installed,
launched, or proposed upstream. They are intentionally stored under the skill's
`examples/` directory so reviewers can exercise the patent-review workflow
without changing the real Autoware source tree or repository dependencies.

The sample configuration represents a conceptual behavior-planning review target:
lane-change path generation with a predicted-object collision-check margin. Use
it as workflow documentation/test data for extracting technical features,
bilingual search keywords, CPC/IPC hints, and BigQuery SQL.
