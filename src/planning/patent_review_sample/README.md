# Patent review workflow sample

This directory contains a test-only Autoware planning sample used to validate the
`autoware-patent-search` skill in this fork.

The sample configuration represents a conceptual behavior-planning change for
lane-change path generation: increasing the predicted-object collision-check
margin used during lane-change safety evaluation. It is intentionally small and
reviewable so the patent review workflow can identify a planning feature,
changed inputs/outputs, algorithmic implications, CPC hints, and bilingual search
keywords without proposing any change to upstream Autoware.

Do not submit this sample change to `autowarefoundation/autoware`.
