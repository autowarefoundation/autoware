## Review guidelines

You are not making a legal conclusion about patent infringement.

For each pull request, identify whether the change introduces or materially modifies a patent-sensitive autonomous driving technical feature.

Focus on:

- behavior planning
- trajectory planning
- motion planning
- vehicle control
- fallback behavior
- MRM
- fail-operational behavior
- route planning
- interaction with other vehicles, pedestrians, traffic signals, maps, or infrastructure

For each relevant change, output:

1. Technical feature summary
2. Changed packages, files, classes, functions, and ROS nodes
3. Inputs and outputs
4. Algorithmic steps
5. Why the feature may be patent-sensitive
6. English patent search keywords
7. Japanese patent search keywords
8. Possible patent classification hints, if inferable
9. Risk level: High / Medium / Watch / Low
10. Reason for the risk level

Do not state that the implementation infringes a patent.
Only flag candidates for intellectual property review.
