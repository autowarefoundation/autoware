#!/usr/bin/env python3
"""List installed packages diverging from the dated ROS snapshot.

Prints one "name=candidate_version" line for every installed package whose
apt candidate comes from snapshots.ros.org (the dated snapshot configured in
locked mode, pinned to priority 1001) at a version different from the one
installed. Scoping to snapshot-origin candidates keeps the reconcile step from
touching Ubuntu-archive packages: those are frozen by the base-image digest
plus explicit apt_pins, and moving them would break that freeze.

Packages under an `apt-mark hold` are skipped and reported on stderr instead:
a hold is an explicit operator decision, and apt refuses to act on a held
package unless told to override the hold, which would abort the whole
reconcile over one package the user deliberately froze.

Requires python3-apt (installed by this role's tasks/main.yaml) and an
up-to-date apt cache.
"""

import sys

import apt
import apt_pkg

SNAPSHOT_SITE = "snapshots.ros.org"


def main() -> None:
    cache = apt.Cache()
    held = []
    for pkg in cache:
        if not pkg.is_installed or pkg.candidate is None:
            continue
        if pkg.candidate.version == pkg.installed.version:
            continue
        if not any(o.site == SNAPSHOT_SITE for o in pkg.candidate.origins):
            continue
        if pkg._pkg.selected_state == apt_pkg.SELSTATE_HOLD:
            held.append(pkg.name)
            continue
        print(f"{pkg.name}={pkg.candidate.version}")
    if held:
        print(
            "skipped (apt-mark hold): " + " ".join(sorted(held)),
            file=sys.stderr,
        )


if __name__ == "__main__":
    main()
