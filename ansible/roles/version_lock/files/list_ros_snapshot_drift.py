#!/usr/bin/env python3
"""List installed packages diverging from the dated ROS snapshot.

Prints one "name=candidate_version" line for every installed package whose
apt candidate comes from snapshots.ros.org (the dated snapshot configured in
locked mode, pinned to priority 1001) at a version different from the one
installed. Scoping to snapshot-origin candidates keeps the reconcile step from
touching Ubuntu-archive packages: those are frozen by the base-image digest
plus explicit apt_pins, and moving them would break that freeze.

Requires python3-apt (already required by Ansible's apt module) and an
up-to-date apt cache.
"""

import apt

SNAPSHOT_SITE = "snapshots.ros.org"


def main() -> None:
    cache = apt.Cache()
    for pkg in cache:
        if not pkg.is_installed or pkg.candidate is None:
            continue
        if pkg.candidate.version == pkg.installed.version:
            continue
        if any(o.site == SNAPSHOT_SITE for o in pkg.candidate.origins):
            print(f"{pkg.name}={pkg.candidate.version}")


if __name__ == "__main__":
    main()
