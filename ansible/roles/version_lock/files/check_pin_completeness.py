#!/usr/bin/env python3
"""Filter package names to those installed from the Ubuntu archive.

Reads package names (one per line) on stdin and prints the ones whose
installed version carries a Release "Origin: Ubuntu" — i.e. packages that a
locked install must have an apt_pins entry for. ROS-repo (Origin: ROS) and
NVIDIA-repo packages are excluded: the dated snapshot and nvidia_pins freeze
those. Used by the locked-mode pin-completeness check in verify.yaml.
"""

import sys

import apt


def main() -> None:
    names = [line.strip() for line in sys.stdin if line.strip()]
    cache = apt.Cache()
    for name in names:
        if name not in cache:
            continue
        pkg = cache[name]
        if not pkg.is_installed:
            continue
        if any(o.origin == "Ubuntu" for o in pkg.installed.origins):
            print(name)


if __name__ == "__main__":
    main()
