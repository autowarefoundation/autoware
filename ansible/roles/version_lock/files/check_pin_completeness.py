#!/usr/bin/env python3
"""Filter package names to those installed from the Ubuntu archive.

Reads package names (one per line) on stdin and prints the ones whose
installed version carries a Release "Origin: Ubuntu" — i.e. packages that a
locked install must have an apt_pins entry for. ROS-repo (Origin: ROS) and
NVIDIA-repo packages are excluded: the dated snapshot and nvidia_pins freeze
those. Used by the locked-mode pin-completeness check in verify.yaml.

Kernel-coupled packages (linux-headers-*, linux-image-*, linux-modules-*) are
also excluded. The agnocast role installs linux-headers-{{ ansible_kernel }}
on bare metal, whose name varies per machine and changes on every kernel
upgrade, so no static lockfile key could ever cover it.
"""

import re
import sys

import apt

KERNEL_COUPLED = re.compile(r"^linux-(headers|image|modules)-")


def main() -> None:
    names = [line.strip() for line in sys.stdin if line.strip()]
    cache = apt.Cache()
    for name in names:
        if KERNEL_COUPLED.match(name):
            continue
        if name not in cache:
            continue
        pkg = cache[name]
        if not pkg.is_installed:
            continue
        if any(o.origin == "Ubuntu" for o in pkg.installed.origins):
            print(name)


if __name__ == "__main__":
    main()
