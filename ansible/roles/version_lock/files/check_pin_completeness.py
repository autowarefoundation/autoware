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

OUT_OF_SCOPE names a second, distinct exemption: packages a static key *could*
cover, but that sit outside the freeze's remit. Pinning them would make
verify.yaml fail a locked run that no role in the play can converge, since the
verify post_task asserts every pin whose package is installed regardless of
which roles ran. See the version_lock README for the per-package rationale.
"""

import re
import sys

import apt

KERNEL_COUPLED = re.compile(r"^linux-(headers|image|modules)-")

# unzip is installed by demo_artifacts alone, which is tags: [never] and in no
# other playbook, so it converges on no normal locked path. It is also a hard
# Depends of ubuntu-desktop, so pinning it fails a locked install_dev_env on
# any desktop whose unzip differs from the pin, with no task able to fix it.
OUT_OF_SCOPE = frozenset({"unzip"})


def main() -> None:
    names = [line.strip() for line in sys.stdin if line.strip()]
    cache = apt.Cache()
    for name in names:
        if KERNEL_COUPLED.match(name) or name in OUT_OF_SCOPE:
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
