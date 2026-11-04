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

# These are installed by opt-in roles alone -- demo_artifacts and carla, both
# tags: [never] and in no other playbook -- so they converge on no normal
# locked path, while verify.yaml asserts every pin whose package is installed
# regardless of which roles ran. All of them but libomp5 are also pulled in by
# ubuntu-desktop, so pinning them fails a locked install_dev_env on any desktop
# whose version differs from the pin, with no task in the play able to fix it.
OUT_OF_SCOPE = frozenset(
    {
        # demo_artifacts
        "unzip",
        # carla: carla__apt_packages in ansible/roles/carla/defaults/main.yaml
        "libomp5",
        "libsdl2-2.0-0",
        "libvulkan1",
        "xdg-user-dirs",
    }
)


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
