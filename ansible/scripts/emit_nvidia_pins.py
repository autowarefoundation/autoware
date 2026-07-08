#!/usr/bin/env python3
"""Additively record the installed NVIDIA apt closure into a lockfile's nvidia_pins section.

Preserves every other section and the header verbatim. NVIDIA publishes no dated
snapshot, so the whole installed CUDA/TensorRT closure is pinned by exact version
over its accretive repo. Run this on a machine that has just completed an
(unlocked) install_nvidia and an `apt-get update`; it reads the currently-installed
versions and rewrites only nvidia_pins.
"""

import glob
import os
import subprocess
import sys

import yaml

APT_LISTS_DIR = os.environ.get("APT_LISTS_DIR", "/var/lib/apt/lists")


def installed_nvidia_pins():
    # pkg -> set of versions offered by an NVIDIA apt index.
    offered = {}
    idx_files = glob.glob(os.path.join(APT_LISTS_DIR, "*developer.download.nvidia.com*_Packages"))
    if not idx_files:
        sys.exit(
            f"Error: no NVIDIA apt index under {APT_LISTS_DIR} "
            "(run `apt-get update` after install_nvidia)."
        )
    for idx in idx_files:
        pkg = ver = None
        with open(idx) as fh:
            for line in fh:
                if line.startswith("Package: "):
                    pkg = line[len("Package: ") :].strip()
                elif line.startswith("Version: "):
                    ver = line[len("Version: ") :].strip()
                elif not line.strip():
                    if pkg and ver:
                        offered.setdefault(pkg, set()).add(ver)
                    pkg = ver = None
        if pkg and ver:
            offered.setdefault(pkg, set()).add(ver)

    pins = {}
    for pkg, versions in offered.items():
        r = subprocess.run(
            ["dpkg-query", "-W", "-f=${Version}", pkg], capture_output=True, text=True
        )
        iv = r.stdout.strip()
        if r.returncode == 0 and iv and iv in versions:
            pins[pkg] = iv
    if not pins:
        sys.exit("Error: no installed NVIDIA-origin packages found; did install_nvidia run?")
    return dict(sorted(pins.items()))


def render(header, data):
    lines = [header.rstrip("\n"), ""]
    lines.append(f'ros_snapshot_date: "{data["ros_snapshot_date"]}"')
    for key in ("apt_pins", "pip_pins", "nvidia_pins", "ros_overrides"):
        section = data.get(key) or {}
        if not section:
            lines.append(f"{key}: {{}}")
        else:
            lines.append(f"{key}:")
            for k in sorted(section):
                lines.append(f"  {k}: {section[k]}")
    return "\n".join(lines) + "\n"


def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: emit_nvidia_pins.py <lockfile.yaml>")
    path = sys.argv[1]
    text = open(path).read()
    header_lines = []
    for line in text.splitlines():
        if line.startswith("#") or not line.strip():
            header_lines.append(line)
        else:
            break
    data = yaml.safe_load(text) or {}
    if "ros_snapshot_date" not in data:
        sys.exit(f"Error: {path} has no ros_snapshot_date; is it a filled lockfile?")
    data["nvidia_pins"] = installed_nvidia_pins()
    with open(path, "w") as fh:
        fh.write(render("\n".join(header_lines), data))
    print(f"Updated nvidia_pins ({len(data['nvidia_pins'])} packages) in {path}")


if __name__ == "__main__":
    main()
