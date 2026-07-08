#!/usr/bin/env python3
"""Additively record the installed NVIDIA apt closure into a lockfile's nvidia_pins section.

Preserves every other section and the header verbatim. NVIDIA publishes no dated
snapshot, so the whole installed CUDA/TensorRT closure is pinned by exact version
over its accretive repo. Run this on a machine that has just completed an
(unlocked) install_nvidia and an `apt-get update`; it reads the currently-installed
versions and rewrites only nvidia_pins.
"""

# cspell:ignore indextargets
import glob
import os
import subprocess
import sys

import yaml

KNOWN_KEYS = {"ros_snapshot_date", "apt_pins", "pip_pins", "nvidia_pins", "ros_overrides"}


def _parse_stanzas(text):
    """Yield (pkg, ver) from Package:/Version: blank-line-delimited apt index stanzas."""
    pkg = ver = None
    for line in text.splitlines():
        if line.startswith("Package: "):
            pkg = line[len("Package: ") :].strip()
        elif line.startswith("Version: "):
            ver = line[len("Version: ") :].strip()
        elif not line.strip():
            if pkg and ver:
                yield pkg, ver
            pkg = ver = None
    if pkg and ver:
        yield pkg, ver


def _nvidia_index_texts_from_lists_dir(lists_dir):
    """Test/fixture path: read plain-text NVIDIA Packages files directly from a directory.

    Only used when APT_LISTS_DIR is explicitly set (the unit test's fixture path, which
    drops plain uncompressed _Packages files). Real apt lists directories store indexes
    compressed (see _nvidia_index_texts_from_apt) so this glob would find nothing there.
    """
    idx_files = sorted(
        glob.glob(os.path.join(lists_dir, "*developer.download.nvidia.com*_Packages"))
    )
    if not idx_files:
        sys.exit(
            f"Error: no NVIDIA apt index under {lists_dir} "
            "(run `apt-get update` after install_nvidia)."
        )
    texts = []
    for idx in idx_files:
        with open(idx) as fh:
            texts.append(fh.read())
    return texts


def _nvidia_index_texts_from_apt():
    """Real/CI path: discover NVIDIA Packages indexes via apt and read them decompressed.

    apt-get's target images enable Acquire::GzipIndexes, so `apt-get update` stores
    indexes as *_Packages.lz4 (or .gz), not plain *_Packages files. Go through apt
    itself instead of guessing a compression scheme: `apt-get indextargets` enumerates
    every index apt knows about (filtered to Packages-type indexes via the
    `CREATED-BY: Packages` record filter, which excludes Translations/Sources/DEP-11),
    and `apt-helper cat-file` decompresses whichever format is on disk transparently.
    """
    try:
        result = subprocess.run(
            [
                "apt-get",
                "indextargets",
                "--format",
                "$(FILENAME)\t$(URI)",
                "CREATED-BY: Packages",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError) as e:
        sys.exit(f"Error: `apt-get indextargets` failed ({e}); is apt-get installed?")

    filenames = []
    for line in result.stdout.splitlines():
        if not line.strip():
            continue
        filename, _, uri = line.partition("\t")
        if "developer.download.nvidia.com" in uri:
            filenames.append(filename)
    if not filenames:
        sys.exit(
            "Error: no NVIDIA apt index found via `apt-get indextargets` "
            "(run `apt-get update` after install_nvidia)."
        )

    texts = []
    for filename in filenames:
        try:
            result = subprocess.run(
                ["/usr/lib/apt/apt-helper", "cat-file", filename],
                capture_output=True,
                text=True,
                check=True,
            )
        except (OSError, subprocess.CalledProcessError) as e:
            sys.exit(
                f"Error: `apt-helper cat-file {filename}` failed ({e}); is apt-helper installed?"
            )
        texts.append(result.stdout)
    return texts


def installed_nvidia_pins():
    # pkg -> set of versions offered by an NVIDIA apt index.
    lists_dir = os.environ.get("APT_LISTS_DIR")
    if lists_dir:
        texts = _nvidia_index_texts_from_lists_dir(lists_dir)
    else:
        texts = _nvidia_index_texts_from_apt()

    offered = {}
    for text in texts:
        for pkg, ver in _parse_stanzas(text):
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
    unknown = set(data) - KNOWN_KEYS
    if unknown:
        sys.exit(
            "Error: lockfile has unknown top-level key(s) that would be silently dropped: "
            + ", ".join(sorted(unknown))
        )
    lines = [header.rstrip("\n")]
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
    try:
        with open(path) as fh:
            text = fh.read()
        data = yaml.safe_load(text) or {}
    except OSError as e:
        sys.exit(f"Error: cannot read {path}: {e}")
    except yaml.YAMLError as e:
        sys.exit(f"Error: {path} is not valid YAML: {e}")
    header_lines = []
    for line in text.splitlines():
        if line.startswith("#") or not line.strip():
            header_lines.append(line)
        else:
            break
    if "ros_snapshot_date" not in data:
        sys.exit(f"Error: {path} has no ros_snapshot_date; is it a filled lockfile?")
    data["nvidia_pins"] = installed_nvidia_pins()
    # Render before truncating the output file: render() can sys.exit (e.g. on an
    # unknown top-level key), and opening in "w" mode truncates immediately, so
    # rendering first avoids destroying the lockfile on a failed run.
    rendered = render("\n".join(header_lines), data)
    with open(path, "w") as fh:
        fh.write(rendered)
    print(f"Updated nvidia_pins ({len(data['nvidia_pins'])} packages) in {path}")


if __name__ == "__main__":
    main()
