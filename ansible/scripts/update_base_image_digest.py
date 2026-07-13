#!/usr/bin/env python3
"""Update one distro's entry in the BASE_IMAGE_DIGESTS map of docker/docker-bake.hcl.

The base-image digests in docker-bake.hcl and the ros_snapshot_date in the ansible
lockfiles are the two halves of one reproducible-build closure; the
regenerate-lockfiles workflow refreshes them together. This script rewrites a
single

    <distro> = "@sha256:<hex>"

line inside the BASE_IMAGE_DIGESTS { default = { ... } } block and leaves the rest
of the file byte-for-byte unchanged. It is scoped to that block (the same approach
as ansible/scripts/validate_lockfiles.sh) so it never rewrites an unrelated
'<distro> = "..."' line elsewhere in the HCL.
"""

import argparse
import re
import sys


def update_digest(text: str, distro: str, digest: str) -> str:
    """Return `text` with the BASE_IMAGE_DIGESTS entry for `distro` set to `@<digest>`."""
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", digest):
        raise ValueError(f"digest must be 'sha256:<64 hex chars>', got: {digest!r}")

    block = re.search(r'variable\s+"BASE_IMAGE_DIGESTS"\s*\{.*?\n\}', text, re.S)
    if not block:
        raise ValueError("BASE_IMAGE_DIGESTS block not found in bake file")

    entry = re.compile(
        r"(^\s*" + re.escape(distro) + r'\s*=\s*")@sha256:[0-9a-f]+(")',
        re.M,
    )
    block_text = block.group(0)
    if not entry.search(block_text):
        raise ValueError(f"no BASE_IMAGE_DIGESTS entry for distro {distro!r}")

    new_block = entry.sub(r"\1@" + digest + r"\2", block_text, count=1)
    start, end = block.span(0)
    return text[:start] + new_block + text[end:]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bake_file", help="path to docker/docker-bake.hcl")
    parser.add_argument("--distro", required=True, help="humble | jazzy")
    parser.add_argument("--digest", required=True, help="sha256:<hex> (no leading '@')")
    args = parser.parse_args(argv)

    with open(args.bake_file, encoding="utf-8") as handle:
        text = handle.read()

    try:
        updated = update_digest(text, args.distro, args.digest)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    with open(args.bake_file, "w", encoding="utf-8") as handle:
        handle.write(updated)

    print(f"Updated {args.distro} base image digest in {args.bake_file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
