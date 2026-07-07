#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ANSIBLE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$ANSIBLE_DIR/.." && pwd)"
BAKE_FILE="${REPO_ROOT}/docker/docker-bake.hcl"

validate_lockfile() {
    local lockfile="$1"

    if [[ ! -f $lockfile ]]; then
        echo "Error: Lockfile not found: $lockfile" >&2
        return 1
    fi

    # cspell:ignore isinstance fullmatch
    if ! python3 -c "
import re, sys, yaml
with open('$lockfile') as f:
    data = yaml.safe_load(f)
if not isinstance(data, dict):
    sys.exit('Error: Lockfile must be a YAML mapping (dict)')
date = data.get('ros_snapshot_date')
if not isinstance(date, str) or not re.fullmatch(r'\d{4}-\d{2}-\d{2}', date):
    sys.exit('Error: ros_snapshot_date must be a YYYY-MM-DD string')
for key in ('apt_pins', 'pip_pins', 'ros_overrides'):
    section = data.get(key, {})
    if section is None:
        section = {}
    if not isinstance(section, dict):
        sys.exit(f'Error: {key} must be a mapping')
    for pkg, ver in section.items():
        if ver is not None and not isinstance(ver, (str, int, float)):
            sys.exit(f'Error: {key}[{pkg}] must be a scalar version')
"; then
        echo "Error: Invalid lockfile format in $lockfile" >&2
        return 1
    fi

    echo "Validated: $lockfile"
    return 0
}

# Bind the two halves of the reproducible-build closure that live in separate
# files: the per-distro base-image digests in docker/docker-bake.hcl
# (BASE_IMAGE_DIGESTS) and the per-distro lockfiles here in ansible/vars/. A
# locked build of a distro needs BOTH, so a distro present in one but not the
# other silently defeats reproducibility (a lockfile with no digest builds on an
# unpinned base; a digest with no lockfile is stale). Fail if the two sets differ.
validate_digest_coverage() {
    if [[ ! -f $BAKE_FILE ]]; then
        echo "Warning: $BAKE_FILE not found; skipping digest/lockfile coverage check." >&2
        return 0
    fi

    # cspell:ignore findall
    python3 - "$BAKE_FILE" "$ANSIBLE_DIR" <<'PY'
import glob, os, re, sys

bake_file, ansible_dir = sys.argv[1], sys.argv[2]

# Extract the distro keys from the BASE_IMAGE_DIGESTS { default = { ... } } block.
text = open(bake_file).read()
m = re.search(r'variable\s+"BASE_IMAGE_DIGESTS"\s*\{(.*?)\n\}', text, re.S)
if not m:
    sys.exit(f"Error: BASE_IMAGE_DIGESTS block not found in {bake_file}")
digest_distros = set(re.findall(r'^\s*([A-Za-z0-9_]+)\s*=\s*"', m.group(1), re.M))

# Distros that have at least one lockfile (a digest is a multi-arch manifest).
lock_distros = set()
for path in glob.glob(os.path.join(ansible_dir, "vars", "locked-versions-*.yaml")):
    fm = re.match(r"locked-versions-(.+)-(amd64|arm64)\.yaml$", os.path.basename(path))
    if fm:
        lock_distros.add(fm.group(1))

missing_digest = lock_distros - digest_distros
missing_lock = digest_distros - lock_distros
errors = []
if missing_digest:
    errors.append("lockfile(s) with no BASE_IMAGE_DIGESTS entry (locked build would use an "
                  f"unpinned base): {', '.join(sorted(missing_digest))}")
if missing_lock:
    errors.append("BASE_IMAGE_DIGESTS entries with no lockfile (stale digest): "
                  f"{', '.join(sorted(missing_lock))}")
if errors:
    sys.exit("Error: docker-bake.hcl digests and ansible lockfiles are out of sync:\n  - "
             + "\n  - ".join(errors))

print(f"Validated: digest/lockfile coverage in sync ({', '.join(sorted(digest_distros))})")
PY
}

main() {
    local exit_code=0
    local found=false
    local files=("$@")
    # Only cross-check digest coverage on a full-repo run (no explicit files);
    # a single-file call (e.g. from generate_ansible_lockfile.sh) validates format only.
    local check_coverage=false
    [[ $# -eq 0 ]] && check_coverage=true

    if [[ ${#files[@]} -eq 0 ]]; then
        files=("${ANSIBLE_DIR}"/vars/locked-versions-*.yaml)
    fi

    for lockfile in "${files[@]}"; do
        if [[ -f $lockfile ]]; then
            found=true
            if ! validate_lockfile "$lockfile"; then
                exit_code=1
            fi
        fi
    done

    if [[ $found == "false" ]]; then
        echo "Error: No lock files found in ${ANSIBLE_DIR}/vars/" >&2
        exit 1
    fi

    if [[ $check_coverage == "true" ]]; then
        if ! validate_digest_coverage; then
            exit_code=1
        fi
    fi

    return $exit_code
}

main "$@"
