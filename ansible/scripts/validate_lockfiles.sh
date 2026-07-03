#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ANSIBLE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

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

main() {
    local exit_code=0
    local found=false
    local files=("$@")

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

    return $exit_code
}

main "$@"
