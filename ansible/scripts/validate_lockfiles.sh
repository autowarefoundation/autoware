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

    # YAML syntax check and ensure it's a flat dict
    # cspell:ignore isinstance
    if ! python3 -c "
import yaml, sys
with open('$lockfile') as f:
    data = yaml.safe_load(f)
if not isinstance(data, dict):
    print('Error: Lockfile must be a YAML dict', file=sys.stderr)
    sys.exit(1)
for key, val in data.items():
    if isinstance(val, dict):
        print(f'Error: Nested dict found at key \"{key}\"', file=sys.stderr)
        sys.exit(1)
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

    for lockfile in "${ANSIBLE_DIR}"/vars/locked-versions-*.yaml; do
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
