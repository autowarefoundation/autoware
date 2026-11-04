#!/usr/bin/env bash

set -o pipefail -eux

# shellcheck disable=SC2086
ansible-test units --color -v --docker default ${COVERAGE:+"$COVERAGE"} ${CHANGED:+"$CHANGED"}

