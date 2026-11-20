#!/usr/bin/env bash

# Following logic in https://github.com/ansible-collections/collection_template/blob/main/.github/workflows/ansible-test.yml
set -o pipefail -eux

if [ "${BASE_BRANCH:-}" ]; then
    base_branch="origin/${BASE_BRANCH}"
else
    base_branch=""
fi

# Run sanity tests inside a Docker container.
# The docker container has all the pinned dependencies that are
# required and all Python versions Ansible supports.

# See the documentation for the following GitHub action on
# https://github.com/ansible-community/ansible-test-gh-action/blob/main/README.md

# shellcheck disable=SC2086
ansible-test sanity --color -v --junit ${COVERAGE:+"$COVERAGE"} ${CHANGED:+"$CHANGED"} \
    --docker --base-branch "${base_branch}" --allow-disabled
