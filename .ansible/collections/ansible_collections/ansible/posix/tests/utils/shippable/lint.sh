#!/usr/bin/env bash

set -o pipefail -eux

echo "${PATH/\~/${HOME}}"
echo "${HOME}"
command -v ansible

pip install --upgrade --user pip
pip install --upgrade --user ansible-lint

# To specify additional options, you can specify them into .ansible-lint file.
PATH="${PATH/\~/${HOME}}" ansible-lint
