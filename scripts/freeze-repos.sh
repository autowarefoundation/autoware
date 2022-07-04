#!/usr/bin/env bash

set -euC

function update_repository_version {
    local -r repos_file="$1"
    local -r repository="$2"

    url=$(yq ".repositories[] | select(key == \"$repository\").url" "$repos_file")
    version=$(yq ".repositories[] | select(key == \"$repository\").version" "$repos_file")

    tag_hash=$(git ls-remote --tags "$url" "$version" | awk '{print $1}')
    branch_hash=$(git ls-remote --heads "$url" "$version" | awk '{print $1}')

    if [ -n "$tag_hash" ]; then
        new_version="$tag_hash"
    elif [ -n "$branch_hash" ]; then
        new_version="$branch_hash"
    else
        new_version="$version"
    fi

    echo "$repository: $new_version"
    yq --inplace ".repositories.\"$repository\".version = \"$new_version\"" "$repos_file"
}

function get_repositories_in_repos() {
    local -r repos_file="$1"

    yq '.repositories' "$repos_file" | grep -E "^[a-zA-Z]+" | sed "s/:.*//g"
}

function freeze_repos_file {
    local -r repos_file="$1"

    local -r repositories=$(get_repositories_in_repos "$repos_file")
    if [ -z "$repositories" ]; then
        echo "no repository found"
        exit 1
    fi

    for repository in $repositories; do
        update_repository_version "$repos_file" "$repository"
    done
}

function freeze_all_repos_files {
    for repos_file in *.repos; do
        freeze_repos_file "$repos_file"
    done
}

function main {
    if ! (command -v yq >/dev/null 2>&1); then
        echo "yq not found"
        exit 1
    fi

    freeze_all_repos_files

    git add "*.repos"
    git commit -s -m "chore(release): freeze repos files"
}

if [[ ${BASH_SOURCE[0]} == "${0}" ]]; then
    main "$@"
fi
