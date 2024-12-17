#!/bin/bash -e

: "${WEBAUTO_CI_ML_PACKAGES_PATH:=}"

: "${ML_MODELS_PATH:=}"

if [ -n "$WEBAUTO_CI_ML_PACKAGES_PATH" ] && [ -n "$ML_MODELS_PATH" ]; then
    sudo mkdir -p "$(dirname "$ML_MODELS_PATH")"
    sudo chown "$(whoami):" "$(dirname "$ML_MODELS_PATH")"
    cp -r "$WEBAUTO_CI_ML_PACKAGES_PATH" "$ML_MODELS_PATH"
    find "$ML_MODELS_PATH" -type d -exec chmod 777 {} \;
    find "$ML_MODELS_PATH" -type f -exec chmod 644 {} \;

    echo "The following ML models have been deployed:"
    ls "$ML_MODELS_PATH"
fi
