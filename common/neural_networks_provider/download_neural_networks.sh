#!/bin/sh -eu

# Copyright (c) 2021-2022, Arm Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The variables "EXEC_DOWNLOAD", "PREFIX_DIR" and "CMAKE_SYSTEM_PROCESSOR" are
# passed to this script by the CmakeLists.txt of the package itself.

NETWORKS_FILE="$PREFIX_DIR"/src/networks-"$CMAKE_SYSTEM_PROCESSOR".tar.gz
S3_BUCKET=https://autoware-modelzoo.s3.us-east-2.amazonaws.com/

# Function to download-extract the networks artifact
download_artifact() {
    curl -sS -o "$NETWORKS_FILE" "$S3_BUCKET"networks-"$CMAKE_SYSTEM_PROCESSOR".tar.gz
    tar -zxvf "$NETWORKS_FILE" -C "$PREFIX_DIR"/src/networks
}

if [ "$EXEC_DOWNLOAD" = "TRUE" ]; then

    # If the networks artifact is not present, simply download
    if [ ! -f "$NETWORKS_FILE" ]; then
        download_artifact

    # If the networks artifact is already present, check the MD5 hash and, if needed, re-download it
    else
        OLD_HASH=$(md5sum "$NETWORKS_FILE" | cut -d ' ' -f 1)
        NEW_HASH=$(curl -sS "$S3_BUCKET"networks-"$CMAKE_SYSTEM_PROCESSOR".md5)
        if [ "$OLD_HASH" != "$NEW_HASH" ]; then
            download_artifact
        fi
    fi
fi
