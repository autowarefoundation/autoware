#!/bin/bash -e

: "${WEBAUTO_CI_SOURCE_PATH:?is not set}"
: "${WEBAUTO_CI_DEBUG_BUILD:?is not set}"

: "${AUTOWARE_PATH:?is not set}"
: "${CCACHE_DIR:=}"
: "${CCACHE_SIZE:=1G}"
: "${PARALLEL_WORKERS:=4}"

sudo mkdir "$AUTOWARE_PATH"
sudo chown "$(whoami)": "$AUTOWARE_PATH"
cd "$WEBAUTO_CI_SOURCE_PATH"
cp -r src .webauto-ci.* "$AUTOWARE_PATH"
cd "$AUTOWARE_PATH"

if [ -n "$CCACHE_DIR" ]; then
    mkdir -p "$CCACHE_DIR"
    export USE_CCACHE=1
    export CCACHE_DIR="$CCACHE_DIR"
    export CC="/usr/lib/ccache/gcc"
    export CXX="/usr/lib/ccache/g++"
    ccache -M "$CCACHE_SIZE"
fi

# install xmlschema<4.0.0 before rosdep install as workaround for scenario_simulator_v2
sudo pip3 install xmlschema==3.4.5

sudo -E apt-get -y update

# shellcheck disable=SC2012
ROS_DISTRO=$(ls -1 /opt/ros | head -1)
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro "$ROS_DISTRO"

[[ $WEBAUTO_CI_DEBUG_BUILD == "true" ]] && build_type="RelWithDebInfo" || build_type="Release"

colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE="$build_type" -DBUILD_TESTING=off -Wno-dev --no-warn-unused-cli \
    --symlink-install \
    --catkin-skip-building-tests \
    --executor parallel \
    --parallel-workers "$PARALLEL_WORKERS"

sudo -E apt-get -y autoremove
