#!/bin/bash -e

: "${WEBAUTO_CI_SOURCE_PATH:?is not set}"
: "${WEBAUTO_CI_DEBUG_BUILD:?is not set}"

: "${AUTOWARE_PATH:?is not set}"
: "${CCACHE_DIR:=}"
: "${CCACHE_SIZE:=1G}"
: "${PARALLEL_WORKERS:=4}"

# get installed ros distro
# shellcheck disable=SC2012
ROS_DISTRO=$(ls -1 /opt/ros | head -1)

# CARET build
if [ "$WEBAUTO_CI_BUILD_OPTION_CARET_ENABLED" = "ENABLED" ]; then
    echo "CARET ENABLED"

    cd "$HOME"
    rm -rf ros2_caret_ws

    # download CARET
    echo "===== GET CARET ====="
    CARET_VERSION="rc/v0.5.2_for_evaluator"
    export GITHUB_TOKEN="$WEBAUTO_CI_GITHUB_TOKEN"
    git clone https://github.com/tier4/caret.git ros2_caret_ws
    cd ros2_caret_ws
    git checkout "$CARET_VERSION"

    # setup CARET
    echo "===== Setup CARET ====="
    mkdir src
    vcs import src <caret.repos
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    ./setup_caret.sh -c

    # build caret
    echo "===== Build CARET ====="
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

fi

# For incremental builds, the source files used in previous builds are already in place.
# Delete any files that have been removed from the new source, except for files specified in .gitignore.
# Also, to take advantage of incremental builds, preserve the timestamps of files with the same checksum.
src=$(mktemp -p /tmp -d src.XXXXX)
cp -rfT "$WEBAUTO_CI_SOURCE_PATH" "$src"
# shellcheck disable=SC2016
find "$src" -name '.gitignore' -printf '%P\0' | xargs -0 -I {} sh -c "sed -n "'s/^!//gp'" $src/{} > $src/"'$(dirname {})'"/.rsync-include"
# shellcheck disable=SC2016
find "$src" -name '.gitignore' -printf '%P\0' | xargs -0 -I {} sh -c "sed -n "'/^[^!]/p'" $src/{} > $src/"'$(dirname {})'"/.rsync-exclude"
rsync -rlpc -f":+ .rsync-include" -f":- .rsync-exclude" --del "$src"/ "$AUTOWARE_PATH"
# The `src` directory is excluded from the root .gitignore and must be synchronized separately.
rsync -rlpc -f":+ .rsync-include" -f":- .rsync-exclude" --del "$src"/src/ "$AUTOWARE_PATH"/src
# `.rsync-include` and `.rsync-exclude` must be included in the output of this phase for reference in the next incremental build.
# These files are removed in the ecu-system-setup phase.
#find "$AUTOWARE_PATH" \( -name ".rsync-include" -or -name ".rsync-exclude" \) -print0 | xargs -0 rm
rm -rf "$src"

chmod 755 "$AUTOWARE_PATH"
cd "$AUTOWARE_PATH"

if [ -n "$CCACHE_DIR" ]; then
    mkdir -p "$CCACHE_DIR"
    export USE_CCACHE=1
    export CCACHE_DIR="$CCACHE_DIR"
    export CC="/usr/lib/ccache/gcc"
    export CXX="/usr/lib/ccache/g++"
    ccache -M "$CCACHE_SIZE"
fi

sudo -E apt-get -y update

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro "$ROS_DISTRO"

# CARET setup
ADDITIONAL_OPTIONS=""
if [ "$WEBAUTO_CI_BUILD_OPTION_CARET_ENABLED" = "ENABLED" ]; then
    echo "===== Modify ament_cmake_auto as workaround ====="
    backup_date="$(date +"%Y%m%d_%H%M%S")"
    cd /opt/ros/humble/share/ament_cmake_auto/cmake &&
        sudo cp ament_auto_add_executable.cmake ament_auto_add_executable.cmake_"$backup_date" &&
        sudo cp ament_auto_add_library.cmake ament_auto_add_library.cmake_"$backup_date" &&
        sudo sed -i -e 's/SYSTEM//g' ament_auto_add_executable.cmake &&
        sudo sed -i -e 's/SYSTEM//g' ament_auto_add_library.cmake

    # cspell: ignore libtracetools
    echo "===== Modify pcl_ros (libtracetools.so) as workaround ====="
    cd /opt/ros/humble/share/pcl_ros/cmake &&
        sudo cp export_pcl_rosExport.cmake export_pcl_rosExport.cmake_"$backup_date" &&
        sudo sed -i -e 's/\/opt\/ros\/humble\/lib\/libtracetools.so;//g' export_pcl_rosExport.cmake

    echo "===== Modify pcl_ros (rclcpp) as workaround ====="
    cd /opt/ros/humble/share/pcl_ros/cmake &&
        sudo cp export_pcl_rosExport.cmake export_pcl_rosExport.cmake_"$backup_date"_2 &&
        sudo sed -i -e 's/\/opt\/ros\/humble\/include\/rclcpp;//g' export_pcl_rosExport.cmake

    cd "$AUTOWARE_PATH"
    rm -f caret_topic_filter.bash
    wget https://raw.githubusercontent.com/tier4/caret_report/main/sample_autoware/caret_topic_filter.bash

    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    # shellcheck disable=SC1091
    source "$HOME/ros2_caret_ws/install/local_setup.sh"
    echo "===== Finish CARET SETUP ====="

    ADDITIONAL_OPTIONS="--packages-skip rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle"
fi
cd "$AUTOWARE_PATH"

[[ $WEBAUTO_CI_DEBUG_BUILD == "true" ]] && build_type="RelWithDebInfo" || build_type="Release"

# shellcheck disable=SC2086
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE="$build_type" -DCMAKE_CXX_FLAGS="-w" -DBUILD_TESTING=off \
    --catkin-skip-building-tests \
    --executor parallel \
    --parallel-workers "$PARALLEL_WORKERS" \
    $ADDITIONAL_OPTIONS

if [ "$WEBAUTO_CI_BUILD_OPTION_CARET_ENABLED" = "ENABLED" ]; then
    echo "===== Check CARET SETUP ====="
    # shellcheck disable=SC1090,SC1091,SC2015
    source "/opt/ros/${ROS_DISTRO}/setup.sh" &&
        source "$HOME/ros2_caret_ws/install/local_setup.sh" &&
        ros2 caret check_caret_rclcpp ./ ||
        (echo "CARET build is failed" && exit 1)
fi
