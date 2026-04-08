#!/bin/bash
set -eo pipefail

function build_and_clean() {
    local ccache_dir=$1
    local install_base=$2
    local colcon_build_args=$3
    # Base paths for colcon package discovery (default: /autoware/src only)
    local base_paths="${4:-/autoware/src}"

    local taskset_cmd=""
    local num_cores
    num_cores=$(nproc)
    if [ "$num_cores" -gt 1 ]; then
        taskset_cmd="taskset --cpu-list 0-$((num_cores - 2))"
    fi

    # Runtime patches (best-effort, don't fail the build)
    set +e

    # M_PIf/M_PI_2f/M_PI_4f: C23 constants not available in GCC 11 (RHEL 9)
    grep -rl 'M_PIf\b' /autoware/src/ --include='*.hpp' --include='*.cpp' 2>/dev/null | \
        while read -r f; do
            if ! grep -q 'ifndef M_PIf' "$f" 2>/dev/null; then
                sed -i '1i #ifndef M_PIf\n#define M_PIf static_cast<float>(M_PI)\n#endif\n#ifndef M_PI_2f\n#define M_PI_2f static_cast<float>(M_PI_2)\n#endif\n#ifndef M_PI_4f\n#define M_PI_4f static_cast<float>(M_PI_4)\n#endif' "$f" 2>/dev/null
            fi
        done

    # COLCON_IGNORE CUDA packages in bind-mounted source
    for pkg in autoware_cuda_dependency_meta autoware_cuda_utils autoware_cuda_pointcloud_preprocessor \
      autoware_tensorrt_common autoware_tensorrt_plugins autoware_tensorrt_yolox autoware_tensorrt_classifier \
      autoware_tensorrt_bevdet autoware_tensorrt_bevformer autoware_tensorrt_vad \
      autoware_lidar_centerpoint autoware_lidar_transfusion autoware_lidar_frnet \
      autoware_lidar_apollo_instance_segmentation autoware_bevfusion autoware_bytetrack \
      autoware_camera_streampetr autoware_ptv3 autoware_simpl_prediction \
      autoware_image_projection_based_fusion autoware_traffic_light_classifier \
      autoware_traffic_light_fine_detector autoware_ground_segmentation_cuda \
      autoware_probabilistic_occupancy_grid_map autoware_calibration_status_classifier \
      autoware_diffusion_planner autoware_shape_estimation \
      bevdet_vendor trt_batched_nms cuda_blackboard; do
        find /autoware/src -name "$pkg" -type d -exec touch {}/COLCON_IGNORE \; 2>/dev/null
    done
    # Duplicate autoware_agnocast_wrapper fix
    touch /autoware/src/universe/autoware_universe/common/autoware_agnocast_wrapper/COLCON_IGNORE 2>/dev/null

    # Suppress tl_expected deprecation warning (system tl/expected.hpp exists, but
    # the #warning fires before the redirect and -Werror=cpp turns it into an error)
    local tl_hdr="/opt/autoware/include/tl_expected/expected.hpp"
    if [ -f "$tl_hdr" ] && grep -q '#warning' "$tl_hdr"; then
        sed -i '/#warning/,/NOLINT/d; /#pragma message/,/NOLINT/d' "$tl_hdr" 2>/dev/null
    fi

    # Boost 1.75 compat: algorithms/correct.hpp and algorithms/is_valid.hpp
    # don't include strategies/cartesian.hpp, causing missing strategy_converter
    # and default_strategy errors for custom Point types (Eigen-based Point2d).
    # Patch these headers to include the full cartesian strategy set.
    for bhpp in \
      /usr/include/boost/geometry/algorithms/correct.hpp \
      /usr/include/boost/geometry/algorithms/is_valid.hpp \
      /usr/include/boost/geometry/algorithms/detail/is_valid/polygon.hpp; do
        if [ -f "$bhpp" ] && ! grep -q 'strategies/cartesian.hpp' "$bhpp"; then
            sed -i '1i #include <boost/geometry/strategies/cartesian.hpp>  // RHEL Boost 1.75 compat' "$bhpp" 2>/dev/null
        fi
    done

    # Patch: boost::geometry::is_valid() fails on Boost < 1.77 with Eigen-based Point2d
    # Replace with a simpler polygon size check on older Boost
    for rcpf in \
      /autoware/src/core/autoware_utils/autoware_utils_geometry/src/geometry/random_concave_polygon.cpp \
      /autoware/src/universe/autoware_universe/common/autoware_universe_utils/src/geometry/random_concave_polygon.cpp; do
        if [ -f "$rcpf" ]; then
            sed -i 's|!is_convex(poly) && boost::geometry::is_valid(poly) && poly.outer().size() != vertices|!is_convex(poly) \&\& poly.outer().size() > 3 \&\& poly.outer().size() != vertices /* Boost 1.75 compat: is_valid removed */|' "$rcpf" 2>/dev/null
        fi
    done

    set -eo pipefail

    # shellcheck disable=SC2086
    du -sh "$ccache_dir" && ccache -s

    # Multiple passes for dependency ordering
    for pass in 1 2 3; do
        # shellcheck disable=SC2086
        $taskset_cmd colcon build --cmake-args \
            " -Wno-dev" \
            " --no-warn-unused-cli" \
            " -DPython3_EXECUTABLE=/usr/bin/python3" \
            " -DBUILD_TESTING=OFF" \
            " -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF" \
            " -DCMAKE_CXX_FLAGS=-Wno-error=cpp" \
            --base-paths $base_paths \
            --merge-install \
            --install-base "$install_base" \
            --mixin release compile-commands ccache \
            $colcon_build_args 2>&1 || true
        local failed
        # shellcheck disable=SC2086
        failed=$(colcon list --base-paths $base_paths --packages-select-build-failed 2>/dev/null | wc -l)
        echo "=== Pass $pass: $failed packages failed ==="
        [ "$failed" -eq 0 ] && break
        # Remove build dirs of failed packages so colcon retries them
        # shellcheck disable=SC2086
        colcon list --base-paths $base_paths --packages-select-build-failed --paths-only 2>/dev/null | while read -r p; do
            rm -rf "/autoware/build/$(basename "$p")"
        done
    done

    du -sh "$ccache_dir" && ccache -s
    rm -rf /autoware/build /autoware/log
}

build_and_clean "$@"
