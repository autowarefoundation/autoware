#!/usr/bin/env bash
set -euo pipefail

# Scenario Simulator v2 setup script for Autoware training workspace.
#
# Purpose:
#   - install required system dependencies
#   - import Scenario Simulator v2 from simulator.repos if missing
#   - install zmqpp if it is not available
#   - apply required compatibility patches
#   - clean old Scenario Simulator build artifacts
#   - clean broken install package folders
#   - build scenario_test_runner and random_test_runner
#
# Usage:
#   cd ~/nit-training/autoware-training
#   ./setup_scenario_simulator.sh
#
# Optional:
#   WORKSPACE=/path/to/workspace ./setup_scenario_simulator.sh
#   RESET_SCENARIO_SIMULATOR=1 ./setup_scenario_simulator.sh

WORKSPACE="${WORKSPACE:-$(pwd)}"
SCENARIO_DIR="${WORKSPACE}/src/simulator/scenario_simulator"
RESET_SCENARIO_SIMULATOR="${RESET_SCENARIO_SIMULATOR:-0}"

log() {
  echo
  echo "============================================================"
  echo "[scenario-simulator-setup] $*"
  echo "============================================================"
}

fail() {
  echo "[ERROR] $*" >&2
  exit 1
}

require_file() {
  local path="$1"
  local message="$2"

  if [ ! -f "$path" ]; then
    fail "$message"
  fi
}

source_setup_file() {
  local setup_file="$1"

  if [ ! -f "$setup_file" ]; then
    fail "Setup file does not exist: $setup_file"
  fi

  # ROS 2 setup files may reference variables that are not defined.
  # This conflicts with 'set -u', so temporarily disable nounset.
  set +u
  source "$setup_file"
  set -u
}

get_expected_scenario_simulator_version() {
  awk '
    /simulator\/scenario_simulator:/ { found=1 }
    found && /version:/ { print $2; exit }
  ' "${WORKSPACE}/simulator.repos"
}

log "Checking workspace"

if [ ! -d "$WORKSPACE" ]; then
  fail "Workspace directory does not exist: $WORKSPACE"
fi

cd "$WORKSPACE"

require_file "/opt/ros/humble/setup.bash" "ROS 2 Humble was not found at /opt/ros/humble/setup.bash"
require_file "${WORKSPACE}/simulator.repos" "simulator.repos was not found in ${WORKSPACE}"

EXPECTED_SCENARIO_VERSION="$(get_expected_scenario_simulator_version)"

if [ -z "$EXPECTED_SCENARIO_VERSION" ]; then
  fail "Could not read scenario_simulator version from simulator.repos"
fi

echo "Expected Scenario Simulator version from simulator.repos:"
echo "  ${EXPECTED_SCENARIO_VERSION}"

log "Installing required Ubuntu packages"

sudo apt update

sudo apt install -y \
  git \
  build-essential \
  cmake \
  pkg-config \
  python3-vcstool \
  python3-colcon-common-extensions \
  python3-rosdep \
  libzmq3-dev \
  libsodium-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libembree-dev

log "Importing Scenario Simulator v2 if missing"

mkdir -p "${WORKSPACE}/src"

if [ ! -d "${SCENARIO_DIR}/.git" ]; then
  vcs import "${WORKSPACE}/src" < "${WORKSPACE}/simulator.repos"
else
  echo "Scenario Simulator repository already exists:"
  echo "  ${SCENARIO_DIR}"

  if [ "$RESET_SCENARIO_SIMULATOR" = "1" ]; then
    echo "RESET_SCENARIO_SIMULATOR=1 is set. Resetting local Scenario Simulator changes."
    git -C "${SCENARIO_DIR}" reset --hard
    git -C "${SCENARIO_DIR}" clean -fd
  elif [ -n "$(git -C "${SCENARIO_DIR}" status --porcelain)" ]; then
    echo "WARNING: Local changes detected in ${SCENARIO_DIR}."
    echo "         They will not be reset because RESET_SCENARIO_SIMULATOR is not set."
  fi
fi

CURRENT_SCENARIO_COMMIT="$(git -C "${SCENARIO_DIR}" rev-parse HEAD)"

echo "Current Scenario Simulator commit:"
git -C "${SCENARIO_DIR}" log -1 --format="  %H %h %ci %s"

if [ "$CURRENT_SCENARIO_COMMIT" != "$EXPECTED_SCENARIO_VERSION" ]; then
  fail "Scenario Simulator commit does not match simulator.repos. Please import/update the repository according to simulator.repos before running this setup script."
fi

log "Installing zmqpp from source if needed"

if [ ! -f /usr/local/include/zmqpp/zmqpp.hpp ] || ! ldconfig -p | grep -q "libzmqpp"; then
  rm -rf /tmp/zmqpp
  git clone https://github.com/zeromq/zmqpp.git /tmp/zmqpp
  make -C /tmp/zmqpp -j"$(nproc)"
  sudo make -C /tmp/zmqpp install
  sudo ldconfig
else
  echo "zmqpp already available."
fi

log "Checking Embree installation"

require_file "/usr/include/embree3/rtcore.h" "Embree header not found. Expected /usr/include/embree3/rtcore.h after installing libembree-dev."

log "Applying compatibility patch for simulation_interface if needed"

SIM_CMAKE="${SCENARIO_DIR}/simulation/simulation_interface/CMakeLists.txt"

if [ -f "$SIM_CMAKE" ]; then
  python3 - "$SIM_CMAKE" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()
original = text

# Some older revisions include zmqpp headers but do not explicitly link libzmqpp.
# The patch is intentionally idempotent.
if "/usr/local/include" not in text:
    text = text.replace(
        "include_directories(\n  include\n  ${CMAKE_BINARY_DIR}\n)",
        "include_directories(\n  include\n  ${CMAKE_BINARY_DIR}\n  /usr/local/include\n)"
    )

if "/usr/local/lib/libzmqpp.so" not in text:
    text = text.replace(
        "target_link_libraries(simulation_interface\n  ${PROTOBUF_LIBRARY}\n  pthread\n  sodium\n  zmq\n)",
        "target_link_libraries(simulation_interface\n  ${PROTOBUF_LIBRARY}\n  pthread\n  sodium\n  /usr/local/lib/libzmqpp.so\n  zmq\n)"
    )

    text = text.replace(
        "target_link_libraries(test_conversion simulation_interface)",
        "target_link_libraries(test_conversion\n  simulation_interface\n  /usr/local/lib/libzmqpp.so\n  zmq\n)"
    )

if text != original:
    path.write_text(text)
    print(f"Patched {path}")
else:
    print(f"No simulation_interface CMake patch needed for {path}")
PY
else
  echo "simulation_interface CMakeLists.txt not found; skipping patch."
fi

log "Applying compatibility patch for cpp_mock_scenarios if needed"

CPP_MOCK_CMAKE="${SCENARIO_DIR}/mock/cpp_mock_scenarios/CMakeLists.txt"

if [ -f "$CPP_MOCK_CMAKE" ]; then
  if grep -q '\${PROJECT_NAME}-extras.cmake' "$CPP_MOCK_CMAKE"; then
    sed -i 's|\${PROJECT_NAME}-extras.cmake|\${PROJECT_NAME}_ament_cmake-extras.cmake|g' "$CPP_MOCK_CMAKE"
    echo "Patched cpp_mock_scenarios CONFIG_EXTRAS."
  else
    echo "No cpp_mock_scenarios patch needed."
  fi
else
  echo "cpp_mock_scenarios CMakeLists.txt not found; skipping patch."
fi

log "Cleaning Scenario Simulator build artifacts"

PACKAGES_TO_CLEAN=(
  traffic_simulator_msgs
  scenario_simulator_exception
  simulation_interface
  concealer
  geometry
  simple_sensor_simulator
  traffic_simulator
  openscenario_interpreter
  openscenario_preprocessor
  openscenario_visualization
  behavior_tree_plugin
  scenario_test_runner
  random_test_runner
  cpp_mock_scenarios
)

for package in "${PACKAGES_TO_CLEAN[@]}"; do
  rm -rf "${WORKSPACE}/build/${package}" "${WORKSPACE}/install/${package}"
done

rm -rf "${WORKSPACE}/log"

log "Removing broken install package folders if any"

if [ -d "${WORKSPACE}/install" ]; then
  for dir in "${WORKSPACE}"/install/*; do
    [ -d "$dir" ] || continue

    pkg="$(basename "$dir")"

    if [ ! -f "$dir/share/$pkg/package.xml" ]; then
      echo "Removing broken install package: $pkg"
      rm -rf "${WORKSPACE}/build/$pkg" "${WORKSPACE}/install/$pkg" "${WORKSPACE}/log/$pkg"
    fi
  done
fi

log "Building traffic_simulator_msgs first"

cd "$WORKSPACE"
source_setup_file /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select traffic_simulator_msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

log "Building Scenario Simulator runners"

source_setup_file /opt/ros/humble/setup.bash
source_setup_file "${WORKSPACE}/install/setup.bash"

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="/usr/local/lib:${LIBRARY_PATH:-}"
export CPLUS_INCLUDE_PATH="/usr/local/include:${CPLUS_INCLUDE_PATH:-}"

colcon build --symlink-install \
  --packages-up-to scenario_test_runner random_test_runner \
  --allow-overriding autoware_common_msgs autoware_planning_msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

log "Verifying installed packages"

source_setup_file /opt/ros/humble/setup.bash
source_setup_file "${WORKSPACE}/install/setup.bash"

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

ros2 pkg list | grep -E "scenario_test_runner|random_test_runner|simple_sensor_simulator|openscenario_interpreter|traffic_simulator"

log "Setup completed successfully"
