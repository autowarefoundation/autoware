#!/usr/bin/env bash
# cspell:ignore
# shellcheck disable=SC1090,SC1091

run_scenario_simulator() {
    echo -e "\e[32mRunning scenario simulator...\e[0m"

    # Prompt if no scenario is provided
    if [ -z "$SCENARIO" ]; then
        echo -e "\e[33mNo scenario provided, using default sample scenario...\e[0m"
    fi

    # Set default values if not provided
    LAUNCH_AUTOWARE=${LAUNCH_AUTOWARE:-false}
    LAUNCH_RVIZ=${LAUNCH_RVIZ:-false}
    ARCHITECTURE_TYPE=${ARCHITECTURE_TYPE:-awf/universe/20240605}
    SENSOR_MODEL=${SENSOR_MODEL:-sample_sensor_kit}
    VEHICLE_MODEL=${VEHICLE_MODEL:-sample_vehicle}
    INITIALIZE_DURATION=${INITIALIZE_DURATION:-90}
    GLOBAL_FRAME_RATE=${GLOBAL_FRAME_RATE:-30}
    OUTPUT_DIRECTORY=${OUTPUT_DIRECTORY:-/autoware/scenario-sim/output}
    SCENARIO=${SCENARIO:-$(ros2 pkg prefix --share scenario_test_runner)/scenario/sample.yaml}
    GLOBAL_TIMEOUT=${GLOBAL_TIMEOUT:-120}
    RECORD=${RECORD:-false}
    USE_SIM_TIME=${USE_SIM_TIME:-false}

    # Print all variables
    echo "LAUNCH_AUTOWARE: $LAUNCH_AUTOWARE"
    echo "LAUNCH_RVIZ: $LAUNCH_RVIZ"
    echo "ARCHITECTURE_TYPE: $ARCHITECTURE_TYPE"
    echo "SENSOR_MODEL: $SENSOR_MODEL"
    echo "VEHICLE_MODEL: $VEHICLE_MODEL"
    echo "INITIALIZE_DURATION: $INITIALIZE_DURATION"
    echo "GLOBAL_FRAME_RATE: $GLOBAL_FRAME_RATE"
    echo "OUTPUT_DIRECTORY: $OUTPUT_DIRECTORY"
    echo "SCENARIO: $SCENARIO"
    echo "GLOBAL_TIMEOUT: $GLOBAL_TIMEOUT"
    echo "RECORD: $RECORD"
    echo "USE_SIM_TIME: $USE_SIM_TIME"

    # Launch scenario test runner
    ros2 launch scenario_test_runner scenario_test_runner.launch.py \
        launch_autoware:="$LAUNCH_AUTOWARE" \
        launch_rviz:="$LAUNCH_RVIZ" \
        architecture_type:="$ARCHITECTURE_TYPE" \
        sensor_model:="$SENSOR_MODEL" \
        vehicle_model:="$VEHICLE_MODEL" \
        initialize_duration:="$INITIALIZE_DURATION" \
        global_frame_rate:="$GLOBAL_FRAME_RATE" \
        output_directory:="$OUTPUT_DIRECTORY" \
        scenario:="$SCENARIO" \
        global_timeout:="$GLOBAL_TIMEOUT" \
        record:="$RECORD" \
        use_sim_time:="$USE_SIM_TIME"
}

# Source ROS and Autoware setup files
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/autoware/setup.bash"

# Execute passed command if provided, otherwise run scenario simulator
if [ $# -gt 0 ]; then
    exec "$@"
else
    run_scenario_simulator
fi
