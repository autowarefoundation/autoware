### Builder
FROM ghcr.io/autowarefoundation/autoware:universe-devel AS builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"

COPY src/simulator /autoware/src/simulator
COPY docker/scripts/ /autoware/
RUN chmod +x /autoware/*.sh
WORKDIR /autoware

# Extract rosdep dependencies for simulator
# hadolint ignore=SC1091
RUN source /opt/autoware/setup.bash \
    && /autoware/resolve_rosdep_keys.sh /autoware/src ${ROS_DISTRO} --dependency-types=exec \
    > /rosdep-simulator-exec-depend-packages.txt \
    && cat /rosdep-simulator-exec-depend-packages.txt

# Install dependencies and build the scenario simulator
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update \
    && source /opt/autoware/setup.bash \
    && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
    && /autoware/cleanup_apt.sh

# Build the scenario simulator
# hadolint ignore=SC1091
RUN --mount=type=cache,target=${CCACHE_DIR} \
    source /opt/autoware/setup.bash \
    && /autoware/build_and_clean.sh ${CCACHE_DIR} /opt/autoware \
    && find /opt/autoware/lib -type f -name "*.py" -exec chmod +x {} \; \
    && find /opt/autoware/share -type f -name "*.py" -exec chmod +x {} \;

### Scenario Simulator
FROM ghcr.io/autowarefoundation/autoware:universe AS scenario-simulator
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

COPY --from=builder /opt/autoware /opt/autoware
COPY --from=builder /rosdep-simulator-exec-depend-packages.txt /tmp/rosdep-simulator-exec-depend-packages.txt

# hadolint ignore=SC2002
RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update \
    # Remove xmlschema and yamale from rosdep packages since we install via pip
    && sed -i '/\(xmlschema\|yamale\)/d' /tmp/rosdep-simulator-exec-depend-packages.txt \
    && pip3 install --no-cache-dir yamale \
    && pip3 install --no-cache-dir xmlschema==3.4.5 \
    && cat /tmp/rosdep-simulator-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
    && /autoware/cleanup_system.sh $LIB_DIR $ROS_DISTRO

COPY docker/tools/scenario-simulator/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
