# hadolint global ignore=DL3006,DL3008,DL3013
ARG BASE_IMAGE

FROM $BASE_IMAGE as base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

# Install apt packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  git \
  ssh \
  wget \
  cmake \
  curl \
  gosu \
  gnupg \
  vim \
  unzip \
  lsb-release \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# Add GitHub to known hosts for private repositories
RUN mkdir -p ~/.ssh \
  && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Copy files
COPY setup-dev-env.sh ansible-galaxy-requirements.yaml amd64.env arm64.env /autoware/
COPY ansible/ /autoware/ansible/
WORKDIR /autoware

# Set up base environment
RUN --mount=type=ssh \
  ./setup-dev-env.sh -y --module base --runtime openadk \
  && pip uninstall -y ansible ansible-core \
  && pip install --no-cache-dir vcstool \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" > /etc/bash.bashrc

# Create entrypoint
CMD ["/bin/bash"]

FROM $BASE_IMAGE as src-imported
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

WORKDIR /autoware
COPY autoware.repos /autoware/
# Register the pacmod3_msgs repository
RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/autonomoustuff-public.list' \
  && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
    python3-vcstool \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache
RUN --mount=type=ssh \
  mkdir src \
  && vcs import src < autoware.repos

# Generate install package lists
RUN rosdep keys --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-all-depend-packages.txt \
    || true \
    && cat /rosdep-all-depend-packages.txt
RUN rosdep keys --dependency-types=exec --ignore-src --from-paths src \
    | xargs rosdep resolve --rosdistro ${ROS_DISTRO} \
    | grep -v '^#' \
    | sed 's/ \+/\n/g'\
    | sort \
    > /rosdep-exec-depend-packages.txt \
    || true \
    && cat /rosdep-exec-depend-packages.txt

FROM base as prebuilt
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG SETUP_ARGS
ENV CCACHE_DIR="/var/tmp/ccache"
ENV CC="/usr/lib/ccache/gcc"
ENV CXX="/usr/lib/ccache/g++"

# cspell: ignore libcu libnv
# Set up development environment
RUN --mount=type=ssh \
  ./setup-dev-env.sh -y --module all ${SETUP_ARGS} --no-cuda-drivers openadk \
  && pip uninstall -y ansible ansible-core \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache \
  && find / -name 'libcu*.a' -delete \
  && find / -name 'libnv*.a' -delete

# Install rosdep dependencies
COPY --from=src-imported /autoware/src /autoware/src
COPY --from=src-imported /rosdep-all-depend-packages.txt /tmp/rosdep-all-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  apt-get update \
  && cat /tmp/rosdep-all-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# Build Autoware
RUN source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-args \
    " -Wno-dev" \
    " --no-warn-unused-cli" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  && find /autoware/install -type d -exec chmod 777 {} \; \
  && chmod -R 777 /var/tmp/ccache \
  && rm -rf /autoware/build /autoware/src

CMD ["/bin/bash"]

FROM prebuilt as devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install development tools and artifacts
RUN --mount=type=ssh \
  ./setup-dev-env.sh -y --module dev-tools --download-artifacts openadk \
  && pip uninstall -y ansible ansible-core \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

# Create entrypoint
COPY docker/autoware-openadk/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

FROM base as runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR
ARG SETUP_ARGS

# Set up runtime environment and artifacts
COPY --from=src-imported /rosdep-exec-depend-packages.txt /tmp/rosdep-exec-depend-packages.txt
# hadolint ignore=SC2002
RUN --mount=type=ssh \
  ./setup-dev-env.sh -y --module all ${SETUP_ARGS} --download-artifacts --no-cuda-drivers --runtime openadk \
  && pip uninstall -y ansible ansible-core \
  && apt-get update \
  && cat /tmp/rosdep-exec-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache \
  && find /usr/lib/$LIB_DIR-linux-gnu -name "*.a" -type f -delete \
  && find / -name "*.o" -type f -delete \
  && find / -name "*.h" -type f -delete \
  && find / -name "*.hpp" -type f -delete \
  && rm -rf /autoware/ansible \
    /root/.local/pipx /opt/ros/"$ROS_DISTRO"/include /etc/apt/sources.list.d/cuda*.list \
    /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
    /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*

# Copy prebuilt binaries
COPY --from=prebuilt /autoware/install/ /autoware/install/

# Copy bash aliases
COPY docker/autoware-openadk/etc/.bash_aliases /root/.bash_aliases
RUN echo "source /autoware/install/setup.bash" > /etc/bash.bashrc

# Create entrypoint
COPY docker/autoware-openadk/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
