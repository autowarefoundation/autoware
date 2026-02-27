# check=skip=InvalidDefaultArgInFrom
ARG ROS_DISTRO

FROM ros:${ROS_DISTRO}-ros-base AS core-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO
ARG RMW_IMPLEMENTATION
ARG USERNAME=aw

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    pipx \
    bash-completion \
    gosu \
    && rm -rf /var/lib/apt/lists/*

RUN userdel -r ubuntu 2>/dev/null || true

RUN useradd -m -s /bin/bash -U ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/90-user-nopasswd && \
    chmod 0440 /etc/sudoers.d/90-user-nopasswd && \
    sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Make pipx shims visible during build steps and at runtime
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# Ansible via pipx
RUN python3 -m pipx ensurepath && \
    pipx install --include-deps --force "ansible==10.*"

COPY --chown=${USERNAME}:${USERNAME} ansible-galaxy-requirements.yaml /tmp/ansible/
COPY --chown=${USERNAME}:${USERNAME} ansible/ /tmp/ansible/ansible/

WORKDIR /tmp/ansible
RUN ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.core \
      --tags rmw \
      --tags ccache \
      -e rosdistro=${ROS_DISTRO} \
      -e rmw_implementation=${RMW_IMPLEMENTATION}

WORKDIR /home/${USERNAME}
RUN rm -rf /tmp/ansible

# Entrypoint runs as root so it can adjust UID/GID, then drops to user
USER root
COPY --chmod=755 docker-new/docker-entrypoint.sh /docker-entrypoint.sh

ENV ROS_DISTRO=${ROS_DISTRO}
ENV USERNAME=${USERNAME}

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["/bin/bash"]

FROM core-base AS core-rosdep

ARG ROS_DISTRO
ARG USERNAME=aw

RUN rosdep update

USER root
COPY repositories/ /tmp/repositories/

RUN mkdir -p /tmp/rosdep-install/src && \
    cd /tmp/rosdep-install && \
    vcs import --recursive --shallow src < /tmp/repositories/autoware.repos && \
    apt-get update && \
    rosdep install -y \
      --from-paths src/core \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" && \
    rm -rf /var/lib/apt/lists/* /tmp/rosdep-install /tmp/repositories

FROM core-base AS core-prebuilt

ARG ROS_DISTRO
ARG USERNAME=aw
