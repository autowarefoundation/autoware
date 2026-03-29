# check=skip=InvalidDefaultArgInFrom
ARG ROS_DISTRO

FROM ros:${ROS_DISTRO}-ros-base AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO
ARG RMW_IMPLEMENTATION
ARG USERNAME=aw

RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    pipx \
    bash-completion \
    gosu

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
RUN --mount=type=cache,target=/home/aw/.cache/pip,uid=1000,gid=1000 \
    --mount=type=cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    python3 -m pipx ensurepath && \
    pipx install --include-deps --force "ansible==10.*"

COPY --chown=${USERNAME}:${USERNAME} ansible-galaxy-requirements.yaml /tmp/ansible/
COPY --chown=${USERNAME}:${USERNAME} ansible/ /tmp/ansible/ansible/

WORKDIR /tmp/ansible
RUN ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.autoware_requirements \
      --tags rmw \
      -e rosdistro=${ROS_DISTRO} \
      -e rmw_implementation=${RMW_IMPLEMENTATION}

COPY docker-new/files/cyclonedds.xml /home/${USERNAME}/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///home/${USERNAME}/cyclonedds.xml

WORKDIR /home/${USERNAME}
RUN rm -rf /tmp/ansible

# Entrypoint runs as root so it can adjust UID/GID, then drops to user
# hadolint ignore=DL3002
USER root
COPY --chmod=755 docker-new/docker-entrypoint.sh /docker-entrypoint.sh

ENV ROS_DISTRO=${ROS_DISTRO}
ENV USERNAME=${USERNAME}

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["/bin/bash"]
