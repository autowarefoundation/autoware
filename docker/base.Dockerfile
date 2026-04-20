# check=skip=InvalidDefaultArgInFrom
ARG ROS_DISTRO

FROM ros:${ROS_DISTRO}-ros-base AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO
ARG USERNAME=aw

RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache && \
    echo 'APT::Install-Recommends "false";' > /etc/apt/apt.conf.d/99-no-recommends && \
    echo 'APT::Install-Suggests "false";' >> /etc/apt/apt.conf.d/99-no-recommends && \
    echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/99-retries && \
    echo 'Acquire::http::Timeout "30";' >> /etc/apt/apt.conf.d/99-retries && \
    echo 'Acquire::https::Timeout "30";' >> /etc/apt/apt.conf.d/99-retries && \
    printf 'http://azure.archive.ubuntu.com/ubuntu\tpriority:1\nhttp://archive.ubuntu.com/ubuntu\tpriority:2\n' > /etc/apt/ubuntu-mirrors.list && \
    for f in /etc/apt/sources.list /etc/apt/sources.list.d/ubuntu.sources; do \
      if [ -f "$f" ]; then \
        sed -E -i 's|http://archive\.ubuntu\.com/ubuntu/?|mirror+file:///etc/apt/ubuntu-mirrors.list|g' "$f"; \
      fi; \
    done

RUN --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    pipx \
    bash-completion \
    iproute2 \
    gosu

# Remove default ubuntu user (present since 24.04, occupies UID 1000)
RUN userdel -r ubuntu 2>/dev/null || true && \
    useradd -m -s /bin/bash -U ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/90-user-nopasswd && \
    chmod 0440 /etc/sudoers.d/90-user-nopasswd && \
    sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Make pipx shims visible during build steps and at runtime
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

ENV ANSIBLE_COLLECTIONS_PATH="/home/${USERNAME}/.ansible/collections"

# hadolint ignore=DL3003
RUN --mount=type=bind,source=ansible-galaxy-requirements.yaml,target=/tmp/ansible/ansible-galaxy-requirements.yaml \
    --mount=type=bind,source=ansible/galaxy.yml,target=/tmp/ansible/ansible/galaxy.yml \
    --mount=type=bind,source=ansible/roles/rmw_implementation,target=/tmp/ansible/ansible/roles/rmw_implementation \
    --mount=type=bind,source=ansible/playbooks/rmw.yaml,target=/tmp/ansible/ansible/playbooks/rmw.yaml \
    --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,id=pipx-cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    cd /tmp/ansible && \
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.rmw \
      -e rosdistro=${ROS_DISTRO} && \
    pipx uninstall ansible

COPY docker/files/cyclonedds.xml /home/${USERNAME}/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///home/${USERNAME}/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Entrypoint runs as root so it can adjust UID/GID, then drops to user
USER root
COPY --chmod=755 docker/docker-entrypoint.sh /docker-entrypoint.sh

ENV ROS_DISTRO=${ROS_DISTRO}
ENV USERNAME=${USERNAME}

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["/bin/bash"]
