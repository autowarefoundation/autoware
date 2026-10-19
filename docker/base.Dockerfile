# check=skip=InvalidDefaultArgInFrom
ARG ROS_DISTRO
ARG BASE_IMAGE_DIGEST=""

FROM ros:${ROS_DISTRO}-ros-base${BASE_IMAGE_DIGEST} AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO
# hadolint ignore=DL3064
ARG USERNAME=aw
ARG USE_LOCKFILE=false

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
    --mount=type=bind,source=ansible,target=/tmp/ansible/ansible \
    --mount=type=cache,id=apt-cache-${ROS_DISTRO},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-lists-${ROS_DISTRO},target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,id=pipx-cache,target=/home/aw/.cache/pipx,uid=1000,gid=1000 \
    pipx install --include-deps "ansible==10.*" && \
    cd /tmp/ansible && \
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml && \
    ansible-playbook autoware.dev_env.install_rmw \
      -e "rosdistro=${ROS_DISTRO}" \
      -e "use_locked_versions=${USE_LOCKFILE}" && \
    pipx uninstall ansible

COPY docker/files/cyclonedds.xml /home/${USERNAME}/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///home/${USERNAME}/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ament_xmllint downloads the package.xml schema from download.ros.org for every package,
# and its tests time out when that server is slow. The catalog points xmllint to local copies.
# The COPY must come first: ADD --chmod would also set the mode of a new directory to 644.
COPY docker/files/ros-schema-catalog.xml /usr/share/xml/ros/catalog.xml
ADD --chmod=644 --checksum=sha256:db232e4957751cfaa7235829691a475929733c3d8a45e8b24946bc3328196879 \
    http://download.ros.org/schema/package_format1.xsd /usr/share/xml/ros/
ADD --chmod=644 --checksum=sha256:a7328af0859b3f15ae4f04912d846a58edc6664db1e69aff061b22940c194304 \
    http://download.ros.org/schema/package_format2.xsd /usr/share/xml/ros/
ADD --chmod=644 --checksum=sha256:585201809cbf8c81ec5a4d7d84d827f4676dd62a4bc0a812da7a485dea2ad43a \
    http://download.ros.org/schema/package_format3.xsd /usr/share/xml/ros/
ENV XML_CATALOG_FILES="file:///etc/xml/catalog file:///usr/share/xml/ros/catalog.xml"

# Entrypoint runs as root so it can adjust UID/GID, then drops to user
USER root
COPY --chmod=755 docker/docker-entrypoint.sh /docker-entrypoint.sh

ENV ROS_DISTRO=${ROS_DISTRO}
# hadolint ignore=DL3064
ENV USERNAME=${USERNAME}

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["/bin/bash"]
