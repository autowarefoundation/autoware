ARG FROM_ARG
ARG ROS_DISTRO
FROM ${FROM_ARG}

ENV USERNAME autoware

# Build Autoware
COPY --chown=autoware ./ /home/$USERNAME/Autoware
RUN su -c "bash -c ' \
    source /opt/ros/$ROS_DISTRO/setup.bash; \
    cd /home/$USERNAME/Autoware; \
    mkdir src; \
    vcs import src < autoware.ai.repos; \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    '" $USERNAME
RUN echo "source /home/$USERNAME/Autoware/install/local_setup.bash" >> \
    /home/$USERNAME/.bashrc

COPY ./docker/generic/entrypoint.sh /tmp
ENTRYPOINT ["/tmp/entrypoint.sh"]
