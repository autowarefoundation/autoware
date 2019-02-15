ARG FROM_ARG
FROM ${FROM_ARG}

ARG ROS_DISTRO
ARG ROS2_DISTRO
ENV USERNAME autoware

# Build Autoware
COPY --chown=autoware ./ /home/$USERNAME/Autoware
RUN su -c "bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; \
                  source /opt/ros/$ROS2_DISTRO/setup.bash; \
                  cd /home/$USERNAME/Autoware/ros/src; \
                  catkin_init_workspace; \
                  cd ../; \
                  ./catkin_make_release'" $USERNAME
RUN echo "source /home/$USERNAME/Autoware/ros/devel/setup.bash" >> \
    /home/$USERNAME/.bashrc

COPY ./docker/generic/entrypoint.sh /tmp
ENTRYPOINT ["/tmp/entrypoint.sh"]
