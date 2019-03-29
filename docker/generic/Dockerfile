ARG FROM_ARG
ARG ROS_DISTRO
FROM ${FROM_ARG}

ENV USERNAME autoware

# Build Autoware
COPY --chown=autoware ./ /home/$USERNAME/Autoware
RUN su -c "bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; \
                  cd /home/$USERNAME/Autoware/ros; \
                  ./colcon_release'" $USERNAME
RUN echo "source /home/$USERNAME/Autoware/ros/install/local_setup.bash" >> \
    /home/$USERNAME/.bashrc

COPY ./docker/generic/entrypoint.sh /tmp
ENTRYPOINT ["/tmp/entrypoint.sh"]
