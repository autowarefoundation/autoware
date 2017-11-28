#!/usr/bin/env bash

# Runs a docker container with the image created by build_demo.bash
# Requires
#   docker
#   nvidia-docker
#   an X server
# Recommended
#   A joystick mounted to /dev/input/js0 or /dev/input/js1


until sudo nvidia-docker ps
do
    echo "Waiting for docker server"
    sleep 1
done

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth-n #XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        # https://superuser.com/questions/806637/xauth-not-creating-xauthority-file
        mv .$XAUTH old.$XAUTH
        touch $XAUTH
        xauth generate :0 . trusted
        xauth add ${HOST}:0 . $(xxd -l 16 -p /dev/urandom)
    fi
    chmod a+r $XAUTH
fi

XSOCK=/tmp/.X11-unix

sudo nvidia-docker run \
        -it --rm \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY=${DISPLAY}" \
        -u autoware \
        --privileged \
        --net=host \
        autoware-image

# --net=host # http://ask.projectatomic.io/en/question/3647/how-to-connect-to-session-dbus-from-a-docker-container/
# --privileged  # to get ride of: ** (gnome-terminal.real:33): WARNING **: Couldn't register with accessibility bus: An AppArmor policy prevents this sender from sending this message to this recipient; type="method_call", sender="(null)" (inactive) interface="org.freedesktop.DBus" member="Hello" error name="(unset)" requested_reply="0" destination="org.freedesktop.DBus" (bus)
