#!/bin/sh

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
sudo apt update
sudo apt install gazebo7 gazebo7-common gazebo7-plugin-base libgazebo7 libgazebo7-dev libignition-math2 libignition-math2-dev libsdformat4 libsdformat4-dev sdformat-sdf
