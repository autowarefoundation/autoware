#!/bin/bash
#
# Copyright 2014-2015 The MathWorks, Inc.
ARCHIVE=$1
CATKIN_WS=$2

catkinWorkspaceHelp() {
   echo ""
   echo "You can create a Catkin workspace as follows:"
   echo "  mkdir -p ~/catkin_ws/src"
   echo "  cd ~/catkin_ws/src"
   echo "  catkin_init_workspace"
}


commandUsage() {
   echo "Usage: $(basename $0) ARCHIVE_NAME... CATKIN_WS..."
   echo "Extract and build a C++ ROS node generated from a Simulink model."
   echo "ARCHIVE_NAME is the name of the TGZ file generated from the Simulink model."
   echo "CATKIN_WS is the full path to your ROS Catkin workspace."
   echo ""
   echo "Example:"
   echo "  ./$(basename $0) simulinkmodel.tgz ~/catkin_ws"   
}


fullUsage() {
   commandUsage
   catkinWorkspaceHelp
   exit
}


toLowerCase() {
   echo $1 | tr '[A-Z]' '[a-z]'
}

if [ -z $1 ] || ([ ! -z $1 ] && [ $1 = "-h" ] || [ $1 = "--help" ]) ; then
   fullUsage
   exit 0
fi

if [ ! $# -eq 2 ] ; then
   echo "Expected two input arguments. Got $#."
   fullUsage
   exit 1
fi

# Check Catkin workspace
if [ ! -d "$CATKIN_WS" ] ; then
   echo "The catkin workspace directory, "$CATKIN_WS", does not exist."
   echo "Enter a valid catkin workspace directory."
   catkinWorkspaceHelp
   exit 1
fi

# Sanity check for CATKIN workspace
if [ ! -f "$CATKIN_WS"/src/CMakeLists.txt ] ; then
   echo "The Catkin workspace directory, "$CATKIN_WS", is not a valid Catkin workspace."
   echo "Enter a valid Catkin workspace directory."
   catkinWorkspaceHelp
   exit 1
fi

# Check Simulink archive
if [ ! -f "$ARCHIVE" ] ; then
   echo "The archive, "$ARCHIVE", does not exist."
   echo "Enter a valid Simulink model archive (.tgz file)."
   echo ""
   commandUsage
   exit 1
fi

# Enforce that $ARCHIVE ends with .tgz, since the model 
# name is derived by stripping off the .tgz extension
if [ ${ARCHIVE: -4} != ".tgz" ] ; then
   echo "The archive, "$ARCHIVE", does not have a .tgz extension."
   echo "Enter a valid Simulink model archive (.tgz file)."
   echo ""   
   commandUsage
   exit 1
fi

# Check if $ARCHIVE is a valid zip file
gzip -t $ARCHIVE 2> /dev/null
VALID_ZIP=$?
if [ $VALID_ZIP -ne 0 ] ; then
   echo "The archive, "$ARCHIVE", is not a valid .tgz (tar zip) file."
   echo ""
   commandUsage
   exit 1   
fi

# Check for one of the standard files generated from Simulink
# (ert_main.cpp)
tar ztf $ARCHIVE | grep -q ert_main.cpp 2> /dev/null
VALID_SIMULINK_ARCHIVE=$?
if [ $VALID_SIMULINK_ARCHIVE -ne 0 ] ; then
   echo "The archive, "$ARCHIVE", is not a valid Simulink model archive (.tgz file)."
   echo ""
   commandUsage
   exit 1
fi

# $ARCHIVE appears to be valid.
# Extract and build it

MODEL_NAME=$(toLowerCase $(basename $ARCHIVE .tgz))
PROJECT_DIR="$CATKIN_WS/src/$MODEL_NAME"
echo "Catkin project directory: $PROJECT_DIR"

# Extract files to catkin project directory
mkdir -p "$PROJECT_DIR"
rm -fr "$PROJECT_DIR"/*
tar -C "$PROJECT_DIR" -xf $ARCHIVE

# Ensure that catkin_make will rebuild the executable
touch "$PROJECT_DIR"/*.cpp

# Build the Simulink model as a catkin project
CURR_DIR=`pwd`
cd "$CATKIN_WS"
catkin_make "$MODEL_NAME"_node
cd "$CURR_DIR"

exit 0
