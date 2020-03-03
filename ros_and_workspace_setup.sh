#!/bin/bash

#
# MAIN VARS TO CHANGE
#
ARENA_INSTALLATION_ROOT="$HOME/ArenaSDK_Linux_x64"
ARENA_ROS_WORDSPACE_TO_SETUP=``pwd`` #change to workspace location
INSTALL_ROS=1



############################################################
# Note:
#    ArenaSDK does not need to be installed as long as 
#    $ARENA_INSTALLATION_ROOT point to ArenaSDK binaries
#
############################################################

set -x #echo on

# only run with sudo
if [ `id -u` -ne 0 ]; then 
    echo "Please run as root"
    exit
fi

# info 
CURR_OS="$(lsb_release -sc)"

if [ $CURR_OS = "xenial" ]; then
    ROS_DIS="kinetic"
else
    echo "$CURR_OS is not saupported yet"
    exit -1
fi

############################################################
#   ROS section
############################################################

if [ $INSTALL_ROS -eq 1 ]; then

    # Set up your system to acquire software from packages.ros.org
    sudo echo "deb http://packages.ros.org/ros/ubuntu $CURR_OS main" > /etc/apt/sources.list.d/ros-latest.list
    # Use apt-key to install the Open Robotics key to your list of trusted keys.
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    # Install ROS Desktop.
    sudo apt-get update
    sudo apt-get install ros-$ROS_DIS-desktop-full
    # Setup system dependencies for ROS.
    sudo rosdep init
    sudo rosdep fix-permissions
    rosdep update

    # Setups ROS environment variables
    echo "# load ROS env vars" >> ~/.bashrc
    echo "source /opt/ros/$ROS_DIS/setup.bash" >> ~/.bashrc

    echo "# load ROS env vars" >> ~/.zshrc
    echo "source /opt/ros/$ROS_DIS/setup.zsh" >> ~/.zshrc

    # would not have an effect if script is not run in intractive mode
    source ~/.bashrc 

    #
    # Install ROS package workspace dependencies. This will allow 
    # you to create and manage your own ROS workspaces, including 
    # the ROS workspace usedby arena_camera.
    #
    sudo apt-get install python-rosinstall \
                        python-rosinstall-generator \
                        python-wstool \
                        build-essential

fi
############################################################
# Arena SDK section
############################################################

# dont not need to be installed as long as 
# $ARENA_INSTALLATION_ROOT points to ArenaSDK binaries


# Set up your ARENA_ROOT environment variable. This
# environment variable should be the path where you have
# installed Arena SDK.
echo "export ARENA_ROOT=$ARENA_INSTALLATION_ROOT">> ~/.bashrc
echo "export ARENA_ROOT=$ARENA_INSTALLATION_ROOT">> ~/.zshrc
# would not have an effect if script is not run in intractive mode
source ~/.bashrc

############################################################
# Workspace section
############################################################

# Copy the included image_encoding.h to your ROS include folder after 
# baking the old one up (if existed).
# A custom image_encoding.h is included to enable streaming
# support for LUCID’s Helios camera.
sudo cp -f \
    /opt/ros/$ROS_DIS/include/sensor_msgs/image_encodings.h \
    /opt/ros/$ROS_DIS/include/sensor_msgs/image_encodings.h.bak
sudo cp -f \
    $ARENA_ROS_WORDSPACE_TO_SETUP/inc/image_encodings.h \
    /opt/ros/$ROS_DIS/include/sensor_msgs/image_encodings.h
