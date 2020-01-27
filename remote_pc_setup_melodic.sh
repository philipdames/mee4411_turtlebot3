#!/bin/bash

# Print message giving an overview of the installation process
echo ""
echo "[Note] Target OS version  >>> Ubuntu 18.04.x (Bionic Beaver) or Linux Mint 19.x"
echo "[Note] Target ROS version >>> ROS Melodic Morenia"
echo "[Note] Catkin workspace   >>> $HOME/catkin_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

# Make sure all of the software in the Ubuntu installation is up to date before starting
echo "[Update the package lists and upgrade them]"
sudo apt update -y
sudo apt upgrade -y

# Download and install ROS using a script provided by Robotis, the company that makes the Turtlebot3
echo "[Download and install ROS]"
# Download file
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
# Change the file to make it executable
chmod 755 ./install_ros_melodic.sh
# Run the file
bash ./install_ros_melodic.sh
# Install other ROS packages
sudo apt-get install -y ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

# Set the Turtlebot model
echo "[Set the ROS turtlebot3 evironment variables]"
sh -c "echo \"export TURTLEBOT3_MODEL=burger\" >> ~/.bashrc"

# Set up the ROS catkin workspace
echo "[Set up catkin workspace]"
# Go to the workspace folder
cd $HOME/catkin_ws/src/
# Download the turtlebot packages from Robotis
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
# Install a few more libraries
sudo apt-get install -y libarmadillo-dev libcgal-dev libcgal-qt5-dev
# Download packages for course
git clone https://github.com/philipdames/mee4411_turtlebot3.git
# Build the packages
source /opt/ros/melodic/setup.bash
source $HOME/catkin_ws/devel/setup.bash
cd $HOME/catkin_ws
catkin_make

echo "[Complete!!!]"
exit 0
