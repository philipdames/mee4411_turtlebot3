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

echo "[Set the target OS, ROS version and name of catkin workspace]"
name_os_version=${name_os_version:="bionic"}
name_ros_version=${name_ros_version:="melodic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}

# Make sure all of the software in the Ubuntu installation is up to date before starting
echo "[Update the package lists and upgrade them]"
sudo apt update -y
sudo apt upgrade -y

echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt install -y chrony ntpdate curl build-essential
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

echo "[Check the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

echo "[Update the package lists]"
sudo apt update -y

echo "[Install the ros-desktop-full and all rqt plugins]"
sudo apt install -y ros-$name_ros_version-desktop-full ros-$name_ros_version-rqt-*

echo "[Install other ROS packages]"
sudo apt install -y ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
sudo apt install -y python3-catkin-tools

echo "[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential git 

echo "[Install rosdep]"
sudo apt install -y python-rosdep

echo "[Initialize rosdep and Update]"
sudo sh -c "rosdep init"
rosdep update

# Set up the ROS catkin workspace
if [ ! -d $HOME/$name_catkin_workspace/src ];
  mkdir -p $HOME/$name_catkin_workspace/src
fi
cd $HOME/$name_catkin_workspace
catkin init
# Go to the workspace folder
cd $HOME/$name_catkin_workspace/src/
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
cd $HOME/$name_catkin_workspace
catkin build

echo "[Set the ROS evironment variables]"
echo "" >> ~/.bashrc
echo "export ROS_WORKSPACE=$name_catkin_workspace" >> ~/.bashrc
echo "" >> ~/.bashrc
echo "alias eb='nano ~/.bashrc'" >> ~/.bashrc
echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
echo "alias gs='git status'" >> ~/.bashrc
echo "alias gp='git pull'" >> ~/.bashrc
echo -e "alias cw='cd ~/\$ROS_WORKSPACE'" >> ~/.bashrc
echo -e "alias cs='cd ~/\$ROS_WORKSPACE/src'" >> ~/.bashrc
echo -e "alias cb='cd ~/\$ROS_WORKSPACE && catkin build'" >> ~/.bashrc
echo "" >> ~/.bashrc
echo "source /opt/ros/$name_ros_version/setup.bash" >> ~/.bashrc
echo -e "source ~/\$ROS_WORKSPACE/devel/setup.bash" >> ~/.bashrc
echo "" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
echo "" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

source $HOME/.bashrc

echo "[Complete!!!]"
exit 0
