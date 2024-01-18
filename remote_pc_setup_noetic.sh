#!/bin/bash

# Print status message in white text on blue background
print_status () {
  echo -e "\033[37;44m$1\033[0m"
}

# Function to add if no such line in file
ainsl () {
  grep -qF -- "$1" $2 || echo "$1" >> $2
}

# Print message giving an overview of the installation process
echo ""
echo "[Note] Target OS version  >>> Ubuntu 20.04.x (Focal Fossa)"
echo "[Note] Target ROS version >>> ROS Noetic Ninjemys"
echo "[Note] Catkin workspace   >>> $HOME/mee4411_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read REPLY

print_status "[Set the target OS, ROS version and name of catkin workspace]"
name_os_version=${name_os_version:="focal"}
name_ros_version=${name_ros_version:="noetic"}
name_catkin_workspace=${name_catkin_workspace:="mee4411_ws"}

# Make sure all of the software in the Ubuntu installation is up to date before starting
print_status "[Update the package lists and upgrade them]"
sudo apt update -y
sudo apt upgrade -y

print_status "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt install -y chrony ntpdate curl build-essential
sudo ntpdate ntp.ubuntu.com

print_status "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
fi

print_status "[Download the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

print_status "[Check the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

print_status "[Update the package lists]"
sudo apt update -y

print_status "[Install the ros-desktop-full and all rqt plugins]"
sudo apt install -y ros-$name_ros_version-desktop-full ros-$name_ros_version-rqt-*

print_status "[Install RQT & Gazebo]"
sudo apt install -y ros-$name_ros_version-rqt-* ros-$name_ros_version-gazebo-*

print_status "[Install other ROS packages]"
sudo apt install -y ros-noetic-joy \
  ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-laser-proc \
  ros-noetic-rgbd-launch \
  ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python \
  ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs \
  ros-noetic-amcl \
  ros-noetic-map-server \
  ros-noetic-move-base \
  ros-noetic-urdf \
  ros-noetic-xacro \
  ros-noetic-compressed-image-transport \
  ros-noetic-rqt* \
  ros-noetic-rviz \
  ros-noetic-gmapping \
  ros-noetic-navigation \
  ros-noetic-interactive-markers

print_status "[Install Turtlebot3 ROS packages]"
sudo apt install -y ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3

print_status "[Install catkin_tools]"
sudo apt install -y python3-catkin-tools

print_status "[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

print_status "[Install rosdep]"
sudo apt install python3-rosdep

if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  print_status "[Initialize rosdep and Update]"
  sudo sh -c "rosdep init"
  rosdep update
fi

if ! [ -d $HOME/$name_catkin_workspace ]; then
  print_status "[Make the catkin workspace and test the catkin_make]"
  mkdir -p $HOME/$name_catkin_workspace/src
  cd $HOME/$name_catkin_workspace
  catkin init
  catkin build
fi

print_status "[Set the ROS evironment]"
ainsl "alias eb='nano ~/.bashrc'" ~/.bashrc
ainsl "alias sb='source ~/.bashrc'" ~/.bashrc
ainsl "alias gs='git status'" ~/.bashrc
ainsl "alias gp='git pull'" ~/.bashrc
ainsl "alias cw='cd ~/$name_catkin_workspace'" ~/.bashrc
ainsl "alias cs='cd ~/$name_catkin_workspace/src'" ~/.bashrc
ainsl "alias cb='cd ~/$name_catkin_workspace && catkin build'" ~/.bashrc

ainsl "source /opt/ros/$name_ros_version/setup.bash" ~/.bashrc
ainsl "source ~/$name_catkin_workspace/devel/setup.bash" ~/.bashrc

ainsl "export ROS_MASTER_URI=http://localhost:11311" ~/.bashrc
ainsl "export ROS_HOSTNAME=localhost" ~/.bashrc

print_status "[Complete!!!]"
exit 0
