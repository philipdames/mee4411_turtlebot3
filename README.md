# mee4411_turtlebot3
Code to use the Turtlebot3 in MEE4411: Introduction to Mobile Robotics

## Code Installation instructions
**Note: This assumes you have a fresh installation of Ubuntu. If you already have a folder in your home directory called catkin_ws then you will need to rename it before running this installation script.**

Download and run the installation script using the following commands:
```
wget https://raw.githubusercontent.com/philipdames/mee4411_turtlebot3/master/remote_pc_setup_kinetic.sh
chmod 755 remote_pc_setup_kinetic.sh
bash remote_pc_setup_kinetic.sh
```    
This will download and install ROS and all of the code libraries that you need in order to run the simulations for this class.
It will also set up your ROS workspace, configure a lot of parameters on your computer, etc.


## Test your simulation
Run the following command to ensure that your simulation environment is working properly. 
```
source ~/.bashrc
roslaunch mee4411_core mee4411_simulation.launch
```
You should see the following screen appear:
![rviz screenshot of Turtlebot3 simulation](https://github.com/philipdames/mee4411_turtlebot3/blob/master/rviz_screenshot.PNG)
