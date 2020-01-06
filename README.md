# mee4411_turtlebot3
Code to use the Turtlebot3 in MEE4411: Introduction to Mobile Robotics

## Windows Subsystem for Linux (WSL) Setup
1. Follow the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/install-win10) to enable WSL and install Ubuntu 18.04.
1. Follow the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/initialize-distro) to initialize your new Ubuntu installation.
1. Download one of the following X server programs to allow your Ubuntu intallation to bring up graphical windows. X is the windowing system used by Linux so installing this will allow you to see the simulation and data visualization windows.
    1. **Recommended:** [Xming](https://sourceforge.net/projects/xming/)
    1. [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
    1. [Cygwin-X](http://x.cygwin.com/) with instructions [here](https://x.cygwin.com/docs/ug/setup.html#setup-cygwin-x-installing)

If you ever have a problem viewing windows, make sure that your xserver program of choice is running!

## Code Installation instructions
You just need to download and run the installation script using the following commands:
```
wget https://raw.githubusercontent.com/philipdames/mee4411_turtlebot3/master/mee4411_core/setup/remote_pc_setup.bash?token=AANO7NQKYFL7SQUEZUZK2526COF6U
chmod 755 remote_pc_setup.bash
bash remote_pc_setup.bash
```    
This will download and install ROS and all of the code libraries that you need in order to run the simulations for this class.
It will also set up your ROS workspace, configure a lot of parameters on your computer, etc.
**Note: This assumes you have a fresh installation and are using WSL with Ubuntu 18.04.**

## Test your X server setup
This will download and install a set of programs to run visual tests. You should see a set of eyes appear on your screen that follow your mouse cursor around.
```
sudo apt-get install x11-apps
xeyes
```

## Test your simulation
Run the following command to ensure that your simulation environment is working properly. 
```
roslaunch mee4411_core mee4411_simulation.launch
```
You should see the following screen appear:
![rviz screenshot of Turtlebot3 simulation](https://raw.githubusercontent.com/philipdames/mee4411_turtlebot3/master/rviz_screenshot.PNG?token=AANO7NSX65RW2QWLZ45CQCS6COJQY)
