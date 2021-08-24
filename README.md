# TinkerOS_ROS
This repository introduces how to install ROS 1, Melodic, in ASUS Tinker board under Tinker OS.

## Hardware
- ASUS Tinker board
## Software
- TinkerOS Debian Stretch v2.1.16

# ROS download and installation in TinkerOS

## Dependencies for building packages
Open a linux terminal shell.
To compile ROS in tinker board,
1. Add the ROS repository to your sources.list to accept software from packages.ros.org
2. Download a key from keyserver directly into the trusted set of keys
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install dirmngr
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
If the key server encounters failure, please try another server below:
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
Ensure the packages are up-to-date and install the packages:
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```
When tools are all installed, initialize **rosdep**. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS:
```
sudo rosdep init
rosdep update
```

Create an environment to compile ROS, download and compile the external parts:
```
mkdir -p ~/ros_catkin_ws && cd ~/ros_catkin_ws

rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall

wstool init src melodic-ros_comm-wet.rosinstall

mkdir -p ~/ros_catkin_ws/external_src && cd ~/ros_catkin_ws/external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make -j4 -w
sudo make install
```
Install the ROS:
```
cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --skip-keys='sbcl'
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic
```
## Environment Setup
To be convienent, add one line at the end of the **~/.bashrc** file to run ROS without sourcing the "setup.bash" file:
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
**Reboot** your machine to update all the setting.
To check if environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:
```
printenv | grep ROS
```
(If not, redo the environment setup part)

Now, ROS is available by calling roscore

## Create a ROS workspace
Create and build a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
(**catkin_make** is a convenient tool for working with catkin workspaces.
It will create 'build', 'devel' and 'src' folders with CMakeLists.txt.
In 'devel' folder, there are several setup.*sh files, sourcing any of these files will overlay this workspace on top of your environment.)
```
source devel/setup.bash
```
To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.
```
echo $ROS_PACKAGE_PATH
```
and it should return '/home/<username>/catkin_ws/src:/opt/ros/melodic/share'

## Build cv_bridge for python3
Install packages needed for building cv_bridge
```
sudo apt-get install python-catkin-tools python3-dev python3-numpy
```
Create new catkin_build_ws to avoid any future problems with catkin_make(assuming you are using it) and config catkin to use your python 3(3.6 in my case) when building packages
Install in system python
```
mkdir -p ~/catkin_build_ws/src && cd ~/catkin_build_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.5m -DPYTHON_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython3.5m.so
catkin config --install
```
or install under miniconda
```
mkdir -p ~/catkin_build_ws/src && cd ~/catkin_build_ws
catkin config -DPYTHON_EXECUTABLE=/home/ran/miniconda3/bin/python3 -DPYTHON_INCLUDE_DIR=/home/ran/miniconda3/bin/python3.5m -DPYTHON_LIBRARY=/home/ran/miniconda3/lib/libpython3.5m.so
catkin config --install
```
clone official vision_opencv repo:
```
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
pip3 install catkin_pkg 
```
Finally, build and source the package:
```
cd ~/catkin_build_ws
catkin build cv_bridge
source install/setup.bash --extend
```
Now, cv_bridge is available in Python 3

## Install ONNX
(ongoing)

## Error message 
1. ImportError: No module named Cryptodome.Cipher
Solution:
```
sudo apt install python3-pip python-pip
pip install pycryptodomex
```
2. ImportError: No module named 'yaml'
```
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
```

## Reference
- ROS installation guide for TinkerBoard. http://nascivera.it/2017/11/20/ros-installation-guide-for-tinkerboard/
- ROS Tutorials. http://wiki.ros.org/ROS/Tutorials
- ROS Python2 & Python3 conflict Resolve. https://rancheng.github.io/ros-python2-3-conflict/
- ONNX github. https://github.com/onnx/onnx
  
