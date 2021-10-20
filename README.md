# TinkerOS-ROS
This repository introduces how to install ROS 1(Melodic) and the corresponding packages in python3 in order to run facial detection&recognition in ASUS Tinker board under Tinker OS.

## Hardware
- ASUS Tinker board S
## Software
- TinkerOS Debian Stretch v2.1.16
- ROS == Melodic
- python == 3.5
- numpy == 1.18.5
- scipy == 1.3.3 
- pillow == 7.2.0
- opencv-python == 3.3.0
- pytorch == 1.0.0
- torchvision == 0.2.2
- onnx == 1.6.0
- onnxruntime == 1.1.0
## repo directory
**python wheel**: contain the python wheel of python3.5~3.8 for arm32 devices (for convenience)

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
In 'devel' folder, there are several **setup.sh** files, sourcing any of these files will overlay this workspace on top of your environment.)

```
source devel/setup.bash
```
To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.
```
echo $ROS_PACKAGE_PATH
```
and it should return `/home/<username>/catkin_ws/src:/opt/ros/melodic/share`

## Build cv_bridge for python3

A most simple one would be installed from the Ubuntu terminal:
```
sudo apt-get install ros-(ROS version name)-cv-bridge
sudo apt-get install ros-(ROS version name)-vision-opencv
```
However, it does not work for Python3. So, another is tried by visit the official ROS page of that required package and download the zip file. Move the extracted package inside the catkin workspace and install it.

- Install packages needed for building cv_bridge
```
sudo apt-get install python-catkin-tools python3-dev python3-numpy
```
- Create new catkin_build_ws to avoid any future problems with catkin_make(assuming you are using it) and config catkin to use your python 3(3.5 in my case) when building packages
```
mkdir -p ~/catkin_build_ws/src && cd ~/catkin_build_ws
```
- Install in system python
```
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.5m -DPYTHON_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython3.5m.so
catkin config --install
```
or install under miniconda
```
mkdir -p ~/catkin_build_ws/src && cd ~/catkin_build_ws
catkin config -DPYTHON_EXECUTABLE=/home/linaro/miniconda3/bin/python3 -DPYTHON_INCLUDE_DIR=/home/linaro/miniconda3/bin/python3.5m -DPYTHON_LIBRARY=/home/linaro/miniconda3/lib/libpython3.5m.so
catkin config --install
```
- clone official vision_opencv repo:
```
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
pip3 install catkin_pkg 
```
- Finally, build and source the package:
```
cd ~/catkin_build_ws
catkin build cv_bridge
source install/setup.bash --extend
```
Now, cv_bridge is available in Python 3

# Install Python libraries
## Install numpy
```
sudo apt install python3-dev build-essential
pip3 install numpy --user
```

## Install scipy
```
# Install gfortran for compile
sudo apt-get install gfortran
# Install blas, lapack, atlas for OS related tasks
sudo apt-get install libopenblas-dev liblapack-dev libatlas-base-dev

pip3 install scipy
```
If the process of building scipy failed, there is another way by download the [python wheel](https://www.piwheels.org/simple/scipy/) and install it via:
```
pip3 install scipy-1.3.3-cp35-cp35m-linux_armv7l.whl
```

## Install Pillow
```
pip3 install Pillow
```

## Install OpenCV
```
pip3 install --upgrade pip
pip3 install setuptools
pip3 install opencv-python
```  

## Install pytorch
Search for corresponding libopenblas package and install it
```
sudo apt-cache search libopenblas
sudo apt install libopenblas-dev
```
Download the torch [python wheel](https://github.com/lawkee48/TinkerOS_ROS/tree/main/python_wheel) and install it
```
pip3 install torch-1.0.0a0+8322165-cp35-cp35m-linux_armv7l.whl
```

## Install torchvision
If your torch version is <=1.0.1, please make sure you are installing the torchvision 0.2.2. The compatability of python3.5 only allow pytorch up to 1.5.1 with torchvision up to 0.6.1.
```
pip3 install torchvision
```


## Install ONNX
Install the library dependencies.
```
sudo apt-get install protobuf-compiler libprotoc-dev pybind11
```
Install the onnx module (the newest 1.10.1 receving error on python3.5)
```
pip3 install onnx==1.8.1 --user    # or other version
```
If **anything goes wrong**, you can still follow the [guideline](https://dev.to/mshr_h/how-to-build-onnx-onnx-for-your-armv7l-devices-3cm5) to build the ONNX via Docker.

## Install onnxruntime
[ONNX Runtime](https://github.com/microsoft/onnxruntime) is a cross-platform, high performance scoring engine for ML models. You can get Python bindings for Linux, Windows, Mac on x64 and arm64 platform from [pypi](https://pypi.org/project/onnxruntime/#files). Onnxruntime is neither avaiable in ```pip3 install``` nor build from [source](https://github.com/microsoft/onnxruntime) due to the platform architecture of armv7l. The possible solution is build through docker, then extract the python wheel from docker.

The Dockerfile used in these instructions specifically targets Raspberry Pi 3/3+ running Raspbian Stretch and Tinker Board running TinkerOS. The same approach should be working for other ARM devices, but may require some changes to the [Dockerfile](https://github.com/lawkee48/TinkerOS_ROS/Dockerfile.arm32v7) such as choosing a different base image (Line 0: FROM ...).
  
Critical changes to make in **Dockerfile.arm32v7**:
- Line0: FROM ... (Please select the specific base image from [Dockerhub](https://hub.docker.com/r/balenalib/raspberrypi3-python/tags?page=1&ordering=last_updated) with the python version that you want)
- Line7: ARG ONNXRUNTIME_SERVER_BRANCH=... (Please select the specific branch of the onnxruntime version that you want to build, you may want to [match with your ONNX version](https://github.com/microsoft/onnxruntime/blob/master/docs/Versioning.md))

1. Install dependencies:

- Install DockerCE

command ```curl -sSL https://get.docker.com/ | sh``` doesn't work, it ends up with an error ```E: Unable to locate package docker-ce-rootless-extras```, because Debian 9 reached EOL and no longer supported, the required package is only available from Ubuntu 20.x (see [here](https://github.com/docker/docker-install/issues/222)).

To solve this problem, follow the manual installation procedure (Section: Install from a package) from https://docs.docker.com/engine/install/debian/. I recommand to select the v19.03.15 of Docker since it works for the [others](https://forums.docker.com/t/google-unable-to-locate-package-docker-ce-rootless-extras/107071/2).
- Install ARM emulator
```
sudo apt-get install -y qemu-user-static
```

2. Create an empty local directory

```
mkdir onnx-build
cd onnx-build
```

3. Save the Dockerfile from this repo to your new directory: [Dockfile.arm32v7](https://github.com/lawkee48/TinkerOS_ROS/Dockerfile.arm32v7)

4. Rund docker build

This will build all the dependencies first, then build ONNX Runtime and its Python bindings. This will take several hours.
```
docker build -t onnxruntime-arm32v7 -f Dockerfile.arm32v7 .
```
  
5. Note the full path of the `.whl` file
  
- Reported at the end of the build after the `# Build Output` line.
- it should be follow the format `onnxruntime-0.3.0-cp35-cp35m-linux_armv7l.whl`, but version number may have changed. You'll use this path to extract the wheel file later.
  
6. Check that the build succeeded

Upon completion, you should see an image tagged `onnxruntime-arm32v7`in your list of docker images:
```
docker images  
```
  
7. Extract the Python wheel file from the docker image 
  
(Update the path/version of the .whl file with the one noted in step 5)
```
docker create -ti --name onnxruntime_temp onnxruntime-arm32v7 bash
docker cp onnxruntime_temp:/code/onnxruntime/build/Linux/MinSizeRel/dist/onnxruntime-0.3.0-cp35-cp35m-linux_armv7l.whl .
docker rm -fv onnxruntime_temp
```
This will save a copy of the wheel file, `onnxruntime-0.3.0-cp35-cp35m-linux_armv7l.whl`, to your working directory on your host machine.
  
8. Copy the wheel file (`onnxruntime-0.3.0-cp35-cp35m-linux_armv7l.whl`) to your Raspberry Pi or other ARM device  
  
9. On device, install the ONNX Runtime wheel file
```
sudo apt-get update
sudo apt-get install -y python3 python3-pip
pip3 install numpy

# Install ONNX Runtime
# Important: Update path/version to match the name and location of your .whl file
pip3 install onnxruntime-0.3.0-cp35-cp35m-linux_armv7l.whl
```

Test by running `import onnxruntime`: Although it will report a warning message as below, it will not affect its functionality.
```
/home/linaro/.local/lib/python3.5/site-packages/onnxruntime/capi/onnxruntime_validation.py:22: UserWarning: Unsupported architecture (32bit). ONNX Runtime supports 64bit architecture, only.
warnings.warn('Unsupported architecture (%s). ONNX Runtime supports 64bit architecture, only.' % __my_arch__)
```

## Test the imx219 camera
```
gst-launch-1.0 rkcamsrc device=/dev/video0 io-mode=4 isp-mode=2A tuning-xml-path=/etc/cam_iq/IMX219.xml ! video/x-raw,format=NV12,width=640,height=480 ! rkximagesink
```
  
## Error message 
1. **ImportError: No module named Cryptodome.Cipher**

Solution:
```
sudo apt install python3-pip python-pip
pip install pycryptodomex
```
2. **ImportError: No module named 'yaml'**

Solution:
```
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
```
3. **rospack error (remain unsolved)**
```
terminate called after throwing an instance of 'rospack::Exception' what(): error parsing manifest of package rosmaster at /opt/ros/melodic/share/rosmaster/package.xml
aborted
```

4. **Could not find sensor_msgs packages**
```
CMake Error at /opt/ros/melodic/share/catkin/cmake/catkinConfig cmake:83 (find_package): Could not find a package configuration file provided by "cv_bridge" with any of the following names:
  
sensor_msgsConfig.cmake
sensor_msgss-config.cmake
```
**Solution:**
```
cd ros_catkin_ws/src/
rm -rf common_msgs/
git clone https://github.com/ros/common_msgs.git
cd ..
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic
source devel_isolated/setup.bash
```
  
5. **Docker build onnxruntime arm32v7l python wheel Error**

Problem with the libseccomp2 package on the host:
```
Fatal Python error: init_interp_main: can't initialize time
Python runtime state: core initialized 
PermissionError: [Errno 1] Operation not permitted
```

**Solution:**
It can be resolved by updating `libseccomp2.deb` package. Downloading and installing a newer `libseccomp2.deb` from source and install it:
```
wget http://ftp.us.debian.org/debian/pool/main/libs/libseccomp/libseccomp2_2.5.1-1_armhf.deb
sudo dpkg -i libseccomp2_2.5.1-1_armhf.deb
```

6. **Cannot find executable path for 'cmake'** 
```  
tools_python_utils [INFO] - flatbuffers module is not installed. parse_config will not be available
build [DEBUG] - Command line arguements:
  --build_dir /code/build/Linux --skip_submodule_sync --config Release --build_wheel --update --build --parallel --cmake_extra_defines ONNXRUNTIME_VERSION=1.8.2
build [ERROR] - Failed to resolve executable path for 'cmake'.
```
  
**Solution:**
Install cmake on your host machine.
  
7. **build cmake error:**

**error 1**
```
CMake Error at Utilities/cmcurl/CMakeLists.txt:525 (message):
  Could not find OpenSSL.  Install an OpenSSL development package or
  configure CMake with -DCMAKE_USE_OPENSSL=OFF to build without OpenSSL.
```
**error 2**
```
CMake Error at CMakeLists.txt:425 (message):
  CMAKE_USE_SYSTEM_ZLIB is ON but a zlib is not found!
Call Stack (most recent call first):
  CMakeLists.txt:710 (CMAKE_BUILD_UTILITIES)
```
**error 3**
```
CMake Error at CMakeLists.txt:480 (message):
  CMAKE_USE_SYSTEM_CURL is ON but a curl is not found!
Call Stack (most recent call first):
  CMakeLists.txt:777 (CMAKE_BUILD_UTILITIES)
```
**Solution 1:**
```
sudo apt-get install openssl
sudo apt-get install libssl-dev
```
**Solution 2:**
```
cd ~/temp
wget https://nchc.dl.sourceforge.net/project/libpng/zlib/1.2.11/zlib-1.2.11.tar.gz
tar -xvf zlib-1.2.11.tar.gz
cd zlib-1.2.11/
./configure --prefix=/usr/local/zlib-1.2.11
make
sudo make install
export LD_LIBRARY_PATH=/usr/local/zlib-1.2.11/lib:$LD_LIBRARY_PATH
```
**Solution 3:**
```
sudo apt-get install curl
sudo apt-get install libssl-dev libcurl4-openssl-dev
```

8. **Catkin build cv_bridge error**
```
CMake Error at /usr/local/share/cmake-3.21/Modules/FindPackageHandleStandardArgs.cmake:230 (message):
 Could NOT find Boost (missing: python3) (found version "1.62.0")
```
**Solution:**

According to the [official Cmake documentation](https://cmake.org/cmake/help/v3.21/module/FindBoost.html), FindBoost cmake-module find_package() function only support Boost 1.67.0 or newer, therefore need to switch to older version of cmake (e.g., default cmake version of 3.7.2 which find_package() support Boost 1.36.0).
Go to the cmake 3.21 source package and uninstall it, e.g.:
```
cd ~/Downloads/cmake-3.21.1/
sudo make uninstall
```
Then you may encounter another issue while catkin build the cv_bridge. 
```
CMake Error at /usr/share/cmake-3.7/Modules/FindBoost.cmake:1831 (message):
  Unable to find the requested Boost libraries.

  Boost version: 1.62.0

  Boost include path: /usr/include

  Could not find the following Boost libraries:

          boost_python3

  No Boost libraries were found.  You may need to set BOOST_LIBRARYDIR to the
  directory containing Boost libraries or BOOST_ROOT to the location of
  Boost.
```
**Solution:**

Accroding to [this issue](https://github.com/ros/ros-overlay/issues/581#issuecomment-445942798), the boost_python3 libraries names got changed in cv_bridge. In /usr/lib/arm-linux-gnueabihf/ for TinkerOS, the library is named libboost_python-py35.so, while it is named libboost_python3.so in the github thread. Creating a symbolic link with this latter name resolved this error.
```
cd /usr/lib/arm-linux-gnueabihf
sudo ln -s libboost_python-py35.so libboost_python3.so
sudo ln -s libboost_python-py35.a libboost_python3.a
```

9. **OpenCV read camera error**
```
AttributeError: 'NoneType' object has no attribute 'shape'
```
**Solution:**
OpenCV need to be compiled with the v4l support enabling. First uninstall the OpenCV you have installed in python3.
```
pip3 uninstall opencv-python
```
Then, make and install the OpenCV from source according to [tinekr board wiki](https://tinkerboarding.co.uk/wiki/index.php/CSI-camera-2.0.8)
```
# Install
sudo apt-get update
# Install a few developer tools
sudo apt-get install -y build-essential git cmake pkg-config
# Install image I/O packages which allow us to load image file formats such as JPEG, PNG, TIFF, etc.
sudo apt-get install -y libjpeg-dev libtiff5-dev libpng-dev
# Install video I/O packages
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
# Install the GTK development library
sudo apt-get install -y libgtk2.0-dev
# Various operations inside of OpenCV (such as matrix operations) can be optimized using added dependencies
sudo apt-get install -y libatlas-base-dev gfortran
# Install the Python 2.7 and Python 3 header files 
sudo apt-get install -y python2.7-dev python3-dev python-opencv
# Others
sudo apt-get install -y python3-pip python3-numpy libtcmalloc-minimal4 glib-2.0 libboost-all-dev

wget -O opencv.zip https://github.com/opencv/opencv/archive/3.3.0.zip
unzip opencv.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.3.0.zip
unzip opencv_contrib.zip
cd ~/opencv-3.3.0/
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D WITH_LIBV4L=ON \
  -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.3.0/modules \
  -D CMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install -j4
sudo ldconfig
```
**Be aware of the those paths according to your own situation**

10. **Crashed Chromium**

After upgraded the Chromium browser in tinker board, you might find chromium is crashed and unable to be used. It might caused by the unstable version of chromium v68 or v69.
**Solution:**
```
wget https://snapshot.debian.org/archive/debian-security/20180701T015633Z/pool/updates/main/c/chromium-browser/chromium_67.0.3396.87-1~deb9u1_armhf.deb
sudo dpkg -i chromium_67.0.3396.87-1~deb9u1_armhf.deb
sudo apt-mark hold chromium
```

## Reference
- ROS installation guide for TinkerBoard. http://nascivera.it/2017/11/20/ros-installation-guide-for-tinkerboard/- ROS Tutorials. http://wiki.ros.org/ROS/Tutorials
- ROS Python2 & Python3 conflict Resolve. https://rancheng.github.io/ros-python2-3-conflict/
- ONNX github. https://github.com/onnx/onnx
- Microsoft onnxruntime github. https://github.com/microsoft/onnxruntime
- NagarajSMurthy RaspberryPi-ONNX-Runtime github. https://github.com/NagarajSMurthy/RaspberryPi-ONNX-Runtime
- ankane onnxruntime github. https://github.com/ankane/onnxruntime-1
- SAj1234 onnxruntime github. https://github.com/Saj1234/onnxruntime
- 菜鸟落泪：debian 9 安装 python 库记录. https://blog.csdn.net/tangshopping/article/details/97265438
- Tinker Board Wiki. CSI camera 2.0.8. https://tinkerboarding.co.uk/wiki/index.php/CSI-camera-2.0.8
