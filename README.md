# Willbot Workspace

Willbot workspace is a collection of core ROS packages for the INRIA UR5 + Robotiq 3F setup.
This repository is a "superproject" where packages joined as submodules ([more about submodules in git](https://git-scm.com/docs/git-submodule)). For each submodule specified branch and commit, so one can update all packages synchronously.

To clone packages use:

```
git clone --recurse-submodules https://github.com/ikalevatykh/willbot_workspace.git
```

Then to update packages to the latest version use:

```
git submodule init
git submodule update --recursive --remote
```

# How to install and configure INRIA setup

## Install ROS Kinetic

See full documentation for [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).
You can see also install scripts in [scripts/](scripts/). These scripts are experimental.

1. Install ROS desktop full

   ```
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
   sudo apt-get update
   sudo apt-get install ros-kinetic-desktop-full
   ```
2. Initialize rosdep

   ```
   sudo apt-get install python-rosdep
   sudo rosdep init
   rosdep update
   ```

3. Environment setup

   ```
   echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

4. Dependencies for building packages

   ```
   sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
   ```

5. Check that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set

   ```
   printenv | grep ROS
   ```

## Install non-ROS dependencies

1. Install NVIDIA GPU driver

   In Paris we use version 396.54

2. Download and install CUDA Toolkit 9.2

   [Instructions](https://developer.nvidia.com/cuda-92-download-archive).

3. Install OpenCL

   ```
   sudo apt-get install nvidia-opencl-icd-396 opencl-headers
   ```

4. Install libfreenect2 (for Kinect2 support)

   See [install instructions](https://github.com/OpenKinect/libfreenect2) for more details.
   
   Install build tools
    ```
    sudo apt-get install build-essential cmake pkg-config
    ```
   Install dependencies
    ```
    sudo apt-get install libusb-1.0-0-dev libturbojpeg libjpeg-turbo8-dev libglfw3-dev libopenni2-dev libva-dev libjpeg-dev
    ```
   Download libfreenect2 source, build and install
    ```
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
    make -j8
    make install    
    ``` 
   Set up udev rules for device access: `sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/`, then replug the Kinect.
    
5. Install libmodbus

   ```
   sudo apt-get install libmodbus-dev
   ```

6. Install yaml for using with a python 3.x

   ```
   sudo apt-get install python3-yaml
   ```
   
7. Install librealsense

   [Install using pre-build packages](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).
   
   Restart a machine.


## Install Catkin tools 

```
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```

[See Catkin Tools documentation for more details](https://catkin-tools.readthedocs.io/en/latest/index.html).


## Create a ROS Workspace

1. Create a workspace

   ```
   mkdir -p ~/ros_kinetic_ws/src
   cd ~/ros_kinetic_ws/
   catkin init
   ```

2. Environment setup

   ```
   echo "source ~/ros_kinetic_ws/devel/setup.bash" >> ~/.bashrc
   source devel/setup.bash
   ```

3. Check that ROS_PACKAGE_PATH are set correctly

   ```console
   foo@bar:~$ echo $ROS_PACKAGE_PATH
   /home/youruser/ros_kinetic_ws/src:/opt/ros/kinetic/share
   ```

4. Clone packages to the workspace

   ```
   cd ~/ros_kinetic_ws/src
   git clone --recurse-submodules https://github.com/ikalevatykh/willbot_workspace.git
   ```

5. Install dependencies

   ```
   cd ~/ros_kinetic_ws/
   rosdep install --from-paths src --ignore-src -r -y
   ```

6. Build packages

   For best performance choose the Release mode.

   ```
   cd ~/ros_kinetic_ws/
   catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Release
   source devel/setup.bash
   ```

   During the building you may have several warning messages, it is normal.
   If you have error messages about missing dependencies or other errors, please notify me.

   Check if building succeeded:

   ```console
   [build]   Summary: All xx packages succeeded!  
   [build]   Abandoned: None.                                                                                                                            
   [build]   Failed:    None. 
   ```

7. Check that packages linked correctly

   ```console
   foo@bar:~$ rosls willbot_bringup
   CMakeLists.txt  config  launch  package.xml  script
   ```

# Check setup configured correctly

1. Check if you can connect to the robot and the gripper

   Enable hardware, ping the arm and the gripper.

   ```
   roslaunch willbot_bringup willbot_real.launch setup:=grenoble
   ```
   or
   ```
   roslaunch willbot_bringup willbot_real.launch setup:=paris
   ```

   You may have several warning, it is normal.

   The gripper should close and open, and in a console should appear a message:

   ```console
   You can start planning now!
   ```

   If in a console you have some errors, please stop and notify me.

2. Check if you can move robot and gripper

   ```
   roslaunch willbot_bringup rviz.launch
   ```

   If you see that robot model in rviz window does not correspond a real robot pose, please stop and notify me.


   Then use an Rviz interface to move the arm (planning group manipulator) and the gripper (planning group gripper).

   See [MoveIt! Quickstart in RViz](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) for details.


# How to update INRIA setup to the latest version

1. Pull changes

   ```
   cd ~/ros_kinetic_ws/src/willbot_workspace/
   git pull
   git submodule update --recursive --remote
   ```

2. Rebild the workspace

   ```
   cd ~/ros_kinetic_ws/
   catkin build
   source devel/setup.bash
   ```
