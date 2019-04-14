# This file installs ROS Melodic from sources without using sudo (except sudo apt)

# The core idea of the installation is that : 
# - first, we need to install ROS Comm, 
# - then we source ROS and
# - and finally, we can install whatever we want.

# Checking the environment
# ------------------------ 
# Python version should be 2.7.15. The script fails for 2.7.16 and 3.6.8
ver=`python -c 'import platform; print(platform.python_version())'`
if [ "$ver" != "2.7.15" ]; then
    echo "This script requires python 2.7.15. You might want to use Pyenv"
    exit 1
fi

# Basic tools for ROS
# -------------------
sudo apt-get install -y python-rosinstall-generator python-wstool python-rosinstall build-essential

# We use a modified version of ROSDEP to avoid writting in /etc/ (forbidden in thoth machines)
pip install git+https://github.com/guhur/rosdep.git
pip install catkin_tools
export ROS_HOME=$HOME/ros/
export ROSDEP_SOURCE_PATH=$ROS_HOME/rosdep/
export ROS_OS_OVERRIDE=ubuntu:18.04:bionic
rosdep init
rosdep update

# Let's install ROS Comm
# ----------------------
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator ros_comm --rosdistro melodic --deps --tar > melodic-ros_comm.rosinstall
wstool init -j8 src melodic-ros_comm.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
# We need to specify a flag against SETUPTOOLS to compile catkin. 
# See https://github.com/ros/catkin/issues/863#issuecomment-290392074
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

# Sourcing ROS
# ------------
cd $HOME/ros_catkin_ws/install_isolated/
if [ -n "$ZSH_VERSION" ]; then
  echo "source $(pwd)/setup.zsh" >> $HOME/.bashrc
  source setup.zsh
elif [ -n "$BASH_VERSION" ]; then
  echo "source $(pwd)/setup.bash" >> $HOME/.bashrc
  source setup.bash
else
  echo "source $(pwd)/setup.sh" >> $HOME/.bashrc
  source setup.sh
fi

# Installing ROS Desktop
# ----------------------
cd $HOME/ros_catkin_ws
mv melodic-ros_comm.install melodic-ros_comm.install.bak
rosinstall_generator desktop --rosdistro melodic --deps --tar > melodic-desktop.rosinstall
wstool merge -t src melodic-desktop.rosinstall
wstool update -j32 -t src
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

# Installing manually the Kinect's driver and other dependencies
# --------------------------------------------
sudo apt-get install -y libturbojpeg0-dev libglfw3-dev libusb-1.0-0-dev libturbojpeg libjpeg-turbo8-dev libglfw3-dev libopenni2-dev libva-dev libjpeg-dev libmodbus-dev python3-yaml ocl-icd-libopencl1 ocl-icd-opencl-dev opencl-headers clinfo python3-catkin-pkg-modules python3-rospkg-modules

mkdir -p $HOME/src
pushd $HOME/src
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local/
    make -j8
    make install 
    ln -s $HOME/.local/lib/pkgconfig/freenect2.pc $HOME/.local/lib/pkgconfig/libfreenect2.pc
    cd ../../
popd

# Install repo from Willow
# ------------------------
cd $HOME/ros_catkin_ws/src/
git clone --recurse-submodules https://github.com/ikalevatykh/willbot_workspace.git
pushd willbot_workspace
  git submodule init
  git submodule update --recursive --remote
popd
git clone https://github.com/ros-drivers/freenect_stack.git
git clone https://github.com/ros-drivers/rgbd_launch.git
git clone https://github.com/ros-controls/ros_controllers.git
git clone https://github.com/rstrudel/willbot_sim2real.git -b grenoble
cd ..
rosdep install --from-paths src --ignore-src -r -y
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF







