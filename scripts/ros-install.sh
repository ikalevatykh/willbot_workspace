# This script installs ROS Melodic + Willow stack 

# Sudo access is required, at least for:
# - write access in /etc/ros/ 
# - source ROS packages:
#    echo "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" >> /etc/apt/sources.list.d/realsense-public.list 
#    apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
#    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
# - apt install
# - installing libfreenect2

# Another script shows how to install ROS without sudo access (except for apt install as it is available for Thoth team)

export ROS_HOME=$HOME/.ros/
export ROS_OS_OVERRIDE=ubuntu:18.04:bionic

# Check whether Ubuntu version is 18.04
if [[ `lsb_release -r` != *"18.04"* ]]; then
  echo "Oops... Sorry, the installation is expecting Ubuntu 18.04"
  exit 1
fi 

# Check whether ROS can be installed from APT or try to source the packages.
sudo apt install ros-melodic-rviz 2> /tmp/sterr.log
if [ -s /tmp/stderr.log ]
then
	echo "Oops you do not have the sources of ROS melodic. Let's try to install them... In case of error, please use script ros-from-sources.sh"
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	sudo apt-get update
fi

# Check if Python 2 is used
ver=`python -c 'import platform; print(platform.python_version())'`
if [ "$ver" != "2.7"* ]; then
    echo "This script requires you are using python 2.7. You might want to use Pyenv"
    exit 1
fi

# Check if CUDA is installed
if [ ! -f /usr/local/cuda ]; then
    echo "No CUDA? No install."
    exit 1
fi

# Install ROS, its dependancies and source it
# We use a modify version of ROSDEP to avoid writing in /etc/
sudo apt-get install -y ros-kinetic-desktop-full python-rosinstall python-rosinstall-generator python-wstool build-essential
pip install git+https://github.com/guhur/rosdep.git
touch /etc/ros/test.txt 2> /tmp/stderr.log 
if [ -s /tmp/stderr.log ]; then
  export ROSDEP_SOURCE_PATH=$ROS_HOME/rosdep/
  echo "Oh. We don't have access to /etc/ros/. Instead, we will write in $ROS_HOME"
fi
rosdep init
rosdep update
pushd $ROS_HOME/src/
  git clone https://github.com/catkin/catkin_tools.git
  cd catkin_tools
  pip install -r requirements.txt --upgrade
  python setup.py install --record install_manifest.txt
  cd ..
popd 

mkdir -p $HOME/ros_ws/src
cd $HOME/ros_ws/
catkin init
if [ -n "$ZSH_VERSION" ]; then
  echo "source /opt/ros/melodic/setup.zsh" >> $HOME/.zshrc
  source $HOME/.zshrc
elif [ -n "$BASH_VERSION" ]; then
  echo "source /opt/ros/melodic/setup.bash" >> $HOME/.bashrc
  source $HOME/.bashrc
else    
  echo "source /opt/ros/melodic/setup.sh" >> $HOME/.profile
  source $HOME/.profile      
fi

# Check if ROS install worked
if [ -z "$ROS_DISTRO" ]; then
  echo "Oops... I don't find ROS installation... Not cool :("
  exit 1
fi

# Installing drivers for the robot
sudo apt-get install -y ros-melodic-soem ros-melodic-octomap \
    ocl-icd-libopencl1 ocl-icd-opencl-dev opencl-headers clinfo \
    libusb-1.0-0-dev libturbojpeg libjpeg-turbo8-dev libglfw3-dev \
    libopenni2-dev libva-dev libjpeg-dev python-yaml libmodbus-dev  \
    libfreenect-dev ros-melodic-moveit ros-melodic-ros-controllers \
    libglvnd0 libglvnd0:i386 \
    libgl1 libgl1:i386 \
    libglx0 libglx0:i386 \
    libegl1 libegl1:i386 \
    libgles2 libgles2:i386 
pip install -U empy PySide2 pymodbus
mkdir -p $ROS_HOME/OpenCL/vendors && \
    echo "libnvidia-opencl.so.1" > $HOME/.ros/OpenCL/vendors/nvidia.icd

mkdir -p $ROS_HOME/src
pushd $ROS_HOME/src
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    mkdir build && cd build
    # we try to install libfreenect without sudo 
    {
      export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:$HOME/.ros/src/libfreenect2/build 
      cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local/
      make -j8
      make install
      ln -s $HOME/.local/lib/pkgconfig/freenect2.pc $HOME/.local/lib/pkgconfig/libfreenect2.pc
    } || 
    { 
        make -j8
	make install
	cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/
        ln -s /usr/local/lib/pkgconfig/freenect2.pc /usr/local/lib/pkgconfig/libfreenect2.pc
	sudo cd /usr/include && ln -s /usr/include libfreenect
    } 
    sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
    echo "Libfreenect is install. Please replug the Kinect camera. You can test depth processing with: 
	$ export LIBFREENECT2_PIPELINE=cuda
        $ sudo apt-get install openni2-utils && sudo make install-openni2 && NiViewer2"

    cd ../../

popd
cd $HOME/ros_ws/src
git clone --recurse-submodules https://github.com/ikalevatykh/willbot_workspace.git
pushd willbot_workspace
  # Willow version is not working on Melodic so we use APT version
  rm -r moveit
  push robotiq
      git checkout kinectic-devel
  popd
popd
git clone https://github.com/ros-drivers/freenect_stack.git
{ # try to install nvidia drivers
    sudo apt install -y sudo apt install -y librealsense2-dev librealsense2-dkms librealsense2-utils
} || { # if it does not work, we just IGNORE realsense packages
  touch willbot_workspace/realsense/realsense2_camera/CATKIN_IGNORE
  touch willbot_workspace/realsense/ddynamic_reconfigure/CATKIN_IGNORE
}
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source catkin profile
devel=$HOME/ros_ws/devel
if [ -n "$ZSH_VERSION" ]; then
  echo "source $devel/setup.zsh" >> $HOME/.zshrc
  source $HOME/.zshrc
elif [ -n "$BASH_VERSION" ]; then
  echo "source $devel/setup.bash" >> $HOME/.bashrc
  source $HOME/.bashrc
else
  echo "source $devel/setup.sh" >> $HOME/.profile
  source $HOME/.profile
fi

