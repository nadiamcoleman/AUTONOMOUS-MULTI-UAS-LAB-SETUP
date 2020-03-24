## Install ROS Kinetic (~20 minutes)
  * Follow [these instructions](https://github.com/intel-aero/meta-intel-aero/wiki/05-Autonomous-drone-programming-with-ROS)

1. Set up
  ```
  sudo add-apt-repository http://packages.ros.org/ros/ubuntu
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
  sudo apt update
  ```
  
* Install ROS desktop version and additional packages
  
  ```
  sudo apt -y install ros-kinetic-desktop-full ros-kinetic-rqt python-rosinstall \
    ros-kinetic-realsense-camera ros-kinetic-mavros ros-kinetic-web-video-server \
    ros-kinetic-visp-tracker ros-kinetic-visp-camera-calibration ros-kinetic-vision-visp \
    ros-kinetic-vision-opencv ros-kinetic-video-stream-opencv ros-kinetic-uvc-camera \
    ros-kinetic-usb-cam ros-kinetic-test-mavros ros-kinetic-rviz-visual-tools \
    ros-kinetic-rostopic ros-kinetic-roslaunch python-rosinstall python-rosinstall-generator \
    python-wstool build-essential ros-kinetic-pyros python-rosdep

  sudo rosdep init
  rosdep update
  sudo geographiclib-get-geoids egm96-5
  sudo apt autoremove
  ```
  
* Create Catkin workspace
  ```
  source /opt/ros/kinetic/setup.bash
  mkdir -p ~/ros_ws/src
  cd ~/ros_ws/
  catkin_make
  ```
  
* Set up ROS Environment (auto-load in bashrc)
  ```
  su
  echo "# ROS Settings" | tee -a /home/intel2/.bashrc > /dev/null
  echo "source /opt/ros/kinetic/setup.bash" | tee -a /home/intel2/.bashrc > /dev/null
  echo "source /home/intel2/ros_ws/devel/setup.bash" | tee -a /home/intel2/.bashrc > /dev/null
  echo "export ROS_MASTER_URI=http://localhost:11311" | tee -a /home/intel2/.bashrc > /dev/null
  echo "#export ROS_MASTER_URI=http://192.168.1.88:11311" | tee -a /home/intel2/.bashrc > /dev/null
  echo "#export ROS_IP=192.168.1.122" | tee -a /home/intel2/.bashrc > /dev/null
  exit
  ```
      
* Set up for 3D Mapping and Path-Planning
  * Install [OctoMap](https://www.ros.org/wiki/octomap_server)
    * `sudo apt install ros-kinetic-octomap-server`
  * Install [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)
    * `sudo apt install ros-kinetic-rtabmap-ros`

* Create a custom package
  * ```
  cd ~/ros_ws/src/
  catkin_create_pkg my_package rospy roscpp std_msgs geometry_msgs sensor_msgs cv_bridge 
  ```
* Build only one or more specified package(s)
  * List packages to build, separated by ';'
    * `catkin_make -DCATKIN_WHITELIST_PACKAGES="pkg1;pkg2"`
