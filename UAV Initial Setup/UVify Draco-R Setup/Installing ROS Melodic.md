## Install ROS melodic (~20 minutes)
  * Follow [these instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)

1. Set up
  ```
  sudo add-apt-repository http://packages.ros.org/ros/ubuntu
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
  sudo apt update
  ```
  
* Install ROS desktop version and additional packages
  
  ```
  sudo apt -y install ros-melodic-desktop-full ros-melodic-rqt python-rosinstall \
    ros-melodic-realsense-camera ros-melodic-mavros ros-melodic-web-video-server \
    ros-melodic-visp-tracker ros-melodic-visp-camera-calibration ros-melodic-vision-visp \
    ros-melodic-vision-opencv ros-melodic-video-stream-opencv ros-melodic-uvc-camera \
    ros-melodic-usb-cam ros-melodic-test-mavros ros-melodic-rviz-visual-tools \
    ros-melodic-rostopic ros-melodic-roslaunch python-rosinstall python-rosinstall-generator \
    python-wstool build-essential ros-melodic-pyros python-rosdep

  sudo rosdep init
  rosdep update
  sudo geographiclib-get-geoids egm96-5
  sudo apt autoremove
  ```
  
* Create Catkin workspace
  ```
  source /opt/ros/melodic/setup.bash
  mkdir -p ~/ros_ws/src
  cd ~/ros_ws/
  catkin_make
  ```
  
* Set up ROS Environment (auto-load in bashrc)
  ```
  su
  echo "# ROS Settings" | tee -a /home/intel2/.bashrc > /dev/null
  echo "source /opt/ros/melodic/setup.bash" | tee -a /home/intel2/.bashrc > /dev/null
  echo "source /home/intel2/ros_ws/devel/setup.bash" | tee -a /home/intel2/.bashrc > /dev/null
  echo "export ROS_MASTER_URI=http://localhost:11311" | tee -a /home/intel2/.bashrc > /dev/null
  echo "#export ROS_MASTER_URI=http://192.168.1.88:11311" | tee -a /home/intel2/.bashrc > /dev/null
  echo "#export ROS_IP=192.168.1.122" | tee -a /home/intel2/.bashrc > /dev/null
  exit
  ```
      
* Set up for 3D Mapping and Path-Planning
  * Install [OctoMap](https://www.ros.org/wiki/octomap_server)
    * `sudo apt install ros-melodic-octomap-server`
  * Install [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)
    * `sudo apt install ros-melodic-rtabmap-ros`

* Create a custom package
  * ```
  cd ~/ros_ws/src/
  catkin_create_pkg my_package rospy roscpp std_msgs geometry_msgs sensor_msgs cv_bridge 
  ```
* Build only one or more specified package(s)
  * List packages to build, separated by ';'
    * `catkin_make -DCATKIN_WHITELIST_PACKAGES="pkg1;pkg2"`
