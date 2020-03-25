# AUTONOMOUS MULTI-UAS CODE
This directory holds all the key files we used to operate 2+ UAVs autonomously using a variety of tools including:

ROS
Python
C++
PX4
Mavros
MAVlink
QGC

This repository includes a ROS workspace folder we created and used to run all of the relevent programs for operating our multi-UAS platform such as code for: 
* Streaming vicon data on ROS
* Running mavros natively on each UAS
* Python Scripts for autonomous flight 

## The following steps show how to use the code provided to operate 2+ autonomous UAVs:
1. Start up the Vicon system
* Power on Vicon system, and computer running Vicon Nexus software. 
* Make sure this computer is connected to the same network as all other machines such as the base station computer and UAVs. 
* Use "ifconfig" to verify the IP of the computer to use for later.
* Open and configure Nexus software to start streaming position data.

2. Run roscore on the Base Station computer. 
* ```
  roscore
  ```
3. Start streaming the vicon data over the "/mavros/vision_pose/pose" rostopic
* ```
  roslaunch msral vicon_stream_multi.launch ip:="IP_OF_COMPUTER_RUNNING_NEXUS"
  ```
4. Stop logging flight data on each UAV (this will also momentarily stop communication with QGC).
  * ssh into the UAV
    * ```
      ssh hostname@ip_adress
      ```
  * Stop the mavlink router
    * ```
      sudo systemctl stop mavlink-router
      ```
  * Note: You may wish to check your flight log files, save or extract important ones and clear out the rest. 
5. Start logging flight data on each UAV when you are ready to begin the test (this will also restart communication with QGC).
  * ssh into the UAV
    * ```
      ssh hostname@ip_adress
      ```
  * Stop the mavlink router
    * ```
      sudo systemctl start mavlink-router
      ```
  * Note: You may wish to check your flight log files, save or extract important ones and clear out the rest. Flight logs are stored on the UAV at '/var/lib/mavlink-router' .
5. Start Mavros on each UAV you plan to fly
  * ```
    roslaunch msral mavros.launch
    ```

