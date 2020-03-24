# AUTONOMOUS MULTI-UAS LAB SETUP
This repository contains information and files for setting up an autonomous multi-UAS laboratory. Two research based unmanned aerial vehicle platforms were used in our multi-UAS lab setup however these instructions can be apllied to a multitude of platforms. The two platforms we used were the Intel Aero RTF Drone, and the UVify Draco-R. This repo will include hardware information, software information and code that we used to setup our autonomous multi-UAS laboratory.

## Equipment Used
### UAS platforms:
1. 2 x [Intel Aero RTF Quadrotor](https://www.intel.com/content/www/us/en/support/articles/000023271/drones/development-drones.html "info and specs")
  * [Ubuntu 16.04.3 LTS](https://ubuntu.com/download/alternative-downloads)
  * [PX4](https://px4.io) [Firmware 1.8.2](https://github.com/PX4/Firmware/tree/v1.8.2)
  * [ROS Kinetic](http://wiki.ros.org/kinetic)    
  
2. 2 x [Draco-R Hexacopter](https://www.uvify.com/draco-r/ "info and specs") 
  * [Ubuntu 18.04.3 LTS](https://ubuntu.com/download/alternative-downloads)
  * [PX4](https://px4.io) [Firmware 1.9.0](https://github.com/PX4/Firmware/tree/v1.9.0)
  * [ROS Melodic](http://wiki.ros.org/melodic)

Note: These are the UAVs we chose to use however the processes described in this guide can be applied to a multitude of UAV platforms so long as they are PX4 firmware and Linux based. 

3. Power Supplies
  * 2 x 4S Lipo battery with XT60 connector
  * 2 x UVify 4S Lipo battery
  * 2 x [Wall power adapter](https://www.amazon.com/LEDMO-Power-Supply-Transformers-Adapter/dp/B01461MOGQ/ref=sr_1_3_sspa?keywords=12V+5A+Power+Adapter&qid=1567702141&refinements=p_72%3A1248909011&rnid=1248907011&s=hi&sr=1-3-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEyWUtYRVhZN0JPM0lHJmVuY3J5cHRlZElkPUEwODU1MTU5MUFQMjJZSFc5VkZESiZlbmNyeXB0ZWRBZElkPUEwMzEyMTU5MkxBUTNTREg2VjlEMyZ3aWRnZXROYW1lPXNwX2F0ZiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU=)

### Computers:
1. Base Station Computer
  * [Ubuntu 18.04.3 LTS](https://ubuntu.com/download/alternative-downloads)
  * [ROS Melodic](http://wiki.ros.org/melodic)

2. Vicon Base Computer
  * Windows Vista
  * Vicon Nexus Software
  * Vicon Tracker Software
        
### Motion Capture System
1. [Vicon Motion Capture System](https://www.vicon.com)
  * 12 Vicon T160 motion/position tracking cameras.  
        
### Asus Wifi Router
  * DCHP server setup for static IP addressing for all aforementioned machines. 

### Netted Lab Space
  * Dimensions: 43' x 28' x 20' 
