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
  
  
# How to Setup an Autonomous Multi-UAS Laboratory
1. The first step to buidling your own UAV lab starts with router setup, netted area setup, and Vicon setup. 
You will want a good size netted area space to fly in. You will also want some guidlines and saftey rules for operations inside the space.  

2. You will also want some kind of motion capture system. One of the most widely used systems is the Vicon motion capture system which is the system we utilized for our platform. We will not cover specific instructions for setting up the entirety of the Vicon system however we will offer guidance on how to stream vicon position data using ROS. The basics of the Vicon system include the system itself i.e. cameras, markers, other hardware mand also a computer, in our case, windows with the neccessary software (Nexus, and Tracker). 

3. Next you need a wifi router, with both wirless and wired capability (wifi and ethernet). You will need to set up a DHCP server so that you can assign static IPs for all the machines you will use such as computers, UAVs, or other robotic system. 

4. Next you will need an Ubuntu machine to serve as the "base station" for all your ROS communications. This computer will be the control center that sends out all control commands in the form of rostopic messages and also routes position data from the computer running the Vicon software
5. After all the previous conditions have been met you can start to setup your UAS platforms. 
