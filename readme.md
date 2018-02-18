# HyphaROS MiniBot pkg
![alt text](https://github.com/Hypha-ROS/hypharos_minibot/blob/master/document/logo/HyphaROS_logo_2.png)  

## Abstract
Low cost, user friendly Pi3 based mini robot for ROS developing !   
Fully open-sourced (hardware & software), total cost <300USD.  

![alt text](https://drive.google.com/open?id=1Csf0uPHfUZ_SAkWxBnUl-ce7ABnQK2LH)  

## About us
FB Page: https://www.facebook.com/HyphaROS/  
Website: https://hypharosworkshop.wordpress.com/  
Contact: hypha.ros@gmail.com  
Developer:   
* HaoChih, LIN  

Date: 2018/02/18  
License: Apache 2.0  

## Features
* Onboard mapping (ICP, Gmapping)  
* STM32 for CL motor speed control  
* AMCL localization  
* Dynamic obstacle avoidance  

## Roadmap
* Documentation  
* ROS 2.0 Multi-robots
* Video tutorial  

## Hardware 
* Raspberry Pi3
* YDLidar X4
* STM32(F103) MCU (with OLED display, bluetooth)
* Diff Motor with A/B encoder(res: 340)
* MPU6050 or GY85
Total Cost: < 300 USD  

## Software
### Pi3 Image
Image file for Pi3.(with SD card >=16G)  
[Google drive] will be released soon  

### Desktop Windows 
VirtualBox Image：  
will be released soon  

### Desktop Ubuntu (16.04) 
64bit RAM > 4G  
Install ROS Kinetic - (Desktop-Full Install)   (http://wiki.ros.org/kinetic/Installation/Ubuntu)  

$ sudo apt-get install remmina synaptic gimp git ros-kinetic-navigation ros-kinetic-amcl ros-kinetic-slam-gmapping ros-kinetic-mrpt-slam ros-kinetic-mrpt-icp-slam-2d ros-kinetic-robot-localization -y  

create your own catkin_ws   
(http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)  
$ cd catkin_ws/src  
$ git clone https://github.com/EAIBOT/ydlidar  
$ git clone https://github.com/Hypha-ROS/hypharos-minibot   
$ cd ..  
$ catkin_make  


