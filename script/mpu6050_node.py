#!/usr/bin/python
# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import time
import sys
import math
from mpu6050 import mpu6050
from sensor_msgs.msg import Imu

class IMUNode:
    def __init__(self):
        # ROS Parameters:
        self.ori_cov = float(rospy.get_param('~ori_cov', '0.0025') ) # Orientation covariance
        self.vel_cov = float(rospy.get_param('~vel_cov', '0.02') ) # Angular velocity covariance
        self.acc_cov = float(rospy.get_param('~acc_cov', '0.04') ) # Linear acceleration covariance
        self.imu_i2c = rospy.get_param('~imu_i2c', '0x68') # I2C device No
        self.imu_link = rospy.get_param('~imu_link', 'imu_link') # imu_link name
        self.pub_freq = float( rospy.get_param('~pub_freq', '50') ) # hz of imu pub
                
        # I2C Communication:
        try:
            self.sensor = mpu6050(0x68)
            rospy.loginfo("Flusing first 50 data readings ...")
            for x in range(0, 50):
                gyro_data = self.sensor.get_gyro_data()
                time.sleep(0.01)

        except:
            rospy.logerr("Can not receive data from the I2C device: "+ self.imu_i2c + 
            ". Did you specify the correct No. ?")
            sys.exit(0) 
        rospy.loginfo("Communication success !")

        # ROS handler        
        self.pub = rospy.Publisher('imu_data', Imu, queue_size=1)   
        self.timer_pub = rospy.Timer(rospy.Duration(1.0/self.pub_freq), self.timerCB) 

    def timerCB(self, event):    
        #I2C Serial read & publish 
        try:           
            gyro_data = self.sensor.get_gyro_data()
            accel_data = self.sensor.get_accel_data()

            # Publish imu raw data
            imuMsg = Imu()
            imuMsg.header.stamp= rospy.Time.now()
            imuMsg.header.frame_id = self.imu_link
            imuMsg.orientation_covariance[0] = self.ori_cov
            imuMsg.orientation_covariance[4] = self.ori_cov
            imuMsg.orientation_covariance[8] = self.ori_cov
            imuMsg.angular_velocity_covariance[0] = self.vel_cov
            imuMsg.angular_velocity_covariance[4] = self.vel_cov
            imuMsg.angular_velocity_covariance[8] = self.vel_cov
            imuMsg.linear_acceleration_covariance[0] = self.acc_cov
            imuMsg.linear_acceleration_covariance[4] = self.acc_cov
            imuMsg.linear_acceleration_covariance[8] = self.acc_cov
            imuMsg.angular_velocity.x = float(gyro_data['x'])
            imuMsg.angular_velocity.y = float(gyro_data['y'])
            imuMsg.angular_velocity.z = float(gyro_data['z'])
            imuMsg.linear_acceleration.x = float(accel_data['x'])
            imuMsg.linear_acceleration.y = float(accel_data['y'])
            imuMsg.linear_acceleration.z = float(accel_data['z'])
            self.pub.publish(imuMsg)
        except: 
            rospy.loginfo("Error in sensor value !") 	          
            pass            
        
if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('mpu6050_node', anonymous=True)

        # Constract IMUNode Obj
        rospy.loginfo("Start Reading ...")
        imu = IMUNode()
        rospy.spin()
    except KeyboardInterrupt:           
        print("Shutting down")
