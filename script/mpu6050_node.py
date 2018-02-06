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
from tf.transformations import quaternion_from_euler

class IMUNode:
    def __init__(self):
        # ROS Parameters:
        self.ori_cov = rospy.get_param('~ori_cov', '0.0025') # Orientation covariance
        self.vel_cov = rospy.get_param('~vel_cov', '0.02') # Angular velocity covariance
        self.acc_cov = rospy.get_param('~acc_cov', '0.04') # Linear acceleration covariance
        self.imu_i2c = rospy.get_param('~imu_i2c', '0x68') # I2C device No
        self.imu_link = rospy.get_param('~imu_link', 'imu_link') # imu_link name
        self.pub_freq = float( rospy.get_param('~pub_freq', '50') ) # hz of imu pub

        self.imuMsg = Imu()
        # Orientation covariance matrix:
        for i in range(9):
            self.imuMsg.orientation_covariance[i] = 0.0
        self.imuMsg.orientation_covariance[0] = self.ori_cov
        self.imuMsg.orientation_covariance[4] = self.ori_cov
        self.imuMsg.orientation_covariance[8] = self.ori_cov

        # Angular velocity covariance matrix:
        for i in range(9):
            self.imuMsg.angular_velocity_covariance[i] = 0.0
        self.imuMsg.angular_velocity_covariance[0] = self.vel_cov
        self.imuMsg.angular_velocity_covariance[4] = self.vel_cov
        self.imuMsg.angular_velocity_covariance[8] = self.vel_cov

        # Linear acceleration covariance matrix:
        for i in range(9):
            self.imuMsg.linear_acceleration_covariance[i] = 0.0
        self.imuMsg.linear_acceleration_covariance[0] = self.acc_cov
        self.imuMsg.linear_acceleration_covariance[4] = self.acc_cov
        self.imuMsg.linear_acceleration_covariance[8] = self.acc_cov
                
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

        # Auto yaw-calibration:
        rospy.loginfo("Auto yaw calibration...")
        count = 0.0
        vyaw_sum = 0.0
        yaw_rad = 0.0
        for x in range(0, 300):
            try:
                gyro_data = self.sensor.get_gyro_data()
                vyaw_sum = vyaw_sum + float(gyro_data['z'])
                count = count + 1.0
            except:
	            rospy.logwarn("Error in Sensor values")
	            pass

        self.vyaw_bias = float(vyaw_sum/count)
        rospy.loginfo("Bias of Vyaw is(rad): %f", self.vyaw_bias)

        # ROS handler        
        self.pub = rospy.Publisher('/imu_data', Imu, queue_size=1)   
        self.timer_pub = rospy.Timer(rospy.Duration(1.0/self.pub_freq), self.timerCB) 

        # Variables
        self.yaw_rad = 0
        self.seq = 0
        self.dt = 0
        self.time_curr = rospy.Time.now()
        self.time_prev = self.time_curr
    
    def timerCB(self, event):    
        #I2C Serial read & publish 
        try:           
            gyro_data = self.sensor.get_gyro_data()
            self.yaw_rad = -( float(gyro_data['z']) - self.vyaw_bias )*self.dt + self.yaw_rad
            if (self.yaw_rad < -math.pi):
                self.yaw_rad += 2*math.pi
            if (self.yaw_rad > math.pi):
                self.yaw_rad -= 2*math.pi
            self.time_curr = rospy.Time.now()
            self.dt = (self.time_curr - self.time_prev).to_sec();
            self.time_prev = self.time_curr
            print gyro_data['z']
           
            q = quaternion_from_euler(0,0,self.yaw_rad)
            self.imuMsg.orientation.x = q[0]
            self.imuMsg.orientation.y = q[1]
            self.imuMsg.orientation.z = q[2]
            self.imuMsg.orientation.w = q[3]
            self.imuMsg.header.stamp= self.time_curr
            self.imuMsg.header.frame_id = self.imu_link
            self.imuMsg.header.seq = self.seq
            self.seq = self.seq + 1
            self.pub.publish(self.imuMsg)
   
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
