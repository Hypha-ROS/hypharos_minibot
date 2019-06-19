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
import tf
import time
import sys
import math
import serial
import string
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import struct

class BaseControl:
    def __init__(self):
        # Get params
        self.baseId = rospy.get_param('~base_id', 'base_footprint') # base link
        self.odomId = rospy.get_param('~odom_id', 'odom') # odom link
        self.device_port = rospy.get_param('~port', '/dev/stm32base') # device port
        self.baudrate = float( rospy.get_param('~baudrate', '115200') ) # baudrate
        self.odom_freq = float( rospy.get_param('~odom_freq', '20') ) # hz of odom pub
        self.wheelSep = float( rospy.get_param('~wheel_separation', '0.158') ) # unit: meter 
        self.wheelRad = float( rospy.get_param('~wheel_radius', '0.032') ) # unit: meter
        self.VxCov = float( rospy.get_param('~vx_cov', '1.0') ) # covariance for Vx measurement
        self.VyawCov = float( rospy.get_param('~vyaw_cov', '1.0') ) # covariance for Vyaw measurement
        self.odom_topic = rospy.get_param('~odom_topic', '/odom') # topic name
        self.pub_tf = bool(rospy.get_param('~pub_tf', True)) # whether publishes TF or not
        self.debug_mode = bool(rospy.get_param('~debug_mode', False)) # true for detail info        

        # Serial Communication
        try:
            self.serial = serial.Serial(self.device_port, self.baudrate, timeout=10)
            rospy.loginfo("Flusing first 10 data readings ...")
            for x in range(0, 10):
                data = self.serial.read()
                time.sleep(0.01)

        except serial.serialutil.SerialException:
            rospy.logerr("Can not receive data from the port: "+ self.device_port + 
            ". Did you specify the correct port ?")
            self.serial.close
            sys.exit(0) 
        rospy.loginfo("Communication success !")

        # ROS handler        
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.cmdCB, queue_size=10)
        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)   
        self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.odom_freq), self.timerOdomCB) 
        self.timer_cmd = rospy.Timer(rospy.Duration(0.1), self.timerCmdCB) # 10Hz
        self.tf_broadcaster = tf.TransformBroadcaster() # TF

        # variable        
        self.trans_x = 0.0 # cmd
        self.rotat_z = 0.0
        self.WL_send = 0.0
        self.WR_send = 0.0
        self.current_time = rospy.Time.now()
        self.previous_time = rospy.Time.now()
        self.pose_x = 0.0 # SI
        self.pose_y = 0.0
        self.pose_yaw = 0.0

        # reading loop 
        while True:         
            reading = self.serial.read(1)
            if reading == '0x53':
                functioon_code = self.serial.read(1)
                if functioon_code == '0x42':        #'B'
                    serial_buf = self.serial.read(5)
                    if serial_buf[4] == '0x45':      #'E'
                        self.batt_fb = serial_buf
                        print 'Batt fb'

                if functioon_code == '0x56':        #'V'
                    serial_buf = self.serial.read(13)
                    if serial_buf[12] == '0x45':      #'E'
                        self.vel_fb = serial_buf
                        print 'vel fb'
            else:
                self.serial.read(1)

    def cmdCB(self, data):
        self.trans_x = data.linear.x
        self.rotat_z = data.angular.z
    
    def timerOdomCB(self, event):
        # Serial read & publish 
        try:           
            vel_fb = self.vel_fb
            if len(vel_fb) == 13:
                VR = float(struct.unpack('f', vel_fb[0:3])[0])    #unit: m/s
                VL = float(struct.unpack('f', vel_fb[4:7])[0])
                gyro_z = float(struct.unpack('f', vel_fb[8:11])[0])    #unit: degree/s
            else:
                print 'vel_fb Error!'

            Vyaw = (VR-VL)/self.wheelSep
            Vx = (VR+VL)/2.0

            # Pose
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.previous_time).to_sec()
            self.previous_time = self.current_time
            self.pose_x   = self.pose_x   + Vx * math.cos(self.pose_yaw) * dt
            self.pose_y   = self.pose_y   + Vx * math.sin(self.pose_yaw) * dt
            self.pose_yaw = self.pose_yaw + Vyaw * dt
            pose_quat = tf.transformations.quaternion_from_euler(0,0,self.pose_yaw)
            
            # Publish Odometry
            msg = Odometry()
            msg.header.stamp = self.current_time
            msg.header.frame_id = self.odomId
            msg.child_frame_id  = self.baseId
            msg.pose.pose.position.x = self.pose_x
            msg.pose.pose.position.y = self.pose_y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.x =  pose_quat[0]
            msg.pose.pose.orientation.y =  pose_quat[1]
            msg.pose.pose.orientation.z =  pose_quat[2]
            msg.pose.pose.orientation.w =  pose_quat[3]
            msg.twist.twist.linear.x = Vx
            msg.twist.twist.angular.z = Vyaw
            for i in range(36):
                msg.twist.covariance[i] = 0
            msg.twist.covariance[0] = self.VxCov
            msg.twist.covariance[35] = self.VyawCov
            msg.pose.covariance = msg.twist.covariance
            self.pub.publish(msg)

            # TF Broadcaster
            if self.pub_tf:
                self.tf_broadcaster.sendTransform( (self.pose_x, self.pose_y, 0.0), pose_quat, self.current_time, self.baseId, self.odomId)          

            # Debug mode                      
            if self.debug_mode: 
                if len(data) == 6:
                    header_1 = int(data[0].encode('hex'),16)
                    header_2 = int(data[1].encode('hex'),16)
                    tx_1 = int(data[2].encode('hex'),16)
                    tx_2 = int(data[3].encode('hex'),16)
                    tx_3 = int(data[4].encode('hex'),16)
                    tx_4 = int(data[5].encode('hex'),16) 
                    rospy.loginfo("[Debug] header_1:%4d, header_2:%4d, tx_1:%4d, tx_2:%4d, tx_3:%4d, tx_4:%4d", header_1, header_2, tx_1, tx_2, tx_3, tx_4 )
            
        except: 
            #rospy.loginfo("Error in sensor value !") 
            pass            

    def timerCmdCB(self, event):
        # send cmd to motor
        WR = (self.trans_x + self.wheelSep/2.0*self.rotat_z)/self.wheelRad; # unit: rad/sec
        WL = (self.trans_x - self.wheelSep/2.0*self.rotat_z)/self.wheelRad;        


        self.WR_send = WR * self.wheelRad # unit: m/sec
        self.WL_send = WL * self.wheelRad
        WR_send_ba = bytearray(struct.pack("f", self.WR_send))  
        WL_send_ba = bytearray(struct.pack("f", self.WL_send))  
        # R_forward = 1 # 0: reverse, >0: forward  
        # L_forward = 1 # 0: reverse, >0: forward       
        # if self.WR_send < 0:
        #     R_forward = 0
        #     self.WR_send = -self.WR_send
        # if self.WL_send < 0:
        #     L_forward = 0
        #     self.WL_send = -self.WL_send
        # if self.WR_send > 255:
        #     self.WR_send = 255
        # if self.WL_send > 255:
        #     self.WL_send = 255
        output = [chr(83), chr(86), chr(WR_send_ba[0]), chr(WR_send_ba[1]), chr(WR_send_ba[2]), chr(WR_send_ba[3]), chr(WL_send_ba[0]), chr(WL_send_ba[1]), chr(WL_send_ba[2]), chr(WL_send_ba[3]), chr(69)]
        # output = '0x53' + chr(254) + chr(self.WL_send) + chr(L_forward) + chr(self.WR_send) + chr(R_forward)   
        # print output
        self.serial.write(output)
        
if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('base_control', anonymous=True)

        # Constract BaseControl Obj
        rospy.loginfo("HyphaROS MiniBot Base Control ...")
        bc = BaseControl()
        rospy.spin()
    except KeyboardInterrupt:    
        bc.serial.close        
        print("Shutting down")
