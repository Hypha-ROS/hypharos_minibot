#!/usr/bin/python
import rospy
import serial
import string
import math
import sys

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler

class GY85:
    calibrate_complete = True
    calibration_counter = 0
    last_time = 0.0
    dt = 0.0
    yaw_position=0
    yaw_bias = 0.0
    sum_yaw_bias = 0.0
    seq=0

    accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
    degrees2rad = math.pi/180.0
    rad2deg = 180/math.pi

    imuMsg = Imu()
    
    # Orientation covariance estimation:
    imuMsg.orientation_covariance = [
    0.0025 , 0 , 0,
    0, 0.0025, 0,
    0, 0, 0.000025
    ]
    
    # Angular velocity covariance estimation:
    imuMsg.angular_velocity_covariance = [
    0.02, 0 , 0,
    0 , 0.02, 0,
    0 , 0 , 0.02
    ]
    
    # linear acceleration covariance estimation:
    imuMsg.linear_acceleration_covariance = [
    -1 , 0 , 0,
    0 , 0, 0,
    0 , 0 , 0
    ]

    imuMsg.linear_acceleration.x = 0
    imuMsg.linear_acceleration.y = 0
    imuMsg.linear_acceleration.z = 0
    imuMsg.angular_velocity.x = 0
    imuMsg.angular_velocity.y = 0

    
    def __init__(self,imu_link, imu_topic):
        self.imu_link = imu_link
        self.pub = rospy.Publisher(imu_topic, Imu, queue_size=1)

    def pubIMUmsg(self, yaw_v):
        dt = rospy.Time.now().to_sec() - self.last_time
        self.last_time = rospy.Time.now().to_sec()
        yaw_v = yaw_v - self.yaw_bias
        self.imuMsg.angular_velocity.z = float(yaw_v * self.degrees2rad)
        self.yaw_position = yaw_v * dt + self.yaw_position
        print 'yaw_position= ', self.yaw_position

        q = quaternion_from_euler(0,0,self.yaw_position * self.degrees2rad)
        self.imuMsg.orientation.x = q[0]
        self.imuMsg.orientation.y = q[1]
        self.imuMsg.orientation.z = q[2]
        self.imuMsg.orientation.w = q[3]
        self.imuMsg.header.stamp= rospy.Time.now()
        self.imuMsg.header.frame_id = self.imu_link
        self.imuMsg.header.seq = self.seq
        self.seq = self.seq + 1
        self.pub.publish(self.imuMsg)

    def calibrate_IMU(self, yaw):
        self.calibration_counter += 1
        self.sum_yaw_bias = self.sum_yaw_bias + yaw
        self.yaw_bias = self.sum_yaw_bias / self.calibration_counter
        print 'yaw=',str(yaw)

        if self.calibration_counter > 200:
            self.calibrate_complete = True
            rospy.loginfo("calibration complete bias = %s", self.yaw_bias)
            self.last_time = rospy.Time.now().to_sec()

    def get_imuData(self, yaw):
        if not self.calibrate_complete:
            self.calibrate_IMU(yaw)
        if self.calibrate_complete:
            self.pubIMUmsg(yaw)