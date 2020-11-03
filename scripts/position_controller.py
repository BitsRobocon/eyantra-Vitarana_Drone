#!/usr/bin/env python

'''
This python file runs a ROS-node of name position_controller which controls the position (latitude and longitude) of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /edrone/drone_command   /edrone/gps

'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
import math
import rospy
import time


class EdroneController():
    def __init__(self):
        rospy.init_node('position_controller')

        self.sample_time = 0.060

        self.cmd_drone = edrone_cmd()

        self.max_values =[2000, 2000, 2000, 2000]
        self.min_values =[1000, 1000, 1000, 1000]

        self.final_setpoint = [19.0000451704 , 72.0, 3] # latitude, longitude, altitude
        self.current_point = [19.0, 72.0, 0.31] # given startpoint

        self.setpoint_error = [0, 0, 0, 0] # roll, pitch, yaw, throttle
        
        self.error = [0, 0, 0, 0] # roll, pitch, yaw, throttle
        self.sum_error = [0, 0, 0, 0]
        self.prev_error = [0, 0, 0, 0]
        
        self.lattitude = 0
        self.longitude = 0
        self.altitude = 0 
        # self.pid_values_roll = [4.02, 0, 2.4] # Kp, Ki, Kd
        # self.pid_values_pitch = [1.02, 0, 1.2] # Kp, Ki, Kd
        # self.pid_values_yaw = [82.56, 0, 0] # Kp, Ki, Kd
        #self.pid_values_throttle = [0, 0, 0] # Kp, Ki, Kd
        self.Kp = [4.02, 1.02, 82.56, 0]  # roll, pitch, yaw, throttle
        self.Ki = [0, 0, 0, 0]
        self.Kd = [2.4, 1.2, 0, 0]

        # Publisher
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # Subscriber
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)


    def gps_callback(self, msg):
        self.current_point[0] = msg.current_point[0] # lattitude
        self.current_point[1] = msg.current_point[1] # longitude
        self.current_point[2] = msg.current_point[2] # altitude

    def control_pid(self):
        # calculate errors
        for i in range(4):
            self.error[i] = self.final_setpoint[i] - self.current_point[i]
            self.sum_error[i] += self.error[i]
        
        # update variables
        for i in range(4):
            self.setpoint_error[i] = (self.error[i]*self.Kp[i]) + (self.Ki[i] * self.error_sum[0]) + (self.Kd[i] * (self.error[0] - self.prev_error[0])/self.sample_time)
        for i in range(4):
            self.prev_error[i] = self.error[i]
        # publish values
        self.cmd_drone.rcRoll = 1500 + self.setpoint_error[0]
        self.cmd_drone.rcPitch = 1500 + self.setpoint_error[1]
        self.cmd_drone.rcYaw = 1500 + self.setpoint_error[2]
        self.cmd_drone.rcThrottle = 1500 + self.setpoint_error[3]

        # Limiting the output values
        self.cmd_drone.rcRoll = max(min(self.max_values[0], self.cmd_drone.rcRoll), self.min_values[0])
        self.cmd_drone.rcPitch = max(min(self.max_values[1], self.cmd_drone.rcPitch), self.min_values[1])
        self.cmd_drone.rcYaw = max(min(self.max_values[2], self.cmd_drone.rcYaw), self.min_values[2])
        self.cmd_drone.rcThrottle = max(min(self.max_values[3], self.cmd_drone.rcThrottle), self.min_values[3])
        
        self.cmd_pub.publish(self.cmd_drone)

if __name__ == '__main__':

    e_drone_controller = EdroneController()
    r = rospy.Rate(30)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone_controller.pid()
        try:
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            continue