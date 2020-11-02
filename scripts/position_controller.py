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

        self.final_setpoint = [19.0000451704 , 72.0, 3] # latitude, longitude, altitude
        self.current_point = [19.0, 72.0, 0.31] # given startpoint

        self.setpoint_error = [0, 0, 0]
        
        self.error = [0, 0, 0]
        self.sum_error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        
        self.lattitude = 0
        self.longitude = 0
        self.altitude = 0 
        self.pid_values_roll = [4.02, 0, 2.4] # Kp, Ki, Kd
        self.pid_values_pitch = [1.02, 0, 1.2] # Kp, Ki, Kd
        self.pid_values_yaw = [82.56, 0, 0] # Kp, Ki, Kd
        self.pid_values_throttle = [0, 0, 0] # Kp, Ki, Kd

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
        for i in range(3):
            self.error[i] = self.final_setpoint[i] - self.current_point[i]
            self.sum_error[i] += self.error[i]
        
        # update variables
        for i in range(3):
            self.setpoint_error[i] = self.error[i]*1
        # publish values


if __name__ == '__main__':

    e_drone_controller = EdroneController()
    r = rospy.Rate(30)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone_controller.pid()
        try:
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            continue
