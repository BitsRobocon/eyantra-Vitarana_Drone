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
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math
import rospy
import time


class EdroneController():
    def __init__(self):
        rospy.init_node('position_controller')

        final_setpoint = [19.0000451704 , 72.0, 3] # latitude, longitude, altitude
        current_point = [19.0, 72.0, 0.31] # given startpoint

        setpoint_error = [0, 0, 0]

        pid_values_roll = [0, 0, 0] # Kp, Ki, Kd
        pid_values_pitch = [0, 0, 0] # Kp, Ki, Kd
        pid_values_yaw = [0, 0, 0] # Kp, Ki, Kd
        pid_values_throttle = [0, 0, 0] # Kp, Ki, Kd

        # Publisher
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # Subscriber
        rospy.Subscriber('/edrone/gps', , self.gps_callback)


    def gps_callback(self, msg):
        pass #todo

    def control_pid(self):
        # calculate errors
        # update variables
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
