#!/usr/bin/env python

'''
This python file runs a ROS-node of name position_controller which controls the position (latitude and longitude) of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /edrone/drone_command   /edrone/gps

'''

##  !!!"use roll gui for latitude, pitch gui for longitude and yaw gui for altitude."

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

        self.final_setpoint = [19.0000451704 , 72.0, 0, 3] # latitude, longitude, yaw, altitude
        self.current_point = [19.0, 72.0, 0, 0.31] # given startpoint

        self.error = [0, 0, 0, 0] # roll, pitch, yaw, throttle
        self.sum_error = [0, 0, 0, 0]
        self.prev_error = [0, 0, 0, 0]

        self.setpoint_error = [0, 0, 0, 0]

        self.latitude = 0
        self.longitude = 0
        self.altitude = 0

        self.Kp = [4000000, 50, 0, 172.92]  # latitude, longitude, yaw, altitude
        self.Ki = [0, 0.32, 0, 4.32]
        self.Kd = [5000000, 80, 0, 150.6]

        # Publisher
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.error_roll_pub = rospy.Publisher('/latitude_r_error', Float32, queue_size=1)
        self.error_pitch_pub = rospy.Publisher('/longitude_p_error', Float32, queue_size=1)
        self.error_yaw_pub = rospy.Publisher('/altitude_y_error', Float32, queue_size=1)

        # Subscriber
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.latitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.longitude_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        rospy.Subscriber('/pid_tuning_throttle', PidTune, self.throttle_set_pid)


    def gps_callback(self, msg):
        self.current_point[0] = msg.latitude  # latitude
        self.current_point[1] = msg.longitude # longitude
        self.current_point[3] = msg.altitude  # altitude

    def latitude_set_pid(self, roll):
        self.Kp[0] = roll.Kp # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki
        self.Kd[0] = roll.Kd

    def longitude_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki
        self.Kd[1] = pitch.Kd

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = yaw.Ki
        self.Kd[2] = yaw.Kd

    def throttle_set_pid(self, throttle):
        self.Kp[3] = throttle.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[3] = throttle.Ki
        self.Kd[3] = throttle.Kd

    def control_pid(self):
        # calculate errors
        for i in range(4):
            self.error[i] = self.final_setpoint[i] - self.current_point[i]
            self.sum_error[i] += self.error[i]

        # update variables
        for i in range(4):
            self.setpoint_error[i] = (self.error[i]*self.Kp[i]) + (self.Ki[i] * self.sum_error[i]) + (self.Kd[i] * (self.error[i] - self.prev_error[i])/self.sample_time)
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
        self.error_roll_pub.publish(self.error[0])
        self.error_pitch_pub.publish(self.error[1])
        self.error_yaw_pub.publish(self.error[3])

if __name__ == '__main__':

    e_drone_controller = EdroneController()
    r = rospy.Rate(30)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone_controller.control_pid()
        try:
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            continue
