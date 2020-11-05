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
        self.Kp = [4000000, 50, 0, 38.4]  # latitude, longitude, yaw, altitude
        self.Ki = [0, 0.32, 0, 0]
        self.Kd = [5000000, 80, 0, 90.6]

        # Publisher
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.error_roll_pub = rospy.Publisher('/latitude_r_error', Float32, queue_size=1)
        self.error_pitch_pub = rospy.Publisher('/longitude_p_error', Float32, queue_size=1)
        self.error_yaw_pub = rospy.Publisher('/altitude_y_error', Float32, queue_size=1)

        # Subscriber
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)


    def gps_callback(self, msg):
        self.current_point[0] = msg.latitude  # latitude
        self.current_point[1] = msg.longitude # longitude
        self.current_point[2] = msg.altitude  # altitude

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3

    def yaw_set_pid(self, yaw):
        self.Kp[3] = yaw.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[3] = yaw.Ki * 0.008
        self.Kd[3] = yaw.Kd * 0.3

    def altitude_control(self):
        self.error[3] = self.final_setpoint[3] - self.current_point[3]
        self.sum_error[3] += self.error[3]
        self.setpoint_error[3] = (self.error[3]*self.Kp[3]) + (self.Ki[3] * self.sum_error[3]) + (self.Kd[3] * (self.error[3] - self.prev_error[3])/self.sample_time)
        self.prev_error[3] = self.error[3]
        self.cmd_drone.rcThrottle = 1500 + self.setpoint_error[3]
        self.cmd_drone.rcThrottle = max(min(self.max_values[3], self.cmd_drone.rcThrottle), self.min_values[3])
        self.cmd_pub.publish(self.cmd_drone.rcThrottle)
        self.error_yaw_pub.publish(self.error[3])

    def coordinate_control(self):
        self.error[0] = self.final_setpoint[0] - self.current_point[0]
        self.sum_error[0] += self.error[0]
        self.setpoint_error[0] = (self.error[0]*self.Kp[0]) + (self.Ki[0] * self.sum_error[0]) + (self.Kd[0] * (self.error[0] - self.prev_error[0])/self.sample_time)
        self.prev_error[0] = self.error[0]
        self.cmd_drone.rcRoll = 1500 + self.setpoint_error[0]
        self.cmd_drone.rcRoll = max(min(self.max_values[0], self.cmd_drone.rcRoll), self.min_values[0])
        self.cmd_pub.publish(self.cmd_drone.rcRoll)

        self.error[1] = self.final_setpoint[1] - self.current_point[1]
        self.sum_error[1] += self.error[1]
        self.setpoint_error[1] = (self.error[1]*self.Kp[1]) + (self.Ki[1] * self.sum_error[1]) + (self.Kd[1] * (self.error[1] - self.prev_error[1])/self.sample_time)
        self.prev_error[1] = self.error[1]
        self.cmd_drone.rcPitch = 1500 + self.setpoint_error[1]
        self.cmd_drone.rcPitch = max(min(self.max_values[1], self.cmd_drone.rcPitch), self.min_values[1])
        self.cmd_pub.publish(self.cmd_drone.rcPitch)

        self.error_roll_pub.publish(self.error[0])
        self.error_pitch_pub.publish(self.error[1])

    def control_pid(self):
        while self.current_point[3] != self.final_setpoint[3]:
            self.altitude_control()

        self.coordinate_control()

        if self.current_point[0] == self.final_setpoint[0] and self.current_point[1] == self.final_setpoint[1]:
            self.final_setpoint[3] == 0.31
            self.altitude_control()
            
        
        

if __name__ == '__main__':

    e_drone_controller = EdroneController()
    r = rospy.Rate(30)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone_controller.control_pid()
        try:
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            continue
