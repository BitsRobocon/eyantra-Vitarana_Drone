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
        self.cmd_drone.rcThrottle = 1500
        self.cmd_drone.rcRoll = 1500
        self.cmd_drone.rcPitch = 1500
        self.cmd_drone.rcYaw = 1500

        self.max_values =[2000, 2000, 2000, 2000]
        self.min_values =[1450, 1450, 1450, 1450]

        self.final_setpoint = [19.0000451704 , 72.0, 3] # latitude, longitude, altitude
        self.current_point = [19.0, 72.0, 0.31] # given startpoint

        self.setpoint_error = [0, 0, 0] # roll, pitch, throttle
        
        self.error = [0, 0, 0] # roll, pitch, throttle
        self.sum_error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        
        self.lattitude = 0
        self.longitude = 0
        self.altitude = 0 

        self.Kp = [400000, 40, 50]  # latitude, longitude, altitude
        self.Ki = [0, 0, 0.32]
        self.Kd = [500000, 50, 80]

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
        self.Kp[2] = yaw.Kp * 100.  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = yaw.Ki * 0.0001
        self.Kd[2] = yaw.Kd * 100.

    def altitude_control(self):
        self.error[2] = self.final_setpoint[2] - self.current_point[2]
        self.sum_error[2] = self.sum_error[2] + self.error[2]
        self.setpoint_error[2] = (self.error[2]*self.Kp[2]) + (self.Ki[2] * self.sum_error[2]) + (self.Kd[2] * (self.error[2] - self.prev_error[2])/self.sample_time)
        self.prev_error[2] = self.error[2]
        self.cmd_drone.rcThrottle = 1500 + self.setpoint_error[2]
        self.cmd_drone.rcThrottle = max(min(self.max_values[2], self.cmd_drone.rcThrottle), self.min_values[2])
        self.cmd_pub.publish(self.cmd_drone)
        self.error_roll_pub.publish(self.error[0])
        self.error_pitch_pub.publish(self.error[1])
        self.error_yaw_pub.publish(self.error[2])

    def coordinate_control(self):
        self.error[0] = self.final_setpoint[0] - self.current_point[0]
        self.sum_error[0] = self.sum_error[0] + self.error[0]
        self.setpoint_error[0] = (self.error[0]*self.Kp[0]) + (self.Ki[0] * self.sum_error[0]) + (self.Kd[0] * (self.error[0] - self.prev_error[0])/self.sample_time)
        self.prev_error[0] = self.error[0]
        self.cmd_drone.rcRoll = 1500 + self.setpoint_error[0]
        self.cmd_drone.rcRoll = max(min(self.max_values[0], self.cmd_drone.rcRoll), self.min_values[0])
        self.cmd_pub.publish(self.cmd_drone)

        self.error[1] = self.final_setpoint[1] - self.current_point[1]
        self.sum_error[1] += self.error[1]
        self.setpoint_error[1] = (self.error[1]*self.Kp[1]) + (self.Ki[1] * self.sum_error[1]) + (self.Kd[1] * (self.error[1] - self.prev_error[1])/self.sample_time)
        self.prev_error[1] = self.error[1]
        self.cmd_drone.rcPitch = 1500 + self.setpoint_error[1]
        self.cmd_drone.rcPitch = max(min(self.max_values[1], self.cmd_drone.rcPitch), self.min_values[1])
        self.cmd_pub.publish(self.cmd_drone)
        self.error_roll_pub.publish(self.error[0])
        self.error_pitch_pub.publish(self.error[1])
        self.error_yaw_pub.publish(self.error[2])

        

    def control_pid(self):
        while abs(self.final_setpoint[2] - self.current_point[2]) > 0.2 :
            self.altitude_control()

        t_end = time.time() + 30 
        while time.time() < t_end:
            self.coordinate_control()
            
        self.final_setpoint[2] == 0.31
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
