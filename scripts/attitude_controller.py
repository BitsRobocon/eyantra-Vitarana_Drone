#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
import math
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500.0, 1500.0, 1500.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]
        self.throttle = 0

        # Declaring pwm_cmd of message type prop_speed and initializing values
        # Hint: To see the message structure of prop_speed type the following command in the terminal
        # rosmsg show vitarana_drone/prop_speed

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [4.02, 1.02, 82.56]
        self.Ki = [0, 0, 0]
        self.Kd = [2.4, 1.2, 0]

        # -----------------------Add other required variables for pid here ----------------------------------------------
        #

        self.prev_values = [0,0,0]
        self.error = [0,0,0]
        self.error_sum = [0,0,0]
        self.min_values = [0,0,0,0]
        self.max_values = [1024,1024,1024,1024]

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.error_roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.error_pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.error_yaw_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
        self.zero_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher('z_error', Float32, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)

        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        # ------------------------------------------------------------------------------------------------------------

    # Imu callback function
    # The function gets executed each time when imu publishes /edrone/imu/data


    # Note: The imu publishes various kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
    # but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
    # so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
    # We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
    # Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
    # rosmsg show sensor_msgs/Imu

    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.throttle = msg.rcThrottle

        # ---------------------------------------------------------------------------------------------------------------


    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------

    # ----------------------------------------------------------------------------------------------------------------------

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp
        self.Ki[0] = roll.Ki
        self.Kd[0] = roll.Kd

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp
        self.Ki[1] = pitch.Ki
        self.Kd[1] = pitch.Kd

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp
        self.Ki[2] = yaw.Ki
        self.Kd[2] = yaw.Kd

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        #   1. Convert the quaternion format of orientation to euler angles
        #   2. Convert the setpoint that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
        #   3. Compute error in each axis. eg: error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0], where error[0] corresponds to error in roll...
        #   4. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        #   5. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        #   6. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #   7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        #   8. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
        #                                                                                                                                      self.pwm_cmd.prop1 = self.max_values[1]
        #   8. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #   9. Add error_sum to use for integral component

        # Converting quaternion to euler angles
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -100 degree to 100 degree for axes
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.2 - 300
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.2 - 300
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.2 - 300

        # Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itself
        self.throttle = (self.throttle - 1000) * 1.024

        # Calculating the error
        for i in range(3):
            self.error[i] = self.setpoint_euler[i] - (self.drone_orientation_euler[i]*(180/math.pi))
            self.error_sum[i] += self.error[i]

        # Calculating pid values
        self.out_roll = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.error_sum[0]) + ((self.Kd[0] * (self.error[0] - self.prev_values[0]))/self.sample_time)
        self.out_pitch = (self.Kp[1] * self.error[1]) + (self.Ki[1] * self.error_sum[1]) + ((self.Kd[1] * (self.error[1] - self.prev_values[1]))/self.sample_time)
        self.out_yaw = (self.Kp[2] * self.error[2]) + (self.Ki[2] * self.error_sum[2]) + ((self.Kd[2] * (self.error[2] - self.prev_values[2]))/self.sample_time)

        # Changing the previous sum value
        self.prev_values[0] = self.error[0]
        self.prev_values[1] = self.error[1]
        self.prev_values[2] = self.error[2]

        # Giving pwm values
        self.pwm_cmd.prop1 = self.throttle - self.out_roll + self.out_pitch - self.out_yaw
        self.pwm_cmd.prop2 = self.throttle - self.out_roll - self.out_pitch + self.out_yaw
        self.pwm_cmd.prop3 = self.throttle + self.out_roll - self.out_pitch - self.out_yaw
        self.pwm_cmd.prop4 = self.throttle + self.out_roll + self.out_pitch + self.out_yaw

        # Limiting the output values
        self.pwm_cmd.prop1 = max(min(self.max_values[0], self.pwm_cmd.prop1), self.min_values[0])
        self.pwm_cmd.prop2 = max(min(self.max_values[1], self.pwm_cmd.prop2), self.min_values[1])
        self.pwm_cmd.prop3 = max(min(self.max_values[2], self.pwm_cmd.prop3), self.min_values[2])
        self.pwm_cmd.prop4 = max(min(self.max_values[3], self.pwm_cmd.prop4), self.min_values[3])

        #
        #
        #
        # ------------------------------------------------------------------------------------------------------------------------

        self.pwm_pub.publish(self.pwm_cmd)
        # self.error_roll_pub.publish(self.error[0])
        # self.error_pitch_pub.publish(self.error[1])
        # self.error_yaw_pub.publish(self.error[2])
        self.zero_pub.publish(0.0)
        #self.z_error_pub.publish(self.error_throttle)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(30)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        try:
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            continue
