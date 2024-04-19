#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subscribes to the following topics:

    PUBLICATIONS            SUBSCRIPTIONS
    /drone_command          /whycon/poses
    /alt_error              /pid_tuning_altitude
    /pitch_error            /pid_tuning_pitch
    /roll_error             /pid_tuning_roll

Rather than using different variables, use lists. eg: self.setpoint = [1,2,3], where index corresponds to x, y, z... rather than defining self.x_setpoint = 1, self.y_setpoint = 2.
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAIN MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class swift():
    """Swift drone controller class"""

    def __init__(self):
        rospy.init_node('drone_control')  # initializing ROS node with name drone_control

        # This corresponds to your current position of the drone. This value must be updated each time in your whycon callback
        # [x, y, z]
        self.drone_position = [0.0, 0.0, 0.0]

        # [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = [2, 2, 20]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # Initial setting of Kp, Ki, and Kd for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in the throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters
        # self.Kp = [18.42, 37.08, 25.44]
        # self.Ki = [0.0, 0.0, 0.0]
        # self.Kd = [116.7, 222.6, 365.7]
        self.Kp = [22.26, 29.7, 33.9]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [132.6, 159, 318]
        # self.Kp = [18.42, 96.48, 77.4]
        # self.Ki = [0.0, 0.0, 0.0]
        # self.Kd = [37.2, 519.3, 471.6]


        # Add other required variables for PID here
        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.prev_values = [0.0, 0.0, 0.0]
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        # This is the sample time in which you need to run PID. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.033 # in seconds

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)

        # Add other ROS Publishers here
        self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)

        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)

        # Add other ROS Subscribers here
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)

        # Arm the drone
        self.arm()

    # Disarming condition of the drone
    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone: Best practice is to disarm and then arm the drone.
    def arm(self):
        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x

        # Set the remaining co-ordinates of the drone from msg
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = alt.Ki * 0.0008
        self.Kd[2] = alt.Kd * 0.3

    # Define callback function like altitude_set_pid to tune pitch, roll
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06
        self.Ki[1] = pitch.Ki * 0.0008
        self.Kd[1] = pitch.Kd * 0.3

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06
        self.Ki[0] = roll.Ki * 0.0008
        self.Kd[0] = roll.Kd * 0.3

    # PID controller function
    def pid(self):
    # Compute errors for roll
            self.error[0] = self.setpoint[0] - self.drone_position[0]

        # Calculate PID control outputs
            # Proportional term
            P0 = self.Kp[0] * self.error[0]

            # Integral term
            I0 = self.Ki[0] * self.error_sum[0]
            self.error_sum[0]+=self.error[0]

            # Derivative term
            D0 = self.Kd[0] * (self.error[0] - self.prev_error[0])

            # Total control output
            control_output_0 = P0 + I0 + D0

            # Apply control output to respective channels (roll, pitch, throttle)
            
            self.cmd.rcRoll = 1500 + int(control_output_0)

            # Limit the control output

            if self.cmd.rcRoll > 2000:
                self.cmd.rcRoll = 2000
            if self.cmd.rcRoll < 1000:
                self.cmd.rcRoll = 1000

            self.prev_error[0]=self.error[0]
    # Compute errors for pitch
            self.error[1] = self.drone_position[1] - self.setpoint[1]

        # Calculate PID control outputs
            # Proportional term
            P1 = self.Kp[1] * self.error[1]

            # Integral term
            I1 = self.Ki[1] * self.error_sum[1]
            self.error_sum[1]+=self.error[1]

            # Derivative term
            D1 = self.Kd[1] * (self.error[1] - self.prev_error[1])

            # Total control output
            control_output_1 = P1 + I1 + D1

            # Apply control output to respective channels (roll, pitch, throttle)
            
            self.cmd.rcPitch = 1500 + int(control_output_1)

            # Limit the control output

            if self.cmd.rcPitch > 2000:
                self.cmd.rcPitch = 2000
            if self.cmd.rcPitch < 1000:
                self.cmd.rcPitch = 1000

            self.prev_error[1]=self.error[1]
    # Compute errors altitude
            self.error[2] = self.drone_position[2] - self.setpoint[2]

        # Calculate PID control outputs
            # Proportional term
            P2 = self.Kp[2] * self.error[2]

            # Integral term
            I2 = self.Ki[2] * self.error_sum[2]
            self.error_sum[2]+=self.error[2]

            # Derivative term
            D2 = self.Kd[2] * (self.error[2] - self.prev_error[2])

            # Total control output
            control_output_2 = P2 + I2 + D2

            # Apply control output to respective channels (roll, pitch, throttle)
            
            self.cmd.rcThrottle = 1585 + int(control_output_2)

            # Limit the control output

            if self.cmd.rcThrottle > 2000:
                self.cmd.rcThrottle = 2000
            if self.cmd.rcThrottle < 1000:
                self.cmd.rcThrottle = 1000

            self.prev_error[2]=self.error[2]

           
        # Publish error values    
            self.roll_error_pub.publish(self.error[0])
            self.pitch_error_pub.publish(self.error[1])
            self.alt_error_pub.publish(self.error[2])        
            self.command_pub.publish(self.cmd)


if __name__ == '__main__':
    swift_drone = swift()
    r = rospy.Rate(30)  # specify rate in Hz based upon your desired PID sampling time, i.e., if the desired sample time is 33ms, specify rate as 30Hz
    while not rospy.is_shutdown():
        swift_drone.pid()
        r.sleep()