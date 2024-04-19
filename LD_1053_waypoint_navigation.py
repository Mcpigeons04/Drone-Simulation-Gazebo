#!/usr/bin/env python3

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy

class SwiftDrone:
    """Swift drone controller class"""

    def __init__(self):
        rospy.init_node('drone_control')  # initializing ROS node with name drone_control

        self.drone_position = [0.0, 0.0, 0.0]

        self.waypoint_navigation = [
            [0, 0, 23],
            [2, 0, 23],
            [2, 2, 23],
            [2, 2, 25],
            [-5, 2, 25],
            [-5, -3, 25],
            [-5, -3, 21],
            [7, -3, 21],
            [7, 0, 21],
            [0, 0, 19]
        ]

        self.current_waypoint_navigation_index = 0
        self.waypoint_reached = False

        self.setpoint = self.waypoint_navigation[self.current_waypoint_navigation_index]

        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        self.Kp = [22.26, 29.7, 33.9]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [132.6, 159, 318]

        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.prev_values = [0.0, 0.0, 0.0]
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        self.sample_time = 0.033
        self.hover_time = 5  # Time to hover at each waypoint

        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
        self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)

        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)

        self.arm()

        # Initialize setpoint as the first waypoint
        self.setpoint = self.waypoint_navigation[0]

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def arm(self):
        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.Kp * 0.06
        self.Ki[2] = alt.Ki * 0.0008
        self.Kd[2] = alt.Kd * 0.3

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06
        self.Ki[1] = pitch.Ki * 0.0008
        self.Kd[1] = pitch.Kd * 0.3

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06
        self.Ki[0] = roll.Ki * 0.0008
        self.Kd[0] = roll.Kd * 0.3

    def pid(self):
        if not self.waypoint_reached:
            # Calculate errors for roll, pitch, and altitude
            self.error[0] = self.setpoint[0] - self.drone_position[0]
            self.error[1] = self.setpoint[1] - self.drone_position[1]
            self.error[2] = self.setpoint[2] - self.drone_position[2]

            for i in range(3):
                # Calculate PID terms
                P = self.Kp[i] * self.error[i]
                I = self.Ki[i] * self.error_sum[i]
                D = self.Kd[i] * (self.error[i] - self.prev_error[i])

                # Calculate control output
                control_output = P + I + D
                self.prev_values[i] = control_output

                # Apply control output to RC channels with bounds
                self.cmd.rcRoll = 1500 + int(control_output)
                self.cmd.rcRoll = max(self.min_values[i], min(self.max_values[i], self.cmd.rcRoll))

                # Update error and error_sum for the next iteration
                self.prev_error[i] = self.error[i]
                self.error_sum[i] += self.error[i]

            # Check if the drone has reached the current waypoint
            if all(abs(e) < 0.2 for e in self.error):
                self.waypoint_reached = True
                self.hover_start_time = rospy.get_time()

        # Hover at the waypoint for a specified time
        elif rospy.get_time() - self.hover_start_time < self.hover_time:
            self.cmd.rcRoll = 1500  # Keep the drone stable
        else:
            # Move to the next waypoint
            self.waypoint_reached = False
            self.current_waypoint_navigation_index += 1
            if self.current_waypoint_navigation_index < len(self.waypoint_navigation):
                self.setpoint = self.waypoint_navigation[self.current_waypoint_navigation_index]

        # Publish errors and command
        self.roll_error_pub.publish(self.error[0])
        self.pitch_error_pub.publish(self.error[1])
        self.alt_error_pub.publish(self.error[2])
        self.command_pub.publish(self.cmd)

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.pid()
            r.sleep()

if __name__ == '__main__':
    swift_drone = SwiftDrone()
    swift_drone.run()
