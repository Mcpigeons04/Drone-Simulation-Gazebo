#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from swift_msgs.msg import *
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
from pid_tune.msg import PidTune
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from luminosity_drone.msg import Biolocation

class Camera:

    def __init__(self):
        # rospy.init_node('drone_control')
        rospy.init_node('camera_read')

        self.drone_position = [0.0,0.0,0.0]
        
        self.waypoint_navigation = [
            [0,0,23],
            [0,2.5,23],
            [0,5,23],
            [2.5,5,23],
            [5,5,23],
            [5,2.5,23],
            [5,0,23],
            [0,-2.5,23],
            [5,-5,23],
            [2.5,-5,23],
            [0,-5,23],
            [-2.5,-5,23],
            [-5,-5,23],
            [-5,-2.5,23],
            [-5,0,23],
            [-5,2.5,23],
            [-5,5,23],
            [-5,6.5,23],
            [-5,8,23],
            [-2.5,8.23],
            [0,8,23],
            [2.5,8,23],
            [5,8,23],
            [6.5,8,23],
            [8,8,23],
            [8,6.5,23],
            [8,5,23],
            [8,2.5,23],
            [8,0,23],
            [8,-2.5,23],
            [8,-5,23],
            [8,-6.5,23],
            [8,-8,23],
            [6.5,-8,23],
            [5,-8,23],
            [2.5,-8,23],
            [0,-8,23],
            [-2.5,-8,23],
            [-5,-8,23],
            [-6.5,-8,23],
            [-8,-8,23],
            [-8,-6.5,23],
            [-8,-5,23],
            [-8,-2.5,23],
            [-8,0,23],
            [-8,2.5,23],
            [-8,5,23],
            [-8,6.5,23],
            [-8,8,23],
            [-8,8,23],
            [-8,8,23]
        ]

        self.current_waypoint_navigation_index = 0
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

        self.sample_time = 0.03333

        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
        self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
        self.biolocation_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=1)



        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber("/swift/camera_rgb/image_raw", Image, self.callback)
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

    
    def callback(self,data):
       bridge = CvBridge()
       cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
       self.detect_leds(cv_image)


    def detect_leds(self,cv_image):
            gray=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
            gray=cv2.GaussianBlur(gray,(3,3),0)

            cv2.imshow("Image",gray)
            # threshold the image to reveal light regions in the blurred image
            threshold,thresh=cv2.threshold(gray,100,255,cv2.THRESH_BINARY)

            # perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
            erosed=cv2.erode(thresh,(5,5),iterations=1)
            # perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
            labels = measure.label(erosed, connectivity=2, background=0)
            mask = np.zeros(erosed.shape, dtype="uint8")
            min_size_threshold=10
            min_area_threshold=50
            # loop over the unique components
            for label in np.unique(labels):
                # if this is the background label, ignore it
                if label == 0:
                    continue
                # otherwise, construct the label mask and count the number of pixels 
                labelMask = np.zeros(erosed.shape, dtype="uint8")
                labelMask[labels == label] = 255
                numPixels = cv2.countNonZero(labelMask)
                # if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
                if numPixels > min_size_threshold:
                    mask = cv2.add(mask, labelMask)
            # find the contours in the mask, then sort them from left to right
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            contours = sorted(contours, key=lambda x: cv2.boundingRect(x)[0])


            # Initialize lists to store centroid coordinates and area
            centroid_coordinates = []
            # Append centroid coordinates and area to the respective lists
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = M["m10"] / M["m00"]
                cY = M["m01"] / M["m00"]
            else:
                cX, cY = 0, 0

            poses=centroid_coordinates.append((cX, cY))
            self.detect_and_land(poses)
        
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
        self.error[0] = self.setpoint[0] - self.drone_position[0]

        P0 = self.Kp[0] * self.error[0]
        I0 = self.Ki[0] * self.error_sum[0]
        self.error_sum[0] += self.error[0]
        D0 = self.Kd[0] * (self.error[0] - self.prev_error[0])
        control_output_0 = P0 + I0 + D0
        self.cmd.rcRoll = 1500 + int(control_output_0)

        if self.cmd.rcRoll > 2000:
            self.cmd.rcRoll = 2000
        if self.cmd.rcRoll < 1000:
            self.cmd.rcRoll = 1000
        self.prev_error[0] = self.error[0]

        self.error[1] = self.drone_position[1] - self.setpoint[1]
        P1 = self.Kp[1] * self.error[1]
        I1 = self.Ki[1] * self.error_sum[1]
        self.error_sum[1] += self.error[1]
        D1 = self.Kd[1] * (self.error[1] - self.prev_error[1])
        control_output_1 = P1 + I1 + D1
        self.cmd.rcPitch = 1500 + int(control_output_1)

        if self.cmd.rcPitch > 2000:
            self.cmd.rcPitch = 2000
        if self.cmd.rcPitch < 1000:
            self.cmd.rcPitch = 1000
        self.prev_error[1] = self.error[1]

        self.error[2] = self.drone_position[2] - self.setpoint[2]
        P2 = self.Kp[2] * self.error[2]
        I2 = self.Ki[2] * self.error_sum[2]
        self.error_sum[2] += self.error[2]
        D2 = self.Kd[2] * (self.error[2] - self.prev_error[2])
        control_output_2 = P2 + I2 + D2
        self.cmd.rcThrottle = 1585 + int(control_output_2)

        if self.cmd.rcThrottle > 2000:
            self.cmd.rcThrottle = 2000
        if self.cmd.rcThrottle < 1000:
            self.cmd.rcThrottle = 1000
        self.prev_error[2] = self.error[2]
        if (
            abs(self.error[0]) < 0.1
            and abs(self.error[1]) < 0.1
            and abs(self.error[2]) < 0.1
        ):
                self.current_waypoint_navigation_index += 1
                if self.current_waypoint_navigation_index >= len(self.waypoint_navigation):
                    self.stabilize_at_end()
                   
                else:
                    self.setpoint = self.waypoint_navigation[self.current_waypoint_navigation_index]

        self.roll_error_pub.publish(self.error[0])
        self.pitch_error_pub.publish(self.error[1])
        self.alt_error_pub.publish(self.error[2])
        self.command_pub.publish(self.cmd)
        

    def detect_and_land(self, poses):
        # Extract LED positions from whycon poses
        led_positions = [(pose.position.x, pose.position.y) for pose in poses]

        # Calculate the centroid of LED positions
        centroid_x = sum(x for x, y in led_positions) / len(led_positions)
        centroid_y = sum(y for x, y in led_positions) / len(led_positions)

        # Calculate the number of LEDs
        num_leds = len(poses)

        # Publish the information to the ROS topic
        self.publish_biolocation(num_leds, centroid_x, centroid_y)

        # Land on the calculated centroid
        self.land_on_centroid(centroid_x, centroid_y)

    

    def land_on_centroid(self, centroid_x, centroid_y):
        # Set the setpoint to the centroid coordinates
        self.setpoint = [centroid_x, centroid_y, 0.0]

        # Land the drone
        self.stabilize_at_end()

    def publish_biolocation(self, num_leds, centroid_x, centroid_y):
        # Create a Biolocation message
        biolocation_msg = Biolocation()
        biolocation_msg.organism_type = self.get_organism_type(num_leds)
        biolocation_msg.whycon_x = centroid_x
        biolocation_msg.whycon_y = centroid_y
        biolocation_msg.whycon_z = 0.0  # Assuming a constant height for simplicity

        # Publish the message
        self.biolocation_pub.publish(biolocation_msg)

    def stabilize_at_end(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.command_pub.publish(self.cmd)
            rate.sleep()

if __name__ == '__main__':
    swift_drone = Camera()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        swift_drone.pid()
        r.sleep()