#!/usr/bin/env python3

from kitware.msg import VisionDriveCmd

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

# Rate to check for frames
RATE = 10

class VisionNode(Node):
    """ROS2 Node for CV"""
    def __init__(self):
        super().__init__('vision_driver')

        # CAMERA_INDEX indicates which webcam to use if we have
        # multiple connected. If you just have one, use '0'.
        CAMERA_INDEX = 0

        # cap is a VideoCapture object we can use to get frames from webcam
        self.cap = cv2.VideoCapture(CAMERA_INDEX)

        self.drive_command_publisher = self.create_publisher(
                VisionDriveCmd,
                'vision_drive_cmd',
                10)
        timer_period = 1.0 / RATE
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Capture a frame from the webcam
        _, frame = self.cap.read()

        # downsize frame
        frame = cv2.resize(frame, (320, 240))

        # Min and max HSV thresholds for green
        GREEN_THRESHOLD = ([70,50,50], [90,255,255])

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # OpenCV needs bounds as numpy arrays
        lower_bound = np.array(GREEN_THRESHOLD[0])
        upper_bound = np.array(GREEN_THRESHOLD[1])

        # Threshold the HSV image to get only green color
        # Mask contains a white on black image, where white pixels
        # represent that a value was within our green threshold.
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Find contours (distinct edges between two colors) in mask using OpenCV builtin
        # This function returns 2 values, but we only care about the first
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If we have contours...
        if len(contours) != 0:
            # Find the biggest countour by area
            c = max(contours, key = cv2.contourArea)

            # Get a bounding rectangle around that contour
            x,y,w,h = cv2.boundingRect(c)

            # Draw the rectangle on our frame
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            
            # Display the message on the console
            self.get_logger().info('Found a Green Ball at x: {} y: {}'.format(x, y))

            vision_drive_cmd_msg = VisionDriveCmd()
            vision_drive_cmd_msg.x = float(x)
            vision_drive_cmd_msg.y = float(y)
            self.drive_command_publisher.publish(vision_drive_cmd_msg)

        # Display that frame (resized to be smaller for convenience)
        cv2.imshow('frame', frame)

        cv2.waitKey(1)

if __name__ == '__main__':
    rclpy.init()
    try:
        vision = VisionNode()  # Run at 100Hz (10ms loop)
        rclpy.spin(vision)
    except KeyboardInterrupt:
        # Destroy the node!
        vision.destroy_node()
        rclpy.shutdown()
