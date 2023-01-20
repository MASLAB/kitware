#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FieldDisplay(Node):
    def __init__(self):
        super().__init__('field_display')

        self.subscription = self.create_subscription(Image,
            'webcam_frame',
            self.listener_callback,
            10)
        self.subscription

        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')

        current_frame = self.br.imgmsg_to_cv2(data)

        cv2.imshow("webcam", current_frame)

        cv2.waitKey(1)

if __name__ == '__main__':
    rclpy.init()

    field_display = FieldDisplay()

    rclpy.spin(field_display)

    field_display.destroy_node()

    rclpy.shutdown()