#!/usr/bin/env python3

import rclpy
from kitware.msg import EncoderRead
from tamproxy import ROS2Sketch
from tamproxy.devices import DigitalOutput, Encoder
import math

class EncoderNode(ROS2Sketch):
    """ROS2 Node that controls the KitBot via the Teensy and tamproxy"""

    # Encoder pin mappings
    ENC_PINS = 7, 6
    ENC_VCC = 8
    ENC_GND = 9

    def setup(self):
        """
        One-time method that sets up the robot, like in Arduino
        Code is run when run_setup() method is called
        """
        # Create a publisher to publish encoder data
        self.encoder_publisher = self.create_publisher(
            EncoderRead,
            'encoder_read',
            10)

        # create encoder pin objects
        self.encoder = Encoder(self.tamp, *self.ENC_PINS, continuous=True)
        # trick to get 3.3 V and Gnd to have header in a single line
        self.encoder_power = DigitalOutput(self.tamp, self.ENC_VCC)
        self.encoder_ground = DigitalOutput(self.tamp, self.ENC_GND)
        self.encoder_power.write(True)
        self.encoder_ground.write(False)

        # Update encoder at same rate as loop
        RATE = 10 # [Hz]
        self.encoder_publisher_timer = self.create_timer(1.0 / RATE, self.encoder_callback)

    def encoder_callback(self):
        encoder_msg = EncoderRead()
        encoder_msg.encoder_value = int(self.encoder.val)
        encoder_msg.measured_angle_radians = self.convert_to_radians(self.encoder.val)

        #self.get_logger().info('Encoder Count: {}'.format(self.encoder.val))
        self.encoder_publisher.publish(encoder_msg)

    def estimate_velocity(self, position):
        pass

    def convert_to_radians(self, counts):
        CPR = 64 # counts per revolution
        N = 50 # gear ratio 50 rotations of motor shaft = 1 output
        angle_radians = 2*math.pi*counts/CPR*1/N
        return float(angle_radians)


if __name__ == '__main__':
    rclpy.init()
    # Create an instance of KeyboardDriverNode
    encoder_node = EncoderNode(rate=10)
    encoder_node.run_setup()
    # Continue to run the node until a stop command is given
    rclpy.spin(encoder_node)
    # Destroy the node!
    encoder_node.destroy_node()
    rclpy.shutdown()
