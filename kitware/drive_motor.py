#!/usr/bin/env python3

import rclpy
from kitware.msg import DrivePIDCmd, EncoderRead
from tamproxy import ROS2Sketch
from tamproxy.devices import DigitalOutput, FeedbackMotor
import math

class DriveMotorNode(ROS2Sketch):
    """ROS2 Node that controls the KitBot via the Teensy and tamproxy"""
    # new dual channel DC motor controller
    # https://wiki.dfrobot.com/Dual-Channel_DC_Motor_Driver-12A_SKU:DFR0601

    # Pin mappings
    # LMOTOR_PINS = (2, 3, 4)  # INA1, INB1, PWM1
    INA1_PIN = 3
    INB1_PIN = 4
    PWM1_PIN = 5

    # Encoder Pin Mappings
    # JOHNZ: change to actual pins before running
    ENC_PINS = 7, 6
    ENC_VCC = 8
    ENC_GND = 9

    def setup(self):
        """
        One-time method that sets up the robot, like in Arduino
        Code is run when run_setup() method is called
        """
        # Create a subscriber to listen for drive motor commands
        self.drive_pid_sub = self.create_subscription(
            DrivePIDCmd,
            'drive_pid_cmd',
            self.drive_callback,
            10)
        self.drive_pid_sub  # prevent unused variable warning

        # Create a publisher to publish encoder data
        self.encoder_pub = self.create_publisher(
            EncoderRead, # message interface
            'encoder_read', # topic name
            10) # queue length

        # Create a publisher to publish Drive PID command data
        self.drive_pid_cmd_pub = self.create_publisher(
            DrivePIDCmd, # message interface
            'drive_pid_cmd', # topic name
            10) # queue length

        # create pin objects
        # left motor
        self.motor = FeedbackMotor(self.tamp, self.INA1_PIN, self.INB1_PIN, self.PWM1_PIN, *self.ENC_PINS)
        # trick to get encoder pins in line for 3.3 V and Gnd
        self.encoder_left_power = DigitalOutput(self.tamp, self.ENC_VCC)
        self.encoder_left_ground = DigitalOutput(self.tamp, self.ENC_GND)
        self.encoder_left_power.write(True) # 3.3 V
        self.encoder_left_ground.write(False) # Gnd

    def convert_to_angle_cmd(self, angle):
        # factor of 4 due to signed 8 bit int
        return math.pi/180*angle/(4*math.pi)*255

    def count_to_rad(self, count):
        """ Converts encoder counts to motor drive shaft angle in radians"""
        gear_ratio = 50
        counts_per_revolution = 64
        angle_rad = 2*math.pi*count/(gear_ratio*counts_per_revolution)
        return angle_rad

    def stop_drive_motors(self):
        """Turn off motors when exiting ROS"""
        self.get_logger().info('Halt! Stopping the motors!')

    def encoder_callback(self, msg):
        """
        Get the current angle from the encoder
        Estimate the motor angular velocity
        Publish data to the encoder_read topic
        """
        # read current angle
        self.current_angle_rad = self.count_to_rad(self.motor.enc_count)
        
        # populate message
        encoder_msg = EncoderRead()
        encoder_msg.left_measured_angle_rad = 180/math.pi*self.current_angle_rad

        # publish message
        self.encoder_pub.publish(encoder_msg)

    def drive_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        #super().get_logger().info('Rate: {}'.format(self.get_clock().now()))

        # update the encoder reading
        self.encoder_callback(msg)
        self.motor.write(self.convert_to_angle_cmd(msg.desired_angle_deg))

if __name__ == '__main__':
    rclpy.init()

    try:
        kb = DriveMotorNode(rate=100)  # Run at 100Hz (10ms loop)
        kb.run_setup()     # Run tamproxy setup and code in setup() method
        rclpy.spin(kb)
    except KeyboardInterrupt:
        kb.stop_drive_motors()
        kb.destroy()       # Shuts down tamproxy
        kb.destroy_node()  # Destroys the ROS node
        rclpy.shutdown()
