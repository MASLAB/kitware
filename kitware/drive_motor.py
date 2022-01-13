#!/usr/bin/env python3

import rclpy
from kitware.msg import DriveCmd, EncoderRead
from tamproxy import ROS2Sketch
from tamproxy.devices import DigitalOutput, AnalogOutput, Encoder
import math

class DriveMotorNode(ROS2Sketch):
    """ROS2 Node that controls the KitBot via the Teensy and tamproxy"""
    # new dual channel DC motor controller
    # https://wiki.dfrobot.com/Dual-Channel_DC_Motor_Driver-12A_SKU:DFR0601
    # TODO: implement TAMProxy Motor object

    # Pin mappings
    # LMOTOR_PINS = (2, 3, 4)  # INA1, INB1, PWM1
    INA1_PIN = 2
    INB1_PIN = 3
    PWM1_PIN = 4

    # Encoder Pin Mappings
    # JOHNZ: change to actual pins before running
    ENC_PINS = 5, 6
    ENC_VCC = 7
    ENC_GND = 8

    # Encoder signals
    current_angle_rad = 0.0

    def setup(self):
        """
        One-time method that sets up the robot, like in Arduino
        Code is run when run_setup() method is called
        """
        # Create a subscriber to listen for drive motor commands
        self.drive_sub = self.create_subscription(
            DriveCmd,
            'drive_cmd',
            self.drive_callback,
            10)
        self.drive_sub  # prevent unused variable warning

        # Create a publisher to publish encoder data
        self.encoder_pub = self.create_publisher(
            EncoderRead, # message interface
            'encoder_read', # topic name
            10) # queue length

        # create pin objects
        # left motor
        self.INA1 = DigitalOutput(self.tamp, self.INA1_PIN)
        self.INB1 = DigitalOutput(self.tamp, self.INB1_PIN)
        self.PWM1 = AnalogOutput(self.tamp, self.PWM1_PIN)

        # encoder: takes channel A and B gives you count [int32]
        self.encoder_left = Encoder(self.tamp, *self.ENC_PINS, continuous=True)
        # trick to get encoder pins in line for 3.3 V and Gnd
        self.encoder_left_power = DigitalOutput(self.tamp, self.ENC_VCC)
        self.encoder_left_ground = DigitalOutput(self.tamp, self.ENC_GND)
        self.encoder_left_power.write(True) # 3.3 V
        self.encoder_left_ground.write(False) # Gnd

    def speed_to_dir_pwm(self, speed):
        """Converts floating point speed (-1.0 to 1.0) to dir and pwm values"""
        speed = max(min(speed, 1), -1)
        return int(abs(speed * 255))

    def count_to_rad(self, count):
        """ Converts encoder counts to motor drive shaft angle in radians"""
        gear_ratio = 50
        counts_per_revolution = 64
        angle_rad = 2*math.pi*count/(gear_ratio*counts_per_revolution)
        return angle_rad

    def stop_drive_motors(self):
        """Turn off motors when exiting ROS"""
        # JOHNZ: Add other actuators and motors
        self.PWM1.write(self.speed_to_dir_pwm(0.0))
        self.INA1.write(False)
        self.INB1.write(False)
        self.get_logger().info('Halt! Stopping the motors!')

    def estimate_angular_velocity(self, current_angle_rad):
        """Estimates the angular velocity given the current angle"""
        # JOHNZ: Implement!
        return 0.0

    def encoder_callback(self):
        """
        Get the current angle from the encoder
        Estimate the motor angular velocity
        Publish data to the encoder_read topic
        """
        self.current_angle_rad = self.count_to_rad(self.encoder_left.val)

        # populate message
        encoder_msg = EncoderRead()
        encoder_msg.left_encoder_count = int(self.encoder.val)
        encoder_msg.left_measured_angle_rad = self.current_angle_rad
        encoder_msg.left_estimated_angular_velocity = self.estimate_angular_velocity(self.current_angle_rad)

        # publish message
        self.encoder_pub.publish(encoder_msg)

    def drive_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        # logic for direction
        # INAx: L INBx: L   Brake
        # INAx: L INBx: H   Rotate Forward
        # INAx: L INBx: H   Rotate Reverse
        # INAx: H INBx: H   Free Wheel

        if msg.l_speed > 0: # forward
            self.INA1.write(False)
            self.INB1.write(True)
        else: # reverse
            self.INA1.write(True)
            self.INB1.write(False)

        self.PWM1.write(self.speed_to_dir_pwm(msg.l_speed)) # left motor

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
