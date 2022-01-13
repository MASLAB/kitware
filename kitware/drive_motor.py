#!/usr/bin/env python3

import rclpy
from kitware.msg import DriveCmd
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

    kb = DriveMotorNode(rate=100)  # Run at 100Hz (10ms loop)
    kb.run_setup()     # Run tamproxy setup and code in setup() method
    rclpy.spin(kb)

    kb.destroy()       # Shuts down tamproxy
    kb.destroy_node()  # Destroys the ROS node
    rclpy.shutdown()
