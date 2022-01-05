#!/usr/bin/env python3

import rclpy
from kitware.msg import DriveCmd
from tamproxy import ROS2Sketch
from tamproxy.devices import DigitalOutput, AnalogOutput


class KitBotNode(ROS2Sketch):
    """ROS2 Node that controls the KitBot via the Teensy and tamproxy"""
    # new dual channel DC motor controller
    # https://wiki.dfrobot.com/Dual-Channel_DC_Motor_Driver-12A_SKU:DFR0601
    # TODO: implement TAMProxy Motor object

    # Pin mappings
    # LMOTOR_PINS = (2, 3, 4)  # INA1, INB1, PWM1
    INA1_PIN = 2
    INB1_PIN = 3
    PWM1_PIN = 4
    # RMOTOR_PINS = (5, 6, 7)  # INA2, INB2, PWM2
    INA2_PIN = 5
    INB2_PIN = 6
    PWM2_PIN = 7

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
        # right motor
        self.INA2 = DigitalOutput(self.tamp, self.INA2_PIN)
        self.INB2 = DigitalOutput(self.tamp, self.INB2_PIN)
        self.PWM2 = AnalogOutput(self.tamp, self.PWM2_PIN)

    def speed_to_dir_pwm(self, speed):
        """Converts floating point speed (-1.0 to 1.0) to dir and pwm values"""
        speed = max(min(speed, 1), -1)
        return speed > 0, int(abs(speed * 255))

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
        else # reverse
            self.INA1.write(True)
            self.INB1.write(False)

        if msg.r_speed > 0:
            self.INA2.write(False)
            self.INB2.write(True)
        else
            self.INA2.write(True)
            self.INB2.write(False)

        self.PWM1.write(*self.speed_to_dir_pwm(msg.l_speed)) # left motor
        self.PWM2.write(*self.speed_to_dir_pwm(msg.r_speed)) # right motor

if __name__ == '__main__':
    rclpy.init()

    kb = KitBotNode(rate=100)  # Run at 100Hz (10ms loop)
    kb.run_setup()     # Run tamproxy setup and code in setup() method
    rclpy.spin(kb)

    kb.destroy()       # Shuts down tamproxy
    kb.destroy_node()  # Destroys the ROS node
    rclpy.shutdown()
