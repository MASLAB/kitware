#!/usr/bin/env python3

import rclpy
from kitware.msg import DriveCmd
from tamproxy import ROS2Sketch
from tamproxy.devices import DFRMotor

class KitBotNode(ROS2Sketch):
    """ROS2 Node that controls the KitBot via the Teensy and tamproxy"""
    # new dual channel DC motor controller
    # https://wiki.dfrobot.com/Dual-Channel_DC_Motor_Driver-12A_SKU:DFR0601

    # Pin mappings
    # LMOTOR_PINS
    INA1_PIN = 3
    INB1_PIN = 2
    PWM1_PIN = 4
    # RMOTOR_PINS
    INA2_PIN = 6
    INB2_PIN = 5
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

        self.left_motor = DFRMotor(self.tamp, self.INA1_PIN, self.INB1_PIN, self.PWM1_PIN)
        self.right_motor = DFRMotor(self.tamp, self.INA2_PIN, self.INB2_PIN, self.PWM2_PIN)
        
    def speed_to_dir_pwm(self, speed):
        """Converts floating point speed (-1.0 to 1.0) to dir and pwm values"""
        speed = max(min(speed, 1), -1)
        return speed > 0, int(abs(speed * 255))

    def drive_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        self.left_motor.write(*self.speed_to_dir_pwm(msg.l_speed))
        self.right_motor.write(*self.speed_to_dir_pwm(msg.r_speed))

if __name__ == '__main__':
    rclpy.init()

    kb = KitBotNode(rate=100)  # Run at 100Hz (10ms loop)
    kb.run_setup()     # Run tamproxy setup and code in setup() method
    rclpy.spin(kb)

    kb.destroy()       # Shuts down tamproxy
    kb.destroy_node()  # Destroys the ROS node
    rclpy.shutdown()