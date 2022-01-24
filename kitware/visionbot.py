#!/usr/bin/env python3

import rclpy
from kitware.msg import VisionDriveCmd
from tamproxy import ROS2Sketch
from tamproxy.devices import DFRMotor

class VisionBotNode(ROS2Sketch):
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
            VisionDriveCmd,
            'vision_drive_cmd',
            self.drive_callback,
            10)
        self.drive_sub  # prevent unused variable warning

        self.left_motor = DFRMotor(self.tamp, self.INA1_PIN, self.INB1_PIN, self.PWM1_PIN)
        self.right_motor = DFRMotor(self.tamp, self.INA2_PIN, self.INB2_PIN, self.PWM2_PIN)
        
    def unicycle_to_diff_drive(self, v, w):
        """Converts unicycle model velocities to differential drive speeds"""
        # v - translation velocity
        # w - rotational velocity
        L = 0.215 # wheel base length [m]
        R = 0.05 # wheel radius [m]
        
        l_speed = (2*v+w*L)/(2*R)
        r_speed = (2*v-w*L)/(2*R)
        
        return l_speed, r_speed

    def speed_to_dir_pwm(self, speed):
        """Converts floating point speed (-1.0 to 1.0) to dir and pwm values"""
        speed = max(min(speed, 1), -1)
        return speed > 0, int(abs(speed * 255))

    def drive_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        # JOHNZ: Consider adding deadband, integral term, saturation, etc.
        Kp = 0.001
        error = 320/2 - msg.x # desired setpoint to center of screen, be careful about the sign
        rotation_velocity = Kp*error

        l_speed, r_speed = self.unicycle_to_diff_drive(0.0, rotation_velocity)

        self.left_motor.write(*self.speed_to_dir_pwm(l_speed))
        self.right_motor.write(*self.speed_to_dir_pwm(r_speed))

    def stop_drive_motors(self):
        self.left_motor.write(*self.speed_to_dir_pwm(0.0))
        self.right_motor.write(*self.speed_to_dir_pwm(0.0))

if __name__ == '__main__':
    rclpy.init()

    try:
        kb = VisionBotNode(rate=100)  # Run at 100Hz (10ms loop)
        kb.run_setup()     # Run tamproxy setup and code in setup() method
        rclpy.spin(kb)
    except KeyboardInterrupt:
        kb.stop_drive_motors()
        kb.destroy()       # Shuts down tamproxy
        kb.destroy_node()  # Destroys the ROS node
        rclpy.shutdown()