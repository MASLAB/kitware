#!/usr/bin/env python3

import rclpy
from kitware.msg import DriveCmd
from tamproxy import ROS2Sketch
from tamproxy.devices import DigitalOutput, AnalogOutput, Servo, DFRMotor

class KitBotNode(ROS2Sketch):
    """ROS2 Node that controls the KitBot via the Teensy and tamproxy"""
    # new dual channel DC motor controller
    # https://wiki.dfrobot.com/Dual-Channel_DC_Motor_Driver-12A_SKU:DFR0601
    # Pin mappings
    INA1_PIN = 2
    INB1_PIN = 4
    PWM1_PIN = 6
    # RMOTOR_PINS = (5, 6, 7)  # INA2, INB2, PWM2
    INA2_PIN = 3
    INB2_PIN = 5
    PWM2_PIN = 7
    # SERVO_PIN
    SERVO_PIN = 9

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
        # servo
        self.servo = Servo(self.tamp, self.SERVO_PIN)
        self.servo.write(0)

    def speed_to_dir_pwm(self, speed):
        """Converts floating point speed (-1.0 to 1.0) to dir and pwm values"""
        speed = max(min(speed, 1), -1)
        return speed > 0, int(abs(speed * 255))

    def drive_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        self.left_motor.write(*self.speed_to_dir_pwm(msg.l_speed))
        self.right_motor.write(*self.speed_to_dir_pwm(msg.r_speed))

        if msg.r_speed > 0:
            self.INA2.write(False)
            self.INB2.write(True)

        else: # reverse
            self.INA2.write(True)
            self.INB2.write(False)

        self.PWM1.write(self.speed_to_dir_pwm(msg.l_speed)) # left motor
        self.PWM2.write(self.speed_to_dir_pwm(-msg.r_speed)) # right motor
        self.servo.write(msg.servo_pos) # servo

if __name__ == '__main__':
    rclpy.init()

    try:
        kb = KitBotNode(rate=100)  # Run at 100Hz (10ms loop)
        kb.run_setup()     # Run tamproxy setup and code in setup() method
        rclpy.spin(kb)
    except KeyboardInterrupt:
        kb.stop_drive_motors()
        kb.destroy()       # Shuts down tamproxy
        kb.destroy_node()  # Destroys the ROS node
        rclpy.shutdown()