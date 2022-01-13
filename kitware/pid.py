#!/usr/bin/env python3

import rclpy
from kitware.msg import PIDCmd, EncoderRead, PIDOutput
from tamproxy import ROS2Sketch
from tamproxy.devices import DigitalOutput, AnalogOutput, Encoder
import math

class PIDNode(ROS2Sketch):
    """ROS2 Node that controls the KitBot via the Teensy and tamproxy"""
    # new dual channel DC motor controller
    # https://wiki.dfrobot.com/Dual-Channel_DC_Motor_Driver-12A_SKU:DFR0601
    # TODO: implement TAMProxy Motor object

    # Motor pin mappings
    INA_PIN = 3
    INB_PIN = 4
    PWM_PIN = 5

    # Encoder pin mappings
    ENC_PINS = 7, 6
    ENC_VCC = 8
    ENC_GND = 9

    # reference signals
    current_position = 0.0
    current_velocity = 0.0
    last_position = 0.0
    last_velocity = 0.0
    
    # PID signals
    proportional_term = 0.0
    integral_term = 0.0
    derivative_term = 0.0
    control_effort = 0.0
    
    def setup(self):
        """
        One-time method that sets up the robot, like in Arduino
        Code is run when run_setup() method is called
        """
        # Create a subscriber to listen for drive motor commands
        self.pid_cmd_sub = self.create_subscription(
            PIDCmd,
            'pid_cmd',
            self.drive_callback,
            10)
        self.pid_cmd_sub  # prevent unused variable warning

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

        # Create a publisher for the PID cmd
        self.pid_cmd_pub = self.create_publisher(
            PIDCmd,
            'pid_cmd',
            10)

        self.pid_output_pub = self.create_publisher(
            PIDOutput,
            'pid_output',
            10)

        # create motor pin objects
        self.INA = DigitalOutput(self.tamp, self.INA_PIN)
        self.INB = DigitalOutput(self.tamp, self.INB_PIN)
        self.PWM = AnalogOutput(self.tamp, self.PWM_PIN)

    
    def speed_to_dir_pwm(self, speed):
        """Converts floating point speed (-1.0 to 1.0) to dir and pwm values"""
        speed = max(min(speed, 1), -1)
        return int(abs(speed * 255))

    def saturate(self, control_effort):
        max_saturation = 1
        min_saturation = -1
        return max(min(control_effort, max_saturation), min_saturation)

    def encoder_callback(self):
        self.current_position = self.convert_to_radians(self.encoder.val)

        encoder_msg = EncoderRead()
        encoder_msg.encoder_value = int(self.encoder.val)
        encoder_msg.measured_angle_radians = self.current_position
        encoder_msg.estimated_angular_velocity = self.estimate_velocity(self.current_position)

        #self.get_logger().info('Encoder Count: {}'.format(self.encoder.val))

        self.encoder_publisher.publish(encoder_msg)

    def brake(self):
        self.PWM.write(self.speed_to_dir_pwm(0.0))
        self.INA.write(False)
        self.INB.write(False)
        self.get_logger().info('Stopping the motors!')

    def drive_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        # logic for direction
        # INAx: L INBx: L   Brake
        # INAx: L INBx: H   Rotate Forward
        # INAx: L INBx: H   Rotate Reverse
        # INAx: H INBx: H   Free Wheel

        # update encoder
        self.encoder_callback()

        position_error = msg.desired_position - self.current_position
        
        # antiwindup
        self.integral_term = self.saturate(self.integral_term + msg.ki*position_error)

        # derivative requires care
        self.derivative_term = (1-msg.alpha)*self.derivative_term + msg.alpha*msg.kd*(self.current_position - self.last_position)
        self.last_position = self.current_position

        self.control_effort = msg.kp*(self.proportional_term + self.integral_term + self.derivative_term)
        self.control_effort = self.saturate(self.control_effort)

        #self.get_logger().info('Speed: {}'.format(control_effort))
        pid_output_msg = PIDOutput()
        pid_output_msg.control_effort = float(self.control_effort)
        pid_output_msg.integral_term = float(self.integral_term)
        pid_output_msg.derivative_term = float(self.derivative_term)
        self.pid_output_pub.publish(pid_output_msg)

        if self.control_effort > 0: # forward
            self.INA.write(False)
            self.INB.write(True)
        else: # reverse
            self.INA.write(True)
            self.INB.write(False)
            
        self.PWM.write(self.speed_to_dir_pwm(self.control_effort))

    def estimate_velocity(self, position):
        alpha = 1
        sample_rate = 1000
        velocity = (1-alpha)*self.last_velocity + alpha*(position - self.last_position)*sample_rate
        self.last_velocity = velocity
        return velocity

    def convert_to_radians(self, counts):
        CPR = 64 # counts per revolution
        N = 50 # gear ratio 50 rotations of motor shaft = 1 output
        angle_radians = 2*math.pi*counts/CPR*1/N
        return float(angle_radians)

if __name__ == '__main__':
    rclpy.init()

    try:
        PID = PIDNode(rate=1000)  # Run at 100Hz (10ms loop)
        PID.run_setup()     # Run tamproxy setup and code in setup() method
        rclpy.spin(PID)
    except KeyboardInterrupt:
        print('Interrupted')
        PID.brake()
        PID.destroy()       # Shuts down tamproxy
        PID.destroy_node()  # Destroys the ROS node
        rclpy.shutdown()
