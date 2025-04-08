#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import Jetson.GPIO as gpio
import time
import board
import adafruit_mcp4728 as mcp
from geometry_msgs.msg import Twist
import math
gpio.setwarnings(True)

class MotorControllerNode(Node):
    _MAX_BIT = 4095
    _MIN_BIT = 0
    def __init__(self):
        super().__init__('motor_controller_node')

        gpio.setmode(gpio.TEGRA_SOC)


        self.direction_left_motor_pin = "GP49_SPI1_MOSI"#changed
        self.direction_right_motor_pin = "GP48_SPI1_MISO"#changed

        self.start_motor_pin = "GP122"  # 12#changed
        self.run_motor_pin = "GP72_UART1_RTS_N"  # 11 #changed    
        self.alarm_reset_pin = "GP36_SPI3_CLK"


        self.linear_speed = 0
        self.angular_speed = 0
        self.i2c=board.I2C()
        self.dac=mcp.MCP4728(self.i2c)
        # Set up GPIO pins
        gpio.setup(self.start_motor_pin, gpio.OUT)
        gpio.setup(self.run_motor_pin, gpio.OUT)
        gpio.setup(self.alarm_reset_pin, gpio.OUT)
        gpio.setup(self.direction_left_motor_pin, gpio.OUT)
        gpio.setup(self.direction_right_motor_pin, gpio.OUT)
        # Set up PWM for motor speed control
        self.dac.channel_a.raw_value = int(0)
        self.dac.channel_b.raw_value = int(0)
        # Engaging Break and Stop pins before run
        self.disengage_motor()
        # Subscribe to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Wheelbase (distance between wheels in meters)
        self.wheelbase = 0.42  # Adjust this value based on your robot
        self.wheel_diameter=0.15
        self.get_logger().info("Motor controller node started and listening to /cmd_vel")
    def cmd_vel_callback(self, msg):
        """
        Callback function for the /cmd_vel topic.
        Converts linear and angular velocities into wheel velocities and sets motor speeds.
        """
        # Extract linear and angular velocities from the Twist message
        v = msg.linear.x  # Linear velocity (m/s)
        omega = msg.angular.z  # Angular velocity (rad/s)
        if omega>0.2:
            omega=0.2
        if v==0:
            print("inp")
            self.inplace_rotation(omega)
            print(omega)
        else:
            # Calculate wheel velocities using inverse kinematics
            v_L, v_R,s_l,s_r = self.calculate_wheel_velocities(v, omega)
        
            # Set motor speeds
            self.set_motor_speed_l(s_l)
            self.set_motor_speed_r(s_r)
    
            self.get_logger().info(f"Left wheel velocity: {v_L:.2f} m/s, Right wheel velocity: {v_R:.2f} m/s")
    def inplace_rotation(self,omega):
        self.engage_motor()
        speed_w=(60 * omega) / (math.pi * self.wheel_diameter)
        if speed_w <0:
            gpio.output(self.direction_left_motor_pin, False)
            gpio.output(self.direction_right_motor_pin, False)
        else:
            gpio.output(self.direction_left_motor_pin, True)
            gpio.output(self.direction_right_motor_pin, True)
        dac_value = (abs(speed_w) - 6.1463) / 0.0332
        if dac_value <= self._MIN_BIT:
            dac_value = 0
        elif dac_value >= self._MAX_BIT:
            dac_value = 4095
        else:
            pass
        self.dac.channel_a.raw_value = int(dac_value)
        self.dac.channel_b.raw_value = int(dac_value)
        print("speed_w:",speed_w)        
        
    def calculate_wheel_velocities(self, v, omega):
        """
        Calculate the left and right wheel velocities using inverse kinematics.
        
        Args:
            v (float): Desired linear velocity (m/s).
            omega (float): Desired angular velocity (rad/s).
        
        Returns:
            v_L (float): Left wheel velocity (m/s).
            v_R (float): Right wheel velocity (m/s).
        """
        v_L = v - (self.wheelbase * omega) / 2
        v_R = v + (self.wheelbase * omega) / 2
        speed_v_L = (60 * v_L) / (math.pi * self.wheel_diameter)
        speed_v_R = (60 * v_R) / (math.pi * self.wheel_diameter)
        return v_L, v_R,speed_v_L,speed_v_R
    def set_motor_speed_l(self,speed):
        self.engage_motor()
        if speed < 0:
            gpio.output(self.direction_left_motor_pin, True)
        
        else:
            gpio.output(self.direction_left_motor_pin, False)
        dac_value = (abs(speed) - 6.1463) / 0.0332
        if dac_value <= self._MIN_BIT:
            dac_value = 0
        elif dac_value >= self._MAX_BIT:
            dac_value = 4095
        else:
            pass
        self.dac.channel_a.raw_value = int(dac_value)
        print("speed_left:",speed)
    def set_motor_speed_r(self,speed):
        self.engage_motor()
        if speed < 0:
            gpio.output(self.direction_right_motor_pin, False)
        else:
            gpio.output(self.direction_right_motor_pin, True)
        dac_value = (abs(speed) - 6.1463) / 0.0332
        if dac_value <= self._MIN_BIT:
            dac_value = 0
        elif dac_value >= self._MAX_BIT:
            dac_value = 4095
        else:
            pass
        self.dac.channel_b.raw_value = int(dac_value)
        print("speed_right:",speed)


    def engage_motor(self):
        gpio.output(self.run_motor_pin, True)
        gpio.output(self.start_motor_pin, True)


    def disengage_motor(self):
        self.dac.channel_a.raw_value = int(0)
        self.dac.channel_b.raw_value = int(0)
        gpio.output(self.run_motor_pin, False)
        gpio.output(self.start_motor_pin, False)
    def destroy(self):
        self.disengage_motor()
        gpio.cleanup()
        self.get_logger().info("Motor controller node stopped and GPIO cleaned up.")
def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    
    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        pass
    
    # Clean up
    motor_controller_node.destroy()
    motor_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
