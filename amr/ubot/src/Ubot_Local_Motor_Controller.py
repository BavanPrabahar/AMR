#!/usr/bin/env python3
import Jetson.GPIO as gpio
import sys
import time
from math import pi
import board
import adafruit_mcp4728 as mcp

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from math import pi

class RobotMover(Node):
    # RASPBERRY PI 4 PIN CONFIGURATION
    # _FWD_REV_L = 17
    # _FWD_REV_R = 27
    # _START_STOP = 16
    # _RUN_BREAK = 26
    # _ALR_RST = 22
    
    # JETSON PIN CONFIGURATION (yet to change the pin configurations)
    _FWD_REV_L = "GP49_SPI1_MOSI"               # 19
    _FWD_REV_R = "GP48_SPI1_MISO"               # 21
    _START_STOP = "GP122"                       # 12
    _RUN_BREAK = "GP72_UART1_RTS_N"             # 11
    _ALR_RST = "GP36_SPI3_CLK"                  # 13
    
    _MAX_BIT = 4095
    _MIN_BIT = 0
    def __init__(self,wheel_dia=0.15,wheel_dist=0.42):
        super().__init__('robot_mover')
        self.wheel_diameter=wheel_dia
        self.wheel_distance=wheel_dist
        self.i2c=board.I2C()
        self.dac=mcp.MCP4728(self.i2c)
        self.maximum_linear_velocity=0.7
        self.maximum_angular_velocity=0.25
        self.prev_velocity=0
        self.current_velocity=0
        self.safe=True
        gpio.setmode(gpio.TEGRA_SOC)
        gpio.setwarnings(True)
        gpio.setup(self._FWD_REV_L, gpio.OUT)
        gpio.setup(self._FWD_REV_R, gpio.OUT)
        gpio.setup(self._START_STOP, gpio.OUT)
        gpio.setup(self._RUN_BREAK, gpio.OUT)
        gpio.setup(self._ALR_RST, gpio.OUT)
        
        self.dac.channel_a.raw_value = int(0)
        self.dac.channel_b.raw_value = int(0)
        
        # MOTOR CONFIGURATION
        gpio.output(self._START_STOP, False)         # OFF STATE
        gpio.output(self._RUN_BREAK, False)          # BREAK STATE
        
        # RESETTING ALARM MANUALLY ONCE
        gpio.output(self._ALR_RST, False)
        time.sleep(0.1)
        gpio.output(self._ALR_RST, True)
        time.sleep(0.1)
        self.create_subscription(Twist,'/localcmd_vel',self.velocity_callback,1)
        self.create_subscription(Bool,'/safety_status',self.safety_callback,1)
        self.timer=self.create_timer(0.05,self.listen)
        
    def __del__(self)->None:
        self.dac.channel_a.raw_value = int(0)
        self.dac.channel_b.raw_value = int(0)
        gpio.cleanup()
    
    def decide_direction(self, desired_vel, linear):
        if linear:
            if desired_vel >= 0:
                gpio.output(self._FWD_REV_L, False)
                gpio.output(self._FWD_REV_R, True)
            else:
                gpio.output(self._FWD_REV_L, True)
                gpio.output(self._FWD_REV_R, False)
        else:
            if desired_vel >= 0:
                gpio.output(self._FWD_REV_L, False)
                gpio.output(self._FWD_REV_R, False)
            else:
                gpio.output(self._FWD_REV_L, True)
                gpio.output(self._FWD_REV_R, True)
        
    def calculate_dac_value(self, desired_vel, linear) -> int:
        # Decide the direction of the wheel rotation using the sign of the velocity received
        self.decide_direction(desired_vel, linear)
        
        # Converting the desired velocity into DAC 12-bit value
        # The corresponding RPM is found for the desired velocity and then using the equation of lead wheel from PID, corresponding DAC value is found
        # Equation of lead wheel (right_wheel): rpm_value = (dac_value * 0.0332) + 6.1463
        # desired_vel = 2 * desired_vel / self.wheel_diameter
        rpm_value = (60 * abs(desired_vel)) / (pi * self.wheel_diameter)
        
        dac_value = (rpm_value - 6.1463) / 0.0332
        
        if dac_value <= self._MIN_BIT:
            dac_value = 0
            rpm_value = 0
            
        elif dac_value >= self._MAX_BIT:
            dac_value = 4095

        else:
            pass
        
      
        print(f"######## The corresponding RPM value: ${rpm_value} and DAC value: ${dac_value} ########")
        return dac_value, rpm_value
        
    def safety_callback(self, data) -> None:
        self.safe = data.data
    
    def velocity_callback(self, data) -> None:
        self.current_velocity = data
      
    def engage_motor(self) -> None:
        gpio.output(self._RUN_BREAK, True)
        gpio.output(self._START_STOP, True)
        # print("Engaged Motor")
        
    def disengage_motor(self) -> None:
        gpio.output(self._RUN_BREAK, False)
        gpio.output(self._START_STOP, False)
      
    def listen(self):
      
         
        if self.current_velocity != self.prev_velocity and self.safe:
            print("Engaged motor")
            self.engage_motor()
            if (self.current_velocity.angular.z != 0):
                dac_value, rpm_value = self.calculate_dac_value(self.current_velocity.angular.z, False)
            else:
                dac_value, rpm_value = self.calculate_dac_value(self.current_velocity.linear.x, True)
            self.dac.channel_a.raw_value = int(dac_value)
            self.dac.channel_b.raw_value = int(dac_value)
            self.prev_velocity = self.current_velocity
         
        elif self.current_velocity != self.prev_velocity and not self.safe:
           
            dac_value, rpm_value = self.calculate_dac_value(0, True)
            self.dac.channel_a.raw_value = int(dac_value)
            self.dac.channel_b.raw_value = int(dac_value)
            self.prev_velocity = self.current_velocity
        else:
            pass

                   
            
                    

        
def main(args=None):
    rclpy.init(args=args)
    '''if len(sys.argv) > 2:
        wheel_diameter = float(sys.argv[1])
        wheel_distance = float(sys.argv[2])'''
    
    node=RobotMover(0.15,0.42)
  
    node.listen()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ =="__main__":
    try:
        main()
    except:
        pass
