#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Float64
import math
import numpy as np
import time
from Jetson.GPIO import gpio

class Conv(Node):
    def __init__(self):
        super().__init__('convertor')

        self.create_subscription(Twist, '/cmd_vel', self.velo, 1)
        self.create_subscription(Float64, 'right_motor_rpm', self.rpm_rm, 1)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.h, 1)
        

        self.create_subscription(Float64, 'left_motor_rpm', self.rpm_lm, 1)
        self.pose=PoseWithCovarianceStamped()
        
        self.vel = Twist()
        self.radius = 0.3316 *2 # Wheel radius in mm
        self.dist = Twist()
        self.theta = 0.0
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        self.last_time=0

        self.pub = self.create_publisher(Twist, '/not_a_twist', 1)
        self.timer = self.create_timer(0.02, self.velocal)

    def velo(self, msg):
        self.vel = msg
    def h(self,msg):
        self.pose=msg
        print("hello")
        self.dist.linear.x=0.0
        self.dist.linear.y=0.0
        self.dist.angular.z=0.0
      

    def rpm_lm(self, msg):
        self.rpm_l = msg.data
        print("left")
        print(self.rpm_l)

    def rpm_rm(self, msg):
        self.rpm_r = msg.data
        print("right")
        print(self.rpm_r)
    def velocal(self):
        current_time = time.time()  
        if not self.last_time: 
            self.last_time = current_time  
        dt = current_time - self.last_time 
        self.last_time = current_time  
        
        rpm = (self.rpm_l + self.rpm_r) / 2.0
        self.omega = (self.rpm_r - self.rpm_l)*2*math.pi*self.radius / 25.2
        
        self.vel1 = rpm * 2 * math.pi * self.radius*17.75/60
        deld = self.vel1 * dt  /10.0
        delt = self.omega * dt 
        
        delt=(delt/360)*2*math.pi
        

        if self.vel.linear.x or self.vel.angular.z:
        	self.dist.angular.z += delt
        	self.dist.linear.x += deld*np.cos(self.dist.angular.z)	
        	self.dist.linear.y += deld*np.sin(self.dist.angular.z)
        else:
            pass

             
        self.pub.publish(self.dist)


def main(args=None):
    rclpy.init(args=args)
    node = Conv()
    try:
        rclpy.spin(node)

    finally:
        rclpy.shutdown()
        gpio.cleanup()


if __name__ == "__main__":
    main()
