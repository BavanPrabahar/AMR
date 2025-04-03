#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
import time

class Conv(Node):
    def __init__(self):
        super().__init__('convertor')

        self.create_subscription(Twist, '/cmd_vel', self.velo, 1)
        self.create_subscription(Float64, 'right_motor_rpm', self.rpm_lm, 1)
        self.create_subscription(Float64, 'left_motor_rpm', self.rpm_rm, 1)

        self.vel = Twist()
        self.radius = 0.075  # Wheel radius in mm
        self.dist = Twist()
        self.theta = 0.0
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        self.last_time=0

        self.pub = self.create_publisher(Twist, '/not_a_twist', 1)
        self.timer = self.create_timer(0.4, self.velocal)

    def velo(self, msg):
        self.vel = msg

    def rpm_lm(self, msg):
        self.rpm_l = msg.data

    def rpm_rm(self, msg):
        self.rpm_r = msg.data

    def velocal(self):
        current_time = time.time()  
        if not self.last_time: 
            self.last_time = current_time  
        dt = current_time - self.last_time 
        self.last_time = current_time  
        
        rpm = (self.rpm_l + self.rpm_r) / 2.0
        self.omega = (self.rpm_r - self.rpm_l)*2*math.pi*self.radius / 25.2
        
        self.vel1 = rpm * 2 * math.pi * self.radius*18/60
        deld = self.vel1 * dt  
        delt = self.omega * dt 


        
        if self.vel.angular.z:

            self.theta += delt
            self.dist.angular.z = self.theta
        elif self.vel.linear.x:
            self.dist.linear.x += deld
        else:
            pass
        
        self.pub.publish(self.dist)


def main(args=None):
    rclpy.init(args=args)
    node = Conv()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
