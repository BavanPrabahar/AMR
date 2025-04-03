#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import floor
import time
from std_msgs.msg import Float64,String
class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.scan_msg = []
        self.current_vel = Twist()
        self.create_subscription(Twist, '/webcmd_vel', self.vel_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub1 = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Float64, '/obstacle_value', self.min_callback, 10)
        self.obstacle_status=self.create_publisher(String,'/obstacle_status',1)        
        # Velocity thresholds
        self.threshold_min = 0.55
        self.threshold_max = 0.55
        self.threshold_recovery = 0.33
        
        self.recovery_started = False
        
        self.vel = Twist()
        self.timer = self.create_timer(0.05, self.process_scan)

    def process_scan(self):
        if not self.scan_msg:
            return

        data_per_degree = len(self.scan_msg) / 360
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0


        fov_along_x = 150
        

        
        region_front = self.get_region_distance(data_per_degree, fov_along_x)
        region_back = self.get_region_distance(data_per_degree, fov_along_x, back=True)

        regions = [region_front,region_back]

        closest_distance = float(min(self.scan_msg))
        if closest_distance < 0.33:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.current_vel.linear.x = 0.0
            self.current_vel.angular.z = 0.0
            self.pub1.publish(self.current_vel)
            self.pub.publish(self.vel)           
        
        elif closest_distance > self.threshold_min:
            self.vel.linear.x = self.current_vel.linear.x
            self.vel.angular.z = self.current_vel.angular.z
            self.pub.publish(self.vel)
            self.obstacle_status.publish(String(data="false"))
            time.sleep(1)

        elif self.threshold_max < closest_distance <= self.threshold_min:
            self.vel.linear.x = 0.1 if self.current_vel.linear.x > 0 else -0.1
            self.pub.publish(self.vel)
            #self.get_logger().info("Slowing down :/")

 
        elif closest_distance <= self.threshold_max:
            self.obstacle_status.publish(String(data="true"))   
            if not self.recovery_started:
                self.recovery_started = True
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.current_vel.linear.x = 0.0
                self.current_vel.angular.z = 0.0  
                              
                self.pub.publish(self.vel)
                #self.get_logger().info("Stopped :(")
            else:
                self.recover_robot(regions)

    def get_region_distance(self, data_per_degree, fov, back=False, left=False, right=False):
        if back:
            return min(self.scan_msg[floor(data_per_degree * (180 - (fov / 2))): 
                                      floor(data_per_degree * (180 + (fov / 2)))])
 
        else:
            # Front region calculation (left and right split)
            region_front_left = min(self.scan_msg[floor(-data_per_degree * (fov / 2)):])
            region_front_right = min(self.scan_msg[:floor(data_per_degree * (fov / 2))])
            return min(region_front_left, region_front_right)

    def recover_robot(self, regions):
        self.recovery_started = False  # Reset recovery mode
        if self.current_vel.linear.x:
            if min(regions)==regions[0]:
                if self.current_vel.linear.x>0.0:

                    self.vel.linear.x = 0.0
                    self.vel.angular.z = self.current_vel.angular.z
                    self.pub.publish(self.vel)
                    #self.get_logger().info("Stopped :(")
                    time.sleep(1)
                elif self.current_vel.linear.x<0.0:
                    self.vel.linear.x = self.current_vel.linear.x
                    self.vel.angular.z = self.current_vel.angular.z
                    self.pub.publish(self.vel)
                    time.sleep(1)
            elif min(regions)==regions[1]:
                if self.current_vel.linear.x<0.0:

                    self.vel.linear.x = 0.0
                    self.vel.angular.z = self.current_vel.angular.z
                    self.pub.publish(self.vel)
                    #self.get_logger().info("Stopped :(")
                    time.sleep(1)
                elif self.current_vel.linear.x>0:
                    self.vel.linear.x = self.current_vel.linear.x
                    self.vel.angular.z = self.current_vel.angular.z
                    self.pub.publish(self.vel)
                    time.sleep(1)
        else:
            self.vel.linear.x = 0.0
            self.vel.angular.z = self.current_vel.angular.z
            self.pub.publish(self.vel)
            #self.get_logger().info("Stopped :(")
            time.sleep(1)
        self.current_vel.linear.x = 0.0
        self.current_vel.angular.z = 0.0               
        self.pub1.publish(self.current_vel)


    def min_callback(self, msg):
        self.real = msg.data   
        if self.real > 0.6:
            self.vel.linear.x = self.current_vel.linear.x
            self.vel.angular.z = self.current_vel.angular.z
            self.pub.publish(self.vel)
            self.obstacle_status.publish(String(data="false"))
        elif self.real<=0.5:
                self.vel.linear.x = 0.0
                self.current_vel.linear.x=0.0
                self.vel.angular.z = self.current_vel.angular.z
                self.pub.publish(self.vel)
                self.pub1.publish(self.current_vel)
                #playsound("/home/crobot/Downloads/obstacle_detected_male.mp3")
                #self.obstacle_status.publish(String(data="true_realsense"))
                self.obstacle_status.publish(String(data="true"))



    def vel_callback(self, msg):
        self.current_vel = msg  # Update current velocity

    def lidar_callback(self, msg):
        size_of_plate = 0.25  # Size of the plate for LiDAR
        self.scan_msg = [12 if (y == float('inf') or y < size_of_plate) else y for y in msg.ranges]

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
