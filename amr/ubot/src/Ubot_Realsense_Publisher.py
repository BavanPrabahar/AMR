#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist  # Import the Twist message type
from cv_bridge import CvBridge

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection')
        
        # Set up the CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Subscribe to the depth image topic published by the RealSense node
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)

        self.set_distance = 0.3
        self.point_depth = [(132, 180), (132, 240), (132, 300), (132, 360), (132, 420), (132, 480), (132, 540),  
                            (176, 180), (176, 240), (176, 300), (176, 360), (176, 420), (176, 480), (176, 540), 
                            (220, 180), (220, 240), (220, 300), (220, 360), (220, 420), (220, 480), (220, 540),  
                            (264, 180), (264, 240), (264, 300), (264, 360), (264, 420), (264, 480), (264, 540),  
                            (308, 180), (308, 240), (308, 300), (308, 360), (308, 420), (308, 480), (308, 540), 
                            (352, 180), (352, 240), (352, 300), (352, 360), (352, 420), (352, 480), (352, 540)]
        
        self.distance1 = []
        self.obstacle_pub = self.create_publisher(Float64, '/obstacle_value', 10)
    def depth_callback(self, msg):
        # Convert ROS image message to OpenCV format
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        
        # Scale depth values to meters (assuming depth image is in meters)
        depth_image = depth_image * 1.0  # Adjust scale if necessary

        for point in self.point_depth:
            d = depth_image[point]
            if  d < 15:
                continue
            elif d != 0:
                self.distance1.append(d/1000)
        
        if self.distance1:
            a = min(self.distance1)
            print(a)
            self.obstacle_pub.publish(Float64(data=float(a)))        
        self.distance1.clear()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

