from std_msgs.msg import Int32MultiArray
import rclpy
from rclpy.node import Node
import time
class Hello(Node):
    def __init__(self):
        super().__init__('ewhh')
        self.pub=self.create_publisher(Int32MultiArray,'/ahh',1)
        self.hell()
    def hell(self):
        self.msg=Int32MultiArray()
        self.msg.data=[0,1,2,3]
        i=0
        while i<10:
            self.pub.publish(self.msg)
            i+=1


def main(args=None):
    rclpy.init(args=args)
    try:
        node=Hello()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()
if __name__=="__main__":
    main()
