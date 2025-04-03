import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Pose
import tf_transformations
from nav2_simple_commander.robot_navigator import BasicNavigator
from pynput.keyboard import Listener,Key


class Hiyaah(Node):
    def __init__(self):
        super().__init__('ga')
        self.pos = PoseStamped()
        self.gooo = Pose()
        self.vel=Twist()
        self.status=0
        self.abort=0
        self.c=0
        self.start=0
        self.way = Int32MultiArray()
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.u, 1)
        self.create_subscription(Int32MultiArray, '/ahh', self.waypt, 1)
        self.create_subscription(Pose, '/goal_pose', self.goo, 1)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.pub1=self.create_publisher(Twist,'cmd_vel',1)
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()
        
    def on_press(self,key):
        try:
            if key.char == "a":  # Handles regular character keys
                self.status=1

                print("a recived")
            elif key.char == "b":
                self.abort=1
            elif key.char == "c":
                self.start=1
            
        except AttributeError:
            if key == Key.esc:  # Example handling for special keys
                print("Escape key pressed")
            

    def u(self, msg):
        self.pos.pose = msg.pose.pose  # Extract pose correctly

    def waypt(self, msg):
        if not self.c:

            self.way= msg
            self.get_logger().info(f"Received: {self.way.data}")
            self.ab()

    def goo(self, msg):
        self.gooo = msg

    def tf_conv(self, b, c, d):
        x, y, z, w = tf_transformations.quaternion_from_euler(0.0, 0.0, d)
        i = PoseStamped()
        i.header.frame_id = "map"
        i.header.stamp = self.get_clock().now().to_msg()  # Corrected timestamp
        i.pose.position.x = b
        i.pose.position.y = c
        i.pose.position.z = 0.0
        i.pose.orientation.x = x
        i.pose.orientation.y = y
        i.pose.orientation.z = z
        i.pose.orientation.w = w
        return i

    def ab(self):
        self.c=1
        hell = BasicNavigator()

        i = self.tf_conv(1.14, -1.26, 0.00539)

        hell.setInitialPose(i)
        j=PoseWithCovarianceStamped()
        j.pose.pose.position.x=0.0
        j.pose.pose.position.y=0.0
        j.pose.pose.position.z=0.0
        j.pose.pose.orientation.x=0.0
        j.pose.pose.orientation.y=0.0
        j.pose.pose.orientation.z=0.0
        j.pose.pose.orientation.w=0.0
        self.pub.publish(j)
        
        hell.waitUntilNav2Active()

        poses = [
            self.tf_conv(23.7,-0.84,1.57),
            self.tf_conv(8.06, 14.5, 2.35),
            self.tf_conv(-11.6,-32.7, 2.35),
            self.tf_conv(-8.5, 20.80, 0.0),
            self.tf_conv(-38.7,20.8,3.14),
            self.tf_conv(-38.7, 2.8, 3.14),
            self.tf_conv(-39.0,-4.2,-1.57),
            self.tf_conv(-20.7,-28.0,-0.785),
            self.tf_conv(-6.15,-33.5,1.57),
            self.tf_conv(-2.38, -30.2, 1.57),
           
        ]
        i=0
        if self.way.data:

            while i<len(self.way.data):
                if (self.pos.pose.position.x - 0.05 <= self.gooo.position.x and self.gooo.position.x < self.pos.pose.position.x + 0.05) and \
                    (self.pos.pose.position.y - 0.05 <= self.gooo.position.y and self.gooo.position.y < self.pos.pose.position.y + 0.05) and self.status:
                    hell.goToPose(poses[self.way.data[i]])  # Fixed incorrect function call
                

                    while not hell.isTaskComplete():
                        feedback = hell.getFeedback()
                        if self.abort==1:
                            self.abort=0
                            hell.cancelTask()
                            self.vel.linear.x=0.0
                            self.vel.angular.z=0.0
                            self.pub1.publish(self.vel)
                            i=i-1
                            break

                    self.status=0
                    print("hiyaaaaaaaah")


                elif self.start:
                    self.start=0
                    hell.goToPose(self.tf_conv(0.0,0.0,0.0)) 
                    i=-1

                else:
                    i = i-1
                 
                i+=1
            self.c=0
            print(hell.getResult())

        else:
            print("iam not in")




def main(args=None):
    rclpy.init(args=args)
    node = Hiyaah()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
