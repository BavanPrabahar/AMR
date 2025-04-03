import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Pose
import tf_transformations
from nav2_simple_commander.robot_navigator import BasicNavigator
from pynput.keyboard import Listener,Key
import os
from rclpy.qos import QoSProfile, DurabilityPolicy
import time
var1=0.0
var2=0.0
var3=0.0


class Hiyaah(Node):
    def __init__(self):
        super().__init__('ga')
        self.pos = PoseWithCovarianceStamped()
        self.gooo = Pose()
        self.vel=Twist()
        self.status=0
        self.no=0
        self.change_p=0
        self.initial=0
        self.abort=0
        self.c=0
        self.once=0
        self.yes=0
        self.start=0
        self.way = Int32MultiArray()
        qos_transient_local = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.u, qos_transient_local)
        self.create_subscription(Int32MultiArray, '/ahh', self.waypt, 1)
        self.create_subscription(Pose, '/goal_pose', self.goo, 1)

        self.pub2=self.create_publisher(Int32,'/popup',1)
        self.pub1=self.create_publisher(Twist,'cmd_vel',1)
        self.listener = Listener(on_press=self.on_press)
        self.b=Int32()
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
            elif key.char =="p":
                self.change_p=1
            elif key.char =="i":
                self.initial=1
            elif key.char=="y":
                self.yes=1
            elif key.char=="n":
                self.no=1
            
        except AttributeError:
            if key == Key.esc:  # Example handling for special keys
                print("Escape key pressed")
            

    def u(self, msg):
        self.pos = msg # Extract pose correctly
        print(self.pos.pose.pose.position.x)

    def waypt(self, msg):


        self.way= msg
        self.get_logger().info(f"Received: {self.way.data}")
        if not self.c:

            self.once=1
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
        global var1,var2,var3
        print(var1,var2,var3)
        self.c=1
        hell = BasicNavigator()
        if self.once:
            while not self.initial:
                pass
            while not self.yes and not self.no:
                pass
            if self.yes:
                self.yes=0
                i = self.tf_conv(0.0,0.0,0.0)
            elif self.no:
                i = self.tf_conv(var1,var2,var3)
        else:
            print("naaan than")
            i=self.tf_conv(var1,var2,var3)


        hell.setInitialPose(i)

        
        hell.waitUntilNav2Active()
        

        poses = [
            self.tf_conv(0.0, 0.0, 1.57),
            self.tf_conv(0.0, 0.0, 0.0),
            self.tf_conv(0.0, 0.0, -1.57),
            self.tf_conv(3.0, 0.0, 0.0),
            self.tf_conv(6.5, 0.0, 0.0),
            self.tf_conv(7.0, -5.0, -1.57),
            self.tf_conv(3.5, 0.0, 0.0),
            self.tf_conv(6.5, 0.0, 0.0),
            self.tf_conv(7.0, -5.0, -1.57),
            self.tf_conv(3.5, 0.0, 0.0),
            self.tf_conv(6.5, 0.0, 0.0),
            self.tf_conv(7.0, -5.0, -1.57)
        ]
        i=0
        if self.way.data:

            while i<len(self.way.data):
                if  self.status:
                    hell.goToPose(poses[self.way.data[i]])  # Fixed incorrect function call
                    self.b.data=1
                    self.pub2.publish(self.b)
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
                elif self.change_p:
                    self.change_p=0
                    
                    break

                else:
                    i = i-1
                 
                i+=1
            
            print(hell.getResult())


        else:
            print("iam not in")
            
            
        print(self.pos.pose.pose.orientation.z)
        var1=self.pos.pose.pose.position.x
        var2=self.pos.pose.pose.position.y
        var3=self.pos.pose.pose.orientation.z
        self.once=0
        self.ab()
def load_variables(filename, default_values=(0.0,0.0, 0.0)):
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            return tuple(map(float, file.read().strip().split(',')))  # Convert back to integers
    print("hi")
    return default_values  # Default

def save_variables(filename, values):
    with open(filename, 'w') as file:
        file.write(','.join(map(str, values)))

def main(args=None):
    global var1,var2,var3
    rclpy.init(args=args)
    filename = "variables.txt"
    var1, var2, var3 = load_variables(filename)  
    node = Hiyaah()
    try:
        rclpy.spin(node)
    finally:
        save_variables(filename, (var1,var2,var3))  # Save the new values
        rclpy.shutdown()


if __name__ == "__main__":
    main()
