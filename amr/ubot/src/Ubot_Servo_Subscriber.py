#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
from std_msgs.msg import String
import time
global pwm
class Servo(Node):
    def __init__(self):
        global pwm
        super().__init__('servocam')
        self.create_subscription(String, "/servo", self.callback, 10)
        
        pin = 15
        self.x =80

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin, GPIO.OUT)

        # PWM setup
        self.pwm = GPIO.PWM(pin, 150)  # 150Hz frequency for PWM control
        pwm=self.pwm
        self.pwm.start(80)  # Initial duty cycle set to 80
        time.sleep(1)

    def callback(self, msg):
        command = msg.data
        if command == "up":
            d=min(88,self.x+1)
            self.pwm.ChangeDutyCycle(d)
            time.sleep(1)
            self.x = d
            print(self.x)
            print(d)
        elif command == "down":
            d=max(78,self.x-1)
            self.pwm.ChangeDutyCycle(d)
            time.sleep(1)
            self.x = d
            print(self.x)
        elif command == "init":
            self.pwm.ChangeDutyCycle(80)
            time.sleep(1)
            self.x = 80
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = Servo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":

    try:
        main()
    except:
        pass
    finally:
        pwm.ChangeDutyCycle(80)
        time.sleep(1)
        pwm.stop()
        GPIO.cleanup()
        
        
        
        
        
