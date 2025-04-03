#!/usr/bin/env python3

import Jetson.GPIO as gpio
from time import sleep
from threading import Thread, ThreadError

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

gpio.setmode(gpio.TEGRA_SOC)


emergency_stop = "GP113_PWM7"  # Board Pin 33
front_bumper = "GP125"  # Board Pin 35
back_bumper = "GP38_SPI3_MOSI"  # Board Pin 37

red = "GP73_UART1_CTS_N"  # Board Pin 36
green = "GP124"  # Board Pin 38
blue = "GP123"  # Board Pin 40


class SafetyController(Node):

    def __init__(self):
        super().__init__('ubot_safety_controller')

        # Setup GPIO pins
        self.setup_gpio()

        # Create a publisher
        self.pub = self.create_publisher(Bool, 'safety_status', 1)

        # Create a timer to periodically check the GPIO status
        self.timer = self.create_timer(0.05, self.check_status)

    def setup_gpio(self):
        gpio.setup(emergency_stop, gpio.IN)
        gpio.setup(front_bumper, gpio.IN)
        gpio.setup(back_bumper, gpio.IN)

        gpio.setup(red, gpio.OUT)
        gpio.setup(blue, gpio.OUT)
        gpio.setup(green, gpio.OUT)

    def constant_red(self):
        gpio.output(red, True)
        gpio.output(green, False)
        gpio.output(blue, False)

    def constant_green(self):
        gpio.output(red, False)
        gpio.output(blue, False)
        gpio.output(green, True)

    def blink_red(self):
        gpio.output(green, False)
        gpio.output(red, True)
        gpio.output(blue, False)
        sleep(0.25)
        gpio.output(red, False)
        gpio.output(blue, True)
        sleep(0.25)
        gpio.output(blue, False)
        gpio.output(green, True)
        sleep(0.25)

    def check_status(self):
        status_e = gpio.input(emergency_stop)
        status_fb = gpio.input(front_bumper)
        status_bb = gpio.input(back_bumper)
        b=1
        if b==1:
            self.blink_red()

        if not (status_e and status_fb and status_bb):
            self.publish_bool(False)

            if not status_e:
                self.constant_red()

            elif not (status_fb and status_bb):
                self.blink_red()

        else:
            self.publish_bool(True)


    def publish_bool(self, msg):
        bool_msg = Bool()
        bool_msg.data = msg
        self.pub.publish(bool_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_controller = SafetyController()

  
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
