#!/usr/bin/env python3
import json
import websocket
import threading
import time
import rel
import webbrowser
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String
import requests
from datetime import datetime
from std_msgs.msg import Float64
check = 0
currentPosition = 50
class Subscriber(Node):
    def __init__(self):
        super().__init__('ros2_subscriber')
        self.subscription = self.create_subscription(Float64,'battery_voltage_percentage',self.bat_callback,10)
        self.obs_sub =self.create_subscription(String,'/obstacle_status',self.obs_callback,10)
        self.subscription  # prevent unused variable warning
        self.obs_sub
        websocket.enableTrace(True)
        websocket.setdefaulttimeout(60)
        self.ws = websocket.WebSocketApp(
            "wss://jb1kqelj3e.execute-api.us-east-1.amazonaws.com/Prod?auth=cvchhjgch@jjh",
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        
        # Set dispatcher for automatic reconnection
        self.ws.run_forever(dispatcher=rel)
        #rel.signal(2, rel.abort)  # Keyboard Interrupt
        #rel.dispatch()
    def on_message(self, ws, message):
        a = json.loads(message)
        #print(a["data"])
        #try:
            #a["data"], time_string = a["data"].split("_")
            #time_hours, time_minutes, time_seconds = time_string.split(":")
        #except ValueError:
         #   print("Invalid message format received.")
          #  return


        #print("Received parameters are: ", a["data"])
                # Check for time interval and discard old messages
    def on_error(self, ws, error):
        print("Encountered error:", error)

    def on_close(self, ws, close_status_code, close_msg):
        print(f"Closed WebSocket Connection - Status: {close_status_code}, Message: {close_msg}")

    def on_open(self, ws):
        print("Opened WebSocket Connection")
    def bat_callback(self, msg):
        """This callback function gets called when new data is received from ROS2."""
        percentage = int(round(msg.data, 0))
        print(percentage)
        self.send_message("battery",percentage,"percent")
    def obs_callback(self, msg):
        obs_status=msg.data
        print(obs_status)
        self.send_message("obstacle",obs_status,"obstacle")
        time.sleep(3)
    def send_message(self, value , message,action):
        if self.ws.sock and self.ws.sock.connected:
            self.ws.send(json.dumps({"action":"sendmessage","data":value,action:message}))
            print(f"Sent message: {message}")
        else:
            print("WebSocket is not connected. Cannot send message.")
        

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
