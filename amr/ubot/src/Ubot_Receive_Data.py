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
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import requests
from datetime import datetime
import numpy as np
import asyncio
from std_msgs.msg import Float64

check = 0
currentPosition = 50

class Tele(Node):
    def __init__(self):
        super().__init__('Telepresence')
        self.talker = self.create_publisher(Twist, '/webcmd_vel', 1)
        self.servo_pub=self.create_publisher(String,'/servo',10)
        self.bat_sub=self.create_subscription(Float64, 'battery_voltage_percentage', self.bat_callback, 10)
        self.create_subscription(String, '/obstacle_status', self.obs_callback, 10)
        self.duty_cycle=65
        self.com=0
        #self.bat_data = 0.0
        self.vel = Twist()
        self.bat_data=0.0
        #self.bat_data= Float32()
        websocket.enableTrace(True)
        websocket.setdefaulttimeout(60)
        self.ws = websocket.WebSocketApp(
            "wss://jb1kqelj3e.execute-api.us-east-1.amazonaws.com/Prod?auth=cvchhjgch@jjh",
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
            
        )
        #self.start_periodic_sender()  # Start periodic WebSocket messages
    # Set dispatcher for automatic reconnection
        self.ws.run_forever()

    '''def get_utc_time(self):
        try:
            response = requests.get('http://worldtimeapi.org/api/timezone/Etc/UTC')
            print("Started at: ", datetime.utcnow())
            response.raise_for_status()
            time_data = response.json()
            current_utc_time = time_data['datetime']
            return datetime.fromisoformat(current_utc_time.rstrip('Z'))
        except requests.RequestException as e:
            print(f"Error fetching time: {e}")
            return None'''

    def start_periodic_sender(self):
        def periodic_send():
            print("Liveness")
            while True:
                if self.ws.sock and self.ws.sock.connected:
                    self.send_message("ping")
                time.sleep(2)  # Adjust interval as needed
        
        threading.Thread(target=periodic_send, daemon=True).start()


    def on_message(self, ws, message):
        global currentPosition, check
        b=1
        linear = True
        stopping = False
        moving_linear = False
        vel = self.vel
        duty_cycle=self.duty_cycle
        c_duty_cycle=0
        talker = self.talker
        servo_pub=self.servo_pub
        print("Message Received")
        a = json.loads(message)
        print(a["data"])


        print("Received parameters are: ", a["data"])
                # Check for time interval and discard old messages
        if (b==1):
            if a["data"] == "left":
                linear = False
                vel.linear.x=0.0
                vel.angular.z = -math.pi * 0.05
                talker.publish(vel)
                time.sleep(0.5)
                vel.angular.z = 0.0
                talker.publish(vel)
            elif a["data"] == "right":
                linear = False
                vel.linear.x=0.0
                vel.angular.z = math.pi * 0.05
                talker.publish(vel)
                time.sleep(0.5)
                vel.angular.z = 0.0
                talker.publish(vel)
            elif a["data"] == "forward":
                linear = True
                if not moving_linear:
                    vel.linear.x = 0.2
                    talker.publish(vel)
            elif a["data"] == "backward":
                linear = True
                vel.linear.x = -0.1
                talker.publish(vel)
                time.sleep(2)
                vel.linear.x = 0.0
                talker.publish(vel)
            elif a["data"] == "connect":
                print(b)
                webbrowser.open(a["Link"])
            elif a["data"] == "stop":
                if linear:
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
                    talker.publish(vel)
            elif "cameraUp" in a["data"]:
                    msg = String()
                    msg.data = str("up")
                    servo_pub.publish(msg)
                    print(f"Duty cycle increased")
          
            elif "cameraDown" in a["data"]:
                    msg = String()
                    msg.data = str("down")
                    servo_pub.publish(msg)
                    print(f"Duty cycle dcreased")            
        else:
            print("Unknown command or delay.")
        
    def on_error(self, ws, error):
        print("Encountered error:", error)

    def on_close(self, ws, close_status_code, close_msg):
        print(f"Closed WebSocket Connection - Status: {close_status_code}, Message: {close_msg}")
    
    def send_message(self, message):
        if self.ws.sock and self.ws.sock.connected:
            self.ws.send(json.dumps({"action":"sendmessage","data":message}))
            print(f"Sent message: {message}")
        else:
            print("WebSocket is not connected. Cannot send message.")


    def on_open(self, ws):
        print("Opened WebSocket Connection")
    def bat_callback(self, msg):
        self.bat_data = msg.data
        # send battery data to the server via websocket
        #self.send_message(self.bat_data)
        print(self.bat_data)
    def obs_callback(self, msg):
        self.obs_status=msg.data
        #self.send_message(self.obs_status)
    def run_websocket():
        ws_url = "wss://jb1kqelj3e.execute-api.us-east-1.amazonaws.com/Prod?auth=cvchhjgch@jjh"  # WebSocket echo server for testing
        ws = websocket.WebSocketApp(
            ws_url,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
        )

        # Run the WebSocket connection
        ws.run_forever()
def main(args=None):
    rclpy.init(args=args)
    node = Tele()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
