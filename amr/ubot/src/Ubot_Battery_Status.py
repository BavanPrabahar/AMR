#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
from datetime import datetime
import time

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.publisher_ = self.create_publisher(Float64, 'battery_voltage_percentage', 10)
        self.log_file_path = "Ubot_battery_log.txt"
        
        # Initialize UART device
        self.uart_device = serial.Serial('/dev/ttyTHS0', 9600, timeout=1, write_timeout=1)
        
        self.timer = self.create_timer(1.0, self.timer_callback)  # Call every second

    def convert_to_value(self, hex_value):
        """Convert hexadecimal value to float representation."""
        value = int(hex_value, 16)
        if value >= 0x8000:  # Handle two's complement
            value -= 10000
        return value * 0.01

    def voltage_to_percentage(self, voltage):
        """Convert voltage to percentage based on defined thresholds."""
        if voltage >= 26.55:
            return 100.0
        elif voltage <= 23.5:
            return 10.0
        else:
            return (voltage - 23.5) / (26.55 - 23.5) * (100 - 10) + 10

    def timer_callback(self):
        try:
            data = self.uart_device.readline().decode().strip()
            if data:
                # Clean and extract voltage
                data = data.replace("CAN Data:", "").strip().replace(" ", "")
                
                if len(data) >= 13 and data[2:5] == "100":  # Ensure valid data length
                    voltage = self.convert_to_value(data[5:9])
                    current = self.convert_to_value(data[9:13])
                    
                    # Convert voltage to percentage
                    voltage_percentage = self.voltage_to_percentage(voltage)
                    output = f"Total voltage: {voltage:.2f} V\nCurrent: {current:.2f} mA\nBattery Percentage: {voltage_percentage:.2f}%\n"
                    
                    # Alert if voltage is below 23.5 V
                    if voltage < 23.5:
                        self.get_logger().warning("Alert: Voltage is below safe level!")

                    # Log the output immediately
                    self.get_logger().info(output.strip())
                    with open(self.log_file_path, "a") as log_file:
                        log_file.write(f"{datetime.now()}: {output.strip()}\n")
                    
                    # Publish the voltage percentage
                    msg = Float64()
                    msg.data = voltage_percentage
                    self.publisher_.publish(msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
        except ValueError as e:
            self.get_logger().error(f"Value error: {e}")

def main(args=None):
    rclpy.init(args=args)
    battery_monitor_node = BatteryMonitorNode()
    rclpy.spin(battery_monitor_node)

    # Cleanup
    battery_monitor_node.uart_device.close()
    battery_monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

