#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import UInt8MultiArray

#ros2 run my_pkg serial_reader_node --ros-args -p port:=/dev/ttyUSB0 -p topic_out:=/lidar1/serial

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')

        port = self.declare_parameter('port', '/dev/ttyUSB0').get_parameter_value().string_value
        baud = self.declare_parameter('baudrate', 115200).get_parameter_value().integer_value
        topic = self.declare_parameter('topic_out', 'serial').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(UInt8MultiArray, topic, 10)
        self.ser = serial.Serial(port, baud, timeout=0.001) #provare con 0.010 (10ms)
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        while rclpy.ok():
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                msg = UInt8MultiArray()
                msg.data = list(data)  # Converti bytes in lista di interi
                self.publisher_.publish(msg)
                #self.get_logger().info(f'Pubblicato: {msg.data}')

def main():
    rclpy.init()
    node = SerialReaderNode()
    rclpy.spin(node)
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
