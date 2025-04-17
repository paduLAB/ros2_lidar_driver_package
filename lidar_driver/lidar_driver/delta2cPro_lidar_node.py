#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, UInt8
from sensor_msgs.msg import LaserScan

import numpy as np

from lidar_driver.delta2cPro_constants import *

'''
ros2 run <your_package_name> delta2cPro_lidar_node \
  --ros-args \
  -p topic_in:=/lidar/serial \
  -p topic_out:=/lidar/scan \
  -p rpm_topic:=/lidar/rpm
'''

class delta2cProLidarNode(Node):
    def __init__(self):
        super().__init__('delta2cPro_lidar_node')

        self.declare_parameter('topic_in', 'serial')
        self.declare_parameter('topic_out', 'scan')
        self.declare_parameter('rpm_topic', 'rpm')

        topic_in = self.get_parameter('topic_in').get_parameter_value().string_value
        topic_out = self.get_parameter('topic_out').get_parameter_value().string_value
        rpm_topic = self.get_parameter('rpm_topic').get_parameter_value().string_value

        self.scan_pub = self.create_publisher(LaserScan, topic_out, 100)
        self.rpm_pub = self.create_publisher(UInt8, rpm_topic, 100)
        self.subscription = self.create_subscription(UInt8MultiArray, topic_in, self.callback, 100)
        
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = 'laser_link'
        self.scan_msg.range_min = LASERSCAN_RANGE_MIN
        self.scan_msg.range_max = LASERSCAN_RANGE_MAX

        self.buffer = bytearray()

    def callback(self, msg):
        self.buffer.extend(msg.data)
        while True:
            if len(self.buffer) <= PACKET_HEADER_LEN:
                #self.get_logger().info("wait for more data")
                return  #exit from callback, new data will call callback again
            
            if self.buffer[PACKET_SYNC_INDEX] != PACKET_SYNC_BYTE:
                self.buffer.pop(0)
                continue
            if self.buffer[1] != 0:
                self.buffer.pop(0)
                continue
            
            total_len = int.from_bytes(self.buffer[PACKET_LENGTH_INDEX:(PACKET_LENGTH_INDEX+PACKET_LENGTH_LEN)], 'big') + PACKET_HEADER_LEN -1
            #print(f"total len: {total_len}, byte0: {self.buffer[0]}, byte1: {self.buffer[1]}, , byte2: {self.buffer[2]}")
            
            if len(self.buffer) < total_len:
                #self.get_logger().warn("Packet too short. wait for more data")
                return

            packet = self.buffer[:total_len]
            self.buffer = self.buffer[total_len:]
            
            #print("packet:")  
            #print(' '.join(f'{b:02X}' for b in packet))            

            if self.verify_checksum(packet):
                self.process_packet(packet)

    def verify_checksum(self, packet):
        computed_checksum = sum(packet[:-PACKET_CHECKSUM_LEN]) & 0xFFFF #compute checksum and consider only last 2 byte
        received_chucksum = int.from_bytes(packet[-PACKET_CHECKSUM_LEN:], 'big') 
        
        if computed_checksum == received_chucksum:
            checksum_verified = True
        else:
            checksum_verified = False
 
        #print(f"checksum: {checksum_verified}")    
        return checksum_verified

    def process_packet(self, packet):
        cmd = packet[PACKET_CMD_INDEX]
        #print(f"comando: {cmd}")
        payload = packet[PACKET_PAYLOAD_INDEX:-PACKET_CHECKSUM_LEN]

        if cmd == PACKET_CMD_DATA:
            if len(payload) < PAYLOAD_DATA_INDEX + PAYLOAD_DATA_PER_ANGLE_SIZE:
                #self.get_logger().warn("Data payload too short. Discarded!")
                return

            rpm = payload[PAYLOAD_RPM_INDEX]

            start_angle = np.deg2rad((int.from_bytes(payload[PAYLOAD_ANGLE_START_INDEX:PAYLOAD_ANGLE_START_INDEX + PAYLOAD_ANGLE_START_LEN], 'big')) * ANGLE_SCALE_FACTOR)
            end_angle = np.deg2rad((int.from_bytes(payload[PAYLOAD_ANGLE_END_INDEX:PAYLOAD_ANGLE_END_INDEX + PAYLOAD_ANGLE_END_LEN], 'big')) * ANGLE_SCALE_FACTOR)

            ranges = []
            intensities = []

            i = PAYLOAD_DATA_INDEX
            while i + PAYLOAD_DATA_PER_ANGLE_SIZE <= len(payload):
                strength = payload[i]
                distance = int.from_bytes(payload[(i+1):(i+PAYLOAD_DATA_PER_ANGLE_SIZE)], 'big') * DISTANCE_SCALE_FACTOR
                intensities.append(float(strength))
                ranges.append(distance)
                i += PAYLOAD_DATA_PER_ANGLE_SIZE
                
            # If the packet is structurally valid but contains no measurements,
            # skip publishing to avoid sending empty LaserScan messages
            if not ranges:
                self.get_logger().warn(f"Packet {hex(PACKET_CMD_DATA).upper()}: valid but contains no measurements. Discarded!")
                return
       
            self.scan_msg.header.stamp = self.get_clock().now().to_msg()

            self.scan_msg.angle_min = start_angle
            self.scan_msg.angle_max = end_angle
            self.scan_msg.angle_increment = (end_angle - start_angle) / (len(ranges) - 1)
            
            # Compute time for one full 360° rotation
            # scan_time is needed by ROS to understand how long it took to collect the scan
            if rpm > 0:
                self.scan_msg.scan_time = SECONDS_PER_MINUTE / rpm
            else:
                self.scan_msg.scan_time = DEFAULT_SCAN_TIME  # fallback if rpm is invalid

            self.scan_msg.ranges = ranges
            self.scan_msg.intensities = intensities
            
            self.publish_rpm(rpm)
            self.scan_pub.publish(self.scan_msg)
            #self.get_logger().info("published")

        elif cmd == PACKET_CMD_ERR:
            if len(payload) == PAYLOAD_RPM_LEN:
                rpm = payload[PAYLOAD_RPM_INDEX]
                self.publish_rpm(rpm)
            else:
                self.get_logger().warn(f"Packet {hex(PACKET_CMD_ERR).upper()}: payload is not 1 byte long. Discarded")
        else:
            self.get_logger().warn("Command not defined")

    def publish_rpm(self, rpm_value):
        msg = UInt8()
        msg.data = rpm_value
        self.rpm_pub.publish(msg)
        #self.get_logger().info("rpm")

def main():
    rclpy.init()
    node = delta2cProLidarNode()
    rclpy.spin(node)
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
