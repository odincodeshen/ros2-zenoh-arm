#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import numpy as np
import time
import sys

## Measure and print latency statistics for camera topics (image or point cloud).
## It's assumed that the ROS topics are published with wall time (use_wall_time:=True for bringup_sim_launch.py)
## Usage: python3 camera_latency.py [points|image]

class LatencyStatsNode(Node):
    def __init__(self, topic):
        super().__init__('camera_latency')
        self.latencies = []

        if topic == 'image':
            self.sub = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.callback,
                10)
        elif topic == 'points':
            self.sub = self.create_subscription(
                PointCloud2,
                '/camera/points',
                self.callback,
                10)
        else:
            self.get_logger().error("Invalid topic. Use 'points' or 'image'.")
            rclpy.shutdown()
            return

        print(f'Subscribing to topic "{self.sub.topic_name}" to measure latency...')
        self.interval_start = time.time_ns() / 1e9

    def callback(self, msg):
        reception_time = time.time_ns() / 1e9
        message_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        latency = reception_time - message_time
        self.latencies.append(latency * 1000)  # convert to ms
        #print(f"{reception_time:.4f} - {message_time:.4f} = {latency*1000:.4f} ms")

        # Print stats every 3 seconds
        if reception_time - self.interval_start > 3.0:
            self.interval_start = reception_time
            if len(self.latencies) > 0:
                print(f"Mean : {np.mean(self.latencies):.2f} ms | Std : {np.std(self.latencies):.2f} ms | Min : {np.min(self.latencies):.2f} ms | Max : {np.max(self.latencies):.2f} ms")
            self.latencies.clear()

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: python3 camera_latency.py [points|image]")
        sys.exit(1)

    topic = sys.argv[1].lower()
    try:
        rclpy.init(args=args)
        node = LatencyStatsNode(topic)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
