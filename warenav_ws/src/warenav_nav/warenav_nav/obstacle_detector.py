#!/usr/bin/env python3
"""
obstacle_detector.py
Detects obstacles from LiDAR proximity (primary) + camera (optional YOLOv8).
Publishes emergency stop signal and obstacle markers for Nav2 costmap.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np, json, time


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.declare_parameter('emergency_dist_m', 0.4)
        self.declare_parameter('warning_dist_m',   0.8)
        self.emergency_dist = self.get_parameter('emergency_dist_m').value
        self.warning_dist   = self.get_parameter('warning_dist_m').value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=1)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos)

        self.obstacle_pub = self.create_publisher(Bool,   '/obstacle_detected', 10)
        self.status_pub   = self.create_publisher(String, '/obstacle_status', 10)
        self.marker_pub   = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        self.emergency_active = False
        self.warning_active   = False
        self.scan_count       = 0

        self.get_logger().info('ObstacleDetector ready')

    def scan_cb(self, msg: LaserScan):
        self.scan_count += 1
        ranges = np.array(msg.ranges, dtype=np.float32)
        valid  = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = np.where(valid, ranges, np.inf)

        # Forward 120° arc (most critical for navigation)
        n = len(ranges)
        front_indices = list(range(0, n//6)) + list(range(5*n//6, n))
        front_ranges  = ranges[front_indices]
        min_front     = float(np.min(front_ranges))
        min_all       = float(np.min(ranges))

        emergency = min_front < self.emergency_dist
        warning   = min_all   < self.warning_dist

        if emergency != self.emergency_active or warning != self.warning_active:
            self.emergency_active = emergency
            self.warning_active   = warning

            level = "EMERGENCY" if emergency else ("WARNING" if warning else "CLEAR")
            dist  = min_front if emergency else min_all

            if emergency:
                self.get_logger().warn(
                    f'OBSTACLE {level}: {dist:.2f}m in forward arc!')

            msg_bool = Bool(); msg_bool.data = emergency
            self.obstacle_pub.publish(msg_bool)

            status = {"level": level, "min_dist_m": round(dist, 3)}
            s_msg  = String(); s_msg.data = json.dumps(status)
            self.status_pub.publish(s_msg)

        # Publish closest obstacle as marker every 10 scans
        if self.scan_count % 10 == 0 and min_all < self.warning_dist:
            self.publish_obstacle_marker(msg, ranges, min_all)

    def publish_obstacle_marker(self, scan_msg, ranges, min_range):
        idx = int(np.argmin(ranges))
        angle = scan_msg.angle_min + idx * scan_msg.angle_increment
        ox = min_range * np.cos(angle)
        oy = min_range * np.sin(angle)

        markers = MarkerArray()
        m = Marker()
        m.header.frame_id = 'laser_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'obstacles'; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = float(ox)
        m.pose.position.y = float(oy)
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color.r = 1.0 if min_range < self.emergency_dist else 1.0
        m.color.g = 0.0 if min_range < self.emergency_dist else 0.5
        m.color.b = 0.0; m.color.a = 0.8
        m.lifetime.sec = 1
        markers.markers.append(m)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
