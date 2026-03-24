#!/usr/bin/env python3
"""inventory_logger.py — saves /inventory_log entries to JSON files."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, os, datetime


class InventoryLogger(Node):
    def __init__(self):
        super().__init__('inventory_logger')
        self.log_dir = '/tmp/warenav_logs'
        os.makedirs(self.log_dir, exist_ok=True)
        self.entries = []
        self.sub = self.create_subscription(
            String, '/inventory_log', self.cb, 10)
        self.get_logger().info(f'InventoryLogger → {self.log_dir}')

    def cb(self, msg: String):
        try:
            entry = json.loads(msg.data)
            self.entries.append(entry)
            self.get_logger().info(
                f'[LOG] {entry.get("shelf","?")} — {entry.get("status","?")}')
            path = os.path.join(
                self.log_dir, f'inv_{datetime.date.today()}.json')
            with open(path, 'w') as f:
                json.dump(self.entries, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Log error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = InventoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
