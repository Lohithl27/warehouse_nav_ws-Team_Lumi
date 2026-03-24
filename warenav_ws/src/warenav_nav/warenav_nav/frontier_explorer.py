#!/usr/bin/env python3
"""
frontier_explorer.py
══════════════════════════════════════════════════════════════
WarehouseNav AMR — Frontier-Based Autonomous Explorer

The robot explores unknown environments exactly like the AI
open-world navigation in the referenced YouTube video:
  → Identify the boundary between "known" and "unknown" space
  → Drive toward the nearest/largest unexplored frontier
  → Repeat until the entire space is mapped

Algorithm:
  1. Subscribe to /map (OccupancyGrid from slam_toolbox)
  2. Find all frontier cells:
       - FREE cell (0) adjacent to UNKNOWN cell (-1)
  3. Cluster adjacent frontier cells into groups
  4. Score each cluster by size + proximity
  5. Send best cluster centroid as Nav2 goal
  6. When Nav2 reaches goal → repeat
  7. Stop when coverage ≥ threshold OR no frontiers remain

Reference repos:
  - AAISHAA1585/Autonomous_Indoor_Robot-ROS2 (Nav2 action pattern)
  - bhavikmk/warehousebot (ROS2 Humble structure)

MAHE Mobility Challenge 2026 — Robotics Track
══════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus

import numpy as np
import math
import time
from enum import Enum


class ExploreState(Enum):
    IDLE      = "IDLE"
    EXPLORING = "EXPLORING"
    NAVIGATING= "NAVIGATING"
    COMPLETE  = "COMPLETE"
    FAILED    = "FAILED"


class FrontierExplorer(Node):
    """
    Implements frontier-based autonomous exploration.
    Publishes frontier markers to RViz2 for visualisation.
    Transitions to COMPLETE when map_coverage >= threshold.
    """

    def __init__(self):
        super().__init__('frontier_explorer')

        # ── PARAMETERS ────────────────────────────────
        self.declare_parameter('coverage_threshold',   0.75)
        self.declare_parameter('min_frontier_size',    10)
        self.declare_parameter('goal_timeout_sec',     30.0)
        self.declare_parameter('update_rate_hz',       2.0)
        self.declare_parameter('min_frontier_dist',    0.5)
        self.declare_parameter('max_frontier_dist',    8.0)
        self.declare_parameter('auto_start',           True)

        self.cov_threshold  = self.get_parameter('coverage_threshold').value
        self.min_front_size = self.get_parameter('min_frontier_size').value
        self.goal_timeout   = self.get_parameter('goal_timeout_sec').value
        self.update_rate    = self.get_parameter('update_rate_hz').value
        self.min_dist       = self.get_parameter('min_frontier_dist').value
        self.max_dist       = self.get_parameter('max_frontier_dist').value
        self.auto_start     = self.get_parameter('auto_start').value

        # ── NAV2 ACTION CLIENT ────────────────────────
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── PUBLISHERS ────────────────────────────────
        self.frontier_viz_pub = self.create_publisher(
            MarkerArray, '/frontier_markers', 10)
        self.status_pub = self.create_publisher(
            String, '/explore_status', 10)
        self.done_pub = self.create_publisher(
            Bool, '/exploration_complete', 10)

        # ── SUBSCRIBERS ───────────────────────────────
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.pose_callback, 10)

        # ── STATE ─────────────────────────────────────
        self.state           = ExploreState.IDLE
        self.current_map     = None
        self.robot_x         = -9.5   # Start at charging dock
        self.robot_y         = -7.2
        self.map_coverage    = 0.0
        self.frontiers_found = 0
        self.goals_sent      = 0
        self.goals_succeeded = 0
        self.current_goal    = None
        self.goal_send_time  = None
        self._goal_handle    = None

        # ── TIMERS ────────────────────────────────────
        self.explore_timer = self.create_timer(
            1.0 / self.update_rate, self.explore_step)
        self.status_timer = self.create_timer(3.0, self.publish_status)

        if self.auto_start:
            self.get_logger().info(
                'FrontierExplorer: auto_start=true — will begin after Nav2 ready')
            self.start_timer = self.create_timer(15.0, self.auto_begin)

        self.get_logger().info('═' * 55)
        self.get_logger().info('WarehouseNav AMR — Frontier Explorer')
        self.get_logger().info(f'  Coverage threshold: {self.cov_threshold:.0%}')
        self.get_logger().info(f'  Min frontier size: {self.min_front_size} cells')
        self.get_logger().info(f'  Goal timeout: {self.goal_timeout}s')
        self.get_logger().info('═' * 55)

    # ── CALLBACKS ─────────────────────────────────────

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg
        total = len(msg.data)
        if total == 0:
            return
        known = sum(1 for c in msg.data if c >= 0)
        self.map_coverage = known / total

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def auto_begin(self):
        self.start_timer.cancel()
        self.begin_exploration()

    # ── EXPLORATION LOGIC ─────────────────────────────

    def begin_exploration(self):
        """Start the exploration loop."""
        if not self._nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 not available — cannot start exploration')
            self.state = ExploreState.FAILED
            return
        self.state = ExploreState.EXPLORING
        self.get_logger().info('Exploration STARTED')

    def explore_step(self):
        """Called at update_rate_hz — core exploration tick."""
        if self.state == ExploreState.IDLE:
            return

        if self.state == ExploreState.COMPLETE:
            done_msg = Bool(); done_msg.data = True
            self.done_pub.publish(done_msg)
            return

        if self.state == ExploreState.FAILED:
            return

        # Check coverage complete
        if self.map_coverage >= self.cov_threshold:
            self.get_logger().info(
                f'Exploration COMPLETE — coverage: {self.map_coverage:.1%}')
            self.state = ExploreState.COMPLETE
            done_msg = Bool(); done_msg.data = True
            self.done_pub.publish(done_msg)
            return

        # Check goal timeout
        if (self.state == ExploreState.NAVIGATING and
                self.goal_send_time is not None and
                time.time() - self.goal_send_time > self.goal_timeout):
            self.get_logger().warn('Goal timeout — picking new frontier')
            self.state = ExploreState.EXPLORING
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()

        # Send next frontier goal
        if self.state == ExploreState.EXPLORING and self.current_map is not None:
            frontier = self.find_best_frontier()
            if frontier is not None:
                self.send_goal(frontier[0], frontier[1])
            else:
                if self.map_coverage > 0.5:
                    self.get_logger().info(
                        f'No more frontiers — coverage: {self.map_coverage:.1%}')
                    self.state = ExploreState.COMPLETE
                else:
                    self.get_logger().warn(
                        f'No frontiers found yet (coverage {self.map_coverage:.1%}) '
                        f'— waiting for map...')

    def find_best_frontier(self):
        """
        Find the best frontier cell to explore next.
        Returns (world_x, world_y) or None.
        """
        if self.current_map is None:
            return None

        msg = self.current_map
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)

        # Find frontier cells: free (0) adjacent to unknown (-1)
        free    = (data == 0)
        unknown = (data == -1)

        # Dilate unknown by 1 pixel
        from scipy.ndimage import binary_dilation
        try:
            unknown_dilated = binary_dilation(unknown, iterations=1)
            frontier_mask = free & unknown_dilated
        except ImportError:
            # Fallback without scipy — manual neighbour check
            frontier_mask = np.zeros_like(free)
            for dy, dx in [(-1,0),(1,0),(0,-1),(0,1)]:
                shifted = np.roll(np.roll(unknown, dy, axis=0), dx, axis=1)
                frontier_mask |= (free & shifted)

        frontier_indices = np.argwhere(frontier_mask)
        if len(frontier_indices) == 0:
            return None

        self.frontiers_found = len(frontier_indices)

        # Cluster frontiers (simple: group by proximity in grid coords)
        # Convert to world coords
        world_pts = []
        for (row, col) in frontier_indices:
            wx = ox + col * res + res / 2
            wy = oy + row * res + res / 2
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if self.min_dist <= dist <= self.max_dist:
                world_pts.append((wx, wy, dist))

        if not world_pts:
            # Relax distance constraint
            world_pts = []
            for (row, col) in frontier_indices:
                wx = ox + col * res + res / 2
                wy = oy + row * res + res / 2
                dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
                world_pts.append((wx, wy, dist))

        if not world_pts:
            return None

        # Score: prefer large clusters close to robot
        # Simple: pick point that maximises (frontier_neighbours / distance)
        # Group into 0.5m clusters
        clusters = self.cluster_points(world_pts)
        if not clusters:
            return None

        best = max(clusters, key=lambda c: c['size'] / (c['dist'] + 0.5))

        # Publish frontier markers for RViz2
        self.publish_frontier_markers(clusters, best)

        return (best['cx'], best['cy'])

    def cluster_points(self, points, cluster_radius=1.0):
        """Simple greedy clustering of world points."""
        clusters = []
        used = set()

        for i, (x, y, d) in enumerate(points):
            if i in used:
                continue
            cluster = {'pts': [(x, y)], 'cx': x, 'cy': y}
            used.add(i)
            for j, (x2, y2, d2) in enumerate(points):
                if j in used:
                    continue
                if math.hypot(x - x2, y - y2) <= cluster_radius:
                    cluster['pts'].append((x2, y2))
                    used.add(j)
            # Centroid
            xs = [p[0] for p in cluster['pts']]
            ys = [p[1] for p in cluster['pts']]
            cluster['cx'] = sum(xs) / len(xs)
            cluster['cy'] = sum(ys) / len(ys)
            cluster['size'] = len(cluster['pts'])
            cluster['dist'] = math.hypot(
                cluster['cx'] - self.robot_x,
                cluster['cy'] - self.robot_y)
            if cluster['size'] >= self.min_front_size:
                clusters.append(cluster)

        return clusters

    def send_goal(self, x: float, y: float):
        """Send a Nav2 NavigateToPose goal."""
        if not self._nav.server_is_ready():
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        self.current_goal = (x, y)
        self.goal_send_time = time.time()
        self.goals_sent += 1
        self.state = ExploreState.NAVIGATING

        self.get_logger().info(
            f'Exploring frontier @ ({x:.2f}, {y:.2f}) | '
            f'coverage: {self.map_coverage:.1%} | '
            f'goal #{self.goals_sent}'
        )

        future = self._nav.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Frontier goal rejected — trying next')
            self.state = ExploreState.EXPLORING
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_succeeded += 1
            self.get_logger().info(
                f'Reached frontier ({self.goals_succeeded} of {self.goals_sent} goals succeeded)')
        else:
            self.get_logger().warn(f'Frontier goal failed (status={result.status})')
        self.state = ExploreState.EXPLORING
        self._goal_handle = None

    # ── VISUALISATION ─────────────────────────────────

    def publish_frontier_markers(self, clusters, best_cluster):
        """Publish cyan spheres for all frontiers, orange for the chosen one."""
        markers = MarkerArray()

        # Clear old markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.ns = 'frontiers'
        markers.markers.append(clear)

        for i, c in enumerate(clusters):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = c['cx']
            m.pose.position.y = c['cy']
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            if c is best_cluster:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 0.8, 0.6
            m.lifetime.sec = 2
            markers.markers.append(m)

        self.frontier_viz_pub.publish(markers)

    def publish_status(self):
        import json
        status = {
            "state":          self.state.value,
            "map_coverage":   round(self.map_coverage * 100, 1),
            "frontiers":      self.frontiers_found,
            "goals_sent":     self.goals_sent,
            "goals_ok":       self.goals_succeeded,
            "robot_pos":      [round(self.robot_x, 2), round(self.robot_y, 2)],
        }
        msg = String(); msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        self.get_logger().info(
            f'[EXPLORE] {self.state.value} | '
            f'Map: {self.map_coverage:.1%} | '
            f'Frontiers: {self.frontiers_found} | '
            f'Goals: {self.goals_succeeded}/{self.goals_sent}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
