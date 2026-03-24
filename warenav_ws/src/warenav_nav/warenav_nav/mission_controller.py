#!/usr/bin/env python3
"""
mission_controller.py
══════════════════════════════════════════════════════════════
WarehouseNav AMR — Mission Controller

Sequences the full warehouse mission:
  Phase 1: EXPLORATION  → frontier_explorer maps the warehouse
  Phase 2: PATROL       → visits all 18 shelf waypoints
  Phase 3: RETURN       → returns to charging dock
  Phase 4: DOCKED       → mission complete

Based on: bhavikmk/warehousebot controller pattern
          AAISHAA1585 Nav2 action interface

Services exposed:
  /start_mission   (std_srvs/Trigger)
  /stop_mission    (std_srvs/Trigger)
  /pause_mission   (std_srvs/Trigger)
  /resume_mission  (std_srvs/Trigger)
  /goto_shelf      (std_srvs/SetBool) → use /nav_cmd topic instead

Topics:
  /mission_status  (String JSON)  ← live status
  /inventory_log   (String JSON)  ← per-shelf records
  /nav_cmd         (String)       ← {"shelf":"B2"} navigation commands

MAHE Mobility Challenge 2026 — Robotics Track
══════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray

import math
import time
import json
import datetime
from enum import Enum


class MissionPhase(Enum):
    IDLE        = "IDLE"
    EXPLORING   = "EXPLORING"    # Phase 1: frontier mapping
    PATROLLING  = "PATROLLING"   # Phase 2: shelf visits
    RECOVERING  = "RECOVERING"   # Retry logic
    RETURNING   = "RETURNING"    # Phase 3: return to dock
    DOCKED      = "DOCKED"       # Complete
    PAUSED      = "PAUSED"
    ABORTED     = "ABORTED"


# ── SHELF WAYPOINTS (match warehouse.world positions) ─────────────
# (x, y, label) — robot navigates 0.7m in front of each shelf face
SHELF_WAYPOINTS = [
    (-7.4, -4.5, "Row-A Shelf-1"),
    (-7.4,  0.0, "Row-A Shelf-2"),
    (-7.4,  4.5, "Row-A Shelf-3"),
    (-5.0, -4.5, "Row-B Shelf-1"),
    (-5.0,  0.0, "Row-B Shelf-2"),
    (-5.0,  4.5, "Row-B Shelf-3"),
    (-2.6, -4.5, "Row-C Shelf-1"),
    (-2.6,  0.0, "Row-C Shelf-2"),
    (-2.6,  4.5, "Row-C Shelf-3"),
    ( 0.2, -4.5, "Row-D Shelf-1"),
    ( 0.2,  0.0, "Row-D Shelf-2"),
    ( 0.2,  4.5, "Row-D Shelf-3"),
    ( 4.2, -4.5, "Row-E Shelf-1"),
    ( 4.2,  0.0, "Row-E Shelf-2"),
    ( 4.2,  4.5, "Row-E Shelf-3"),
    ( 6.6, -4.5, "Row-F Shelf-1"),
    ( 6.6,  0.0, "Row-F Shelf-2"),
    ( 6.6,  4.5, "Row-F Shelf-3"),
]

# Named locations registry (ref: bhavikmk/warehousebot structure)
LOCATION_REGISTRY = {
    **{f"{r}{i}": SHELF_WAYPOINTS[ri*3 + (i-1)]
       for ri, r in enumerate("ABCDEF")
       for i in range(1, 4)},
    "DOCK":    (-9.5, -7.2, "Charging Dock"),
    "PACKING": (-9.0,  6.5, "Packing Station"),
}


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')
        self.cbg = ReentrantCallbackGroup()

        # ── PARAMETERS ────────────────────────────────
        self.declare_parameter('auto_start',         False)
        self.declare_parameter('waypoint_pause_sec', 2.0)
        self.declare_parameter('max_retries',        3)
        self.declare_parameter('skip_exploration',   False)

        self.auto_start      = self.get_parameter('auto_start').value
        self.wp_pause        = self.get_parameter('waypoint_pause_sec').value
        self.max_retries     = self.get_parameter('max_retries').value
        self.skip_explore    = self.get_parameter('skip_exploration').value

        # ── NAV2 CLIENT ───────────────────────────────
        self._nav = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cbg)

        # ── PUBLISHERS ────────────────────────────────
        self.status_pub  = self.create_publisher(String, '/mission_status', 10)
        self.inv_pub     = self.create_publisher(String, '/inventory_log',  10)
        self.wp_viz_pub  = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # ── SUBSCRIBERS ───────────────────────────────
        self.explore_done_sub = self.create_subscription(
            Bool, '/exploration_complete', self.explore_done_cb, 10,
            callback_group=self.cbg)
        self.nav_cmd_sub = self.create_subscription(
            String, '/nav_cmd', self.nav_cmd_cb, 10,
            callback_group=self.cbg)

        # ── SERVICES ──────────────────────────────────
        self.start_srv  = self.create_service(Trigger, '/start_mission',  self.start_cb)
        self.stop_srv   = self.create_service(Trigger, '/stop_mission',   self.stop_cb)
        self.pause_srv  = self.create_service(Trigger, '/pause_mission',  self.pause_cb)
        self.resume_srv = self.create_service(Trigger, '/resume_mission', self.resume_cb)
        self.status_srv = self.create_service(Trigger, '/mission_report', self.report_cb)

        # ── STATE ─────────────────────────────────────
        self.phase           = MissionPhase.IDLE
        self.wp_index        = 0
        self.retry_count     = 0
        self.start_time      = None
        self.completed_wps   = []
        self.failed_wps      = []
        self.inventory       = []
        self._goal_handle    = None
        self._navigating     = False

        # ── TIMERS ────────────────────────────────────
        self.status_timer = self.create_timer(3.0, self.publish_status)
        self.viz_timer    = self.create_timer(5.0, self.publish_waypoint_markers)

        if self.auto_start:
            self.get_logger().info('auto_start=true — waiting 20s for Nav2...')
            self.auto_timer = self.create_timer(20.0, self.auto_start_cb)

        self.get_logger().info('═' * 55)
        self.get_logger().info('WarehouseNav AMR — Mission Controller')
        self.get_logger().info(f'  Waypoints: {len(SHELF_WAYPOINTS)} shelves (Rows A-F)')
        self.get_logger().info(f'  Pause per shelf: {self.wp_pause}s')
        self.get_logger().info(f'  Max retries: {self.max_retries}')
        self.get_logger().info('  Call: ros2 service call /start_mission std_srvs/srv/Trigger')
        self.get_logger().info('═' * 55)

    # ── SERVICE CALLBACKS ─────────────────────────────

    def start_cb(self, req, res):
        if self.phase not in (MissionPhase.IDLE, MissionPhase.DOCKED,
                               MissionPhase.ABORTED):
            res.success = False
            res.message = f'Already running: {self.phase.value}'
            return res
        self.begin_mission()
        res.success = True
        res.message = 'Mission started'
        return res

    def stop_cb(self, req, res):
        self.get_logger().warn('Mission ABORTED by operator')
        self.phase = MissionPhase.ABORTED
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
        res.success = True
        res.message = 'Mission aborted'
        return res

    def pause_cb(self, req, res):
        if self.phase == MissionPhase.PATROLLING:
            self.phase = MissionPhase.PAUSED
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            res.success = True
            res.message = 'Mission paused'
        else:
            res.success = False
            res.message = f'Cannot pause from state: {self.phase.value}'
        return res

    def resume_cb(self, req, res):
        if self.phase == MissionPhase.PAUSED:
            self.phase = MissionPhase.PATROLLING
            self.navigate_to_waypoint(self.wp_index)
            res.success = True
            res.message = 'Mission resumed'
        else:
            res.success = False
            res.message = f'Not paused: {self.phase.value}'
        return res

    def report_cb(self, req, res):
        report = {
            "phase":      self.phase.value,
            "duration_s": round(time.time() - self.start_time, 1) if self.start_time else 0,
            "completed":  self.completed_wps,
            "failed":     self.failed_wps,
            "inventory":  len(self.inventory),
        }
        res.success = True
        res.message = json.dumps(report, indent=2)
        return res

    def auto_start_cb(self):
        self.auto_timer.cancel()
        self.begin_mission()

    # ── TOPIC CALLBACKS ───────────────────────────────

    def explore_done_cb(self, msg: Bool):
        if msg.data and self.phase == MissionPhase.EXPLORING:
            self.get_logger().info('Exploration complete — starting shelf patrol')
            self.phase = MissionPhase.PATROLLING
            self.wp_index = 0
            self.navigate_to_waypoint(0)

    def nav_cmd_cb(self, msg: String):
        """Accept direct navigation commands: {"shelf":"B2"}"""
        try:
            cmd = json.loads(msg.data)
            loc_key = cmd.get('shelf', cmd.get('location', '')).upper()
            if loc_key in LOCATION_REGISTRY:
                wp = LOCATION_REGISTRY[loc_key]
                self.get_logger().info(f'Direct nav command: {loc_key} → ({wp[0]}, {wp[1]})')
                self.send_nav_goal(wp[0], wp[1], 0.0)
            else:
                self.get_logger().error(
                    f'Unknown location: {loc_key}. Available: {list(LOCATION_REGISTRY.keys())}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid nav_cmd JSON: {msg.data}')

    # ── MISSION LOGIC ─────────────────────────────────

    def begin_mission(self):
        """Initialise state and start Phase 1 or Phase 2."""
        self.wp_index      = 0
        self.retry_count   = 0
        self.start_time    = time.time()
        self.completed_wps = []
        self.failed_wps    = []
        self.inventory     = []

        if not self._nav.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 not available after 15s!')
            return

        if self.skip_explore:
            self.get_logger().info('Skipping exploration — going direct to patrol')
            self.phase = MissionPhase.PATROLLING
            self.navigate_to_waypoint(0)
        else:
            self.phase = MissionPhase.EXPLORING
            self.get_logger().info(
                'Phase 1: Exploration (frontier_explorer will map the warehouse)')

    def navigate_to_waypoint(self, idx: int):
        """Send navigation goal for waypoint[idx]."""
        if idx >= len(SHELF_WAYPOINTS):
            self.get_logger().info('All shelves visited — returning to dock')
            self.phase = MissionPhase.RETURNING
            dock = LOCATION_REGISTRY['DOCK']
            self.send_nav_goal(dock[0], dock[1], 0.0,
                               callback=self.dock_result_cb)
            return

        x, y, label = SHELF_WAYPOINTS[idx]
        self.get_logger().info(
            f'[{idx+1}/{len(SHELF_WAYPOINTS)}] Navigating to: {label}')
        self.send_nav_goal(x, y, 0.0,
                           callback=lambda f: self.waypoint_result_cb(f, idx, label))

    def send_nav_goal(self, x, y, yaw, callback=None):
        """Build and send a NavigateToPose goal."""
        if not self._nav.server_is_ready():
            self.get_logger().error('Nav2 not ready for goal')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        future = self._nav.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )
        if callback:
            future.add_done_callback(
                lambda f: self._on_goal_accepted(f, callback))
        else:
            future.add_done_callback(self._default_goal_accepted)

    def _on_goal_accepted(self, future, result_callback):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(result_callback)

    def _default_goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2')
            return
        self._goal_handle = handle

    def feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        if dist > 1.0:
            self.get_logger().debug(f'Distance remaining: {dist:.1f}m')

    def waypoint_result_cb(self, future, idx, label):
        """Called when navigation to a shelf waypoint completes."""
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.retry_count = 0
            self.completed_wps.append(label)
            self.get_logger().info(f'✓ REACHED: {label}')
            self.log_inventory(SHELF_WAYPOINTS[idx], label)
            time.sleep(self.wp_pause)
            self.wp_index = idx + 1
            self.navigate_to_waypoint(self.wp_index)
        else:
            self.get_logger().warn(f'✗ FAILED: {label} (status={result.status})')
            self.handle_failure(idx, label)

    def dock_result_cb(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            elapsed = time.time() - self.start_time
            self.phase = MissionPhase.DOCKED
            self.get_logger().info('═' * 55)
            self.get_logger().info('MISSION COMPLETE ✓')
            self.get_logger().info(f'  Duration: {elapsed:.0f}s ({elapsed/60:.1f} min)')
            self.get_logger().info(f'  Completed: {len(self.completed_wps)}/{len(SHELF_WAYPOINTS)}')
            self.get_logger().info(f'  Failed: {len(self.failed_wps)} waypoints')
            self.get_logger().info('═' * 55)
            self.save_log()
        else:
            self.get_logger().error('Failed to return to dock')
            self.phase = MissionPhase.ABORTED

    def handle_failure(self, idx, label):
        self.retry_count += 1
        self.phase = MissionPhase.RECOVERING
        if self.retry_count <= self.max_retries:
            self.get_logger().warn(
                f'Retry {self.retry_count}/{self.max_retries}: {label}')
            time.sleep(2.0)
            self.phase = MissionPhase.PATROLLING
            self.navigate_to_waypoint(idx)
        else:
            self.get_logger().error(f'Max retries exceeded — skipping: {label}')
            self.failed_wps.append(label)
            self.retry_count = 0
            self.wp_index = idx + 1
            self.phase = MissionPhase.PATROLLING
            self.navigate_to_waypoint(self.wp_index)

    # ── INVENTORY LOGGING ─────────────────────────────

    def log_inventory(self, wp, label):
        entry = {
            "ts":       datetime.datetime.now().isoformat(),
            "shelf":    label,
            "x":        wp[0],
            "y":        wp[1],
            "status":   "scanned",
        }
        self.inventory.append(entry)
        msg = String(); msg.data = json.dumps(entry)
        self.inv_pub.publish(msg)

    def save_log(self):
        data = {
            "date":       datetime.datetime.now().isoformat(),
            "duration_s": round(time.time() - self.start_time, 1),
            "completed":  self.completed_wps,
            "failed":     self.failed_wps,
            "inventory":  self.inventory,
        }
        path = f'/tmp/mission_{int(time.time())}.json'
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f'Mission log saved: {path}')

    # ── VISUALISATION ─────────────────────────────────

    def publish_waypoint_markers(self):
        markers = MarkerArray()
        # Clear first
        clear = Marker(); clear.action = Marker.DELETEALL
        clear.ns = 'waypoints'; markers.markers.append(clear)

        for i, (x, y, label) in enumerate(SHELF_WAYPOINTS):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'waypoints'; m.id = i
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y
            m.pose.position.z = 0.1; m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.3; m.scale.z = 0.2

            done = any(label in s for s in self.completed_wps)
            fail = any(label in s for s in self.failed_wps)
            current = (self.wp_index < len(SHELF_WAYPOINTS) and
                       SHELF_WAYPOINTS[self.wp_index][2] == label)

            if done:   m.color.r,m.color.g,m.color.b = 0.0, 0.9, 0.0
            elif fail: m.color.r,m.color.g,m.color.b = 0.9, 0.0, 0.0
            elif current: m.color.r,m.color.g,m.color.b = 1.0, 0.6, 0.0
            else:      m.color.r,m.color.g,m.color.b = 0.2, 0.5, 1.0
            m.color.a = 0.85

            # Label
            t = Marker()
            t.header = m.header; t.ns = 'wp_labels'; t.id = i + 100
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = x; t.pose.position.y = y
            t.pose.position.z = 0.55; t.pose.orientation.w = 1.0
            t.scale.z = 0.22; t.color.r=t.color.g=t.color.b=t.color.a=1.0
            t.text = label.replace("Row-", "").replace(" Shelf-", "")

            markers.markers.append(m); markers.markers.append(t)

        # Dock marker
        dm = Marker()
        dm.header.frame_id = 'map'
        dm.header.stamp = self.get_clock().now().to_msg()
        dm.ns = 'waypoints'; dm.id = 200
        dm.type = Marker.CUBE; dm.action = Marker.ADD
        dm.pose.position.x = -9.5; dm.pose.position.y = -7.2
        dm.pose.position.z = 0.05; dm.pose.orientation.w = 1.0
        dm.scale.x=0.6; dm.scale.y=0.6; dm.scale.z=0.1
        dm.color.r=1.0; dm.color.g=0.65; dm.color.b=0.0; dm.color.a=0.9
        markers.markers.append(dm)

        self.wp_viz_pub.publish(markers)

    def publish_status(self):
        status = {
            "phase":     self.phase.value,
            "wp":        self.wp_index,
            "total_wps": len(SHELF_WAYPOINTS),
            "done":      len(self.completed_wps),
            "failed":    len(self.failed_wps),
            "elapsed":   round(time.time() - self.start_time, 0) if self.start_time else 0,
        }
        msg = String(); msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        self.get_logger().info(
            f'[MISSION] {self.phase.value} | '
            f'WP {self.wp_index}/{len(SHELF_WAYPOINTS)} | '
            f'Done: {len(self.completed_wps)} | '
            f'Failed: {len(self.failed_wps)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
