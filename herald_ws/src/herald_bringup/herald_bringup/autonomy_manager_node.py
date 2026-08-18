#!/usr/bin/env python3
import csv
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose


LOG_PATH = os.path.expanduser('~/herald_autonomy_log.csv')
VALID_MODES = ('explore_only', 'navigate_only', 'explore_then_navigate')


class AutonomyManagerNode(Node):
    def __init__(self):
        super().__init__('autonomy_manager_node')

        self.declare_parameter('explore_timeout_s', 300.0)
        self.declare_parameter('min_frontier_size_cells', 8)
        self.declare_parameter('nav_goal_timeout_s', 60.0)
        self.declare_parameter('default_mode', 'explore_then_navigate')
        self.declare_parameter('frontier_blacklist_radius_m', 0.5)
        self.declare_parameter('frontier_blacklist_cooldown_s', 30.0)

        self.explore_timeout_s = self.get_parameter('explore_timeout_s').value
        self.min_frontier_size = self.get_parameter('min_frontier_size_cells').value
        self.nav_goal_timeout_s = self.get_parameter('nav_goal_timeout_s').value
        self.frontier_blacklist_radius_m = self.get_parameter('frontier_blacklist_radius_m').value
        self.frontier_blacklist_cooldown_s = self.get_parameter('frontier_blacklist_cooldown_s').value
        self.mode = self.get_parameter('default_mode').value
        if self.mode not in VALID_MODES:
            self.get_logger().warn(f'Invalid default_mode "{self.mode}", using explore_then_navigate')
            self.mode = 'explore_then_navigate'

        self.state = 'IDLE'
        self.latest_map = None
        self.explore_start_time = None
        self._nav_active_goal_handle = None
        self._state_before_override = 'IDLE'
        self._frontier_blacklist = []
        self._nav_server_ready = False

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)

        self.command_sub = self.create_subscription(
            String, '/autonomy/command', self._command_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/autonomy/goal_pose', self._goal_pose_callback, 10)

        self.state_pub = self.create_publisher(String, '/autonomy/state', 10)
        self.mode_pub = self.create_publisher(String, '/autonomy/mode', 10)
        self.override_lock_pub = self.create_publisher(Bool, '/manual_override_lock', 10)
        self.frontier_target_pub = self.create_publisher(
            PoseStamped, '/autonomy/current_frontier_target', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._init_log()

        self.explore_timer = self.create_timer(2.0, self._explore_tick)
        self.nav_server_check_timer = self.create_timer(0.5, self._check_nav_server_ready)
        self._publish_state()
        self._publish_mode()
        self._publish_override_lock(False)

        self.get_logger().info(
            f'HERALD autonomy_manager_node ready. Mode={self.mode}. '
            'Publish "start_explore" or "start_navigate" to /autonomy/command to begin, '
            'or "manual_override_on" to take manual control at any time.')

    def _init_log(self):
        if not os.path.exists(LOG_PATH):
            with open(LOG_PATH, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'event', 'detail'])

    def _log(self, event: str, detail: str = ''):
        with open(LOG_PATH, 'a', newline='') as f:
            csv.writer(f).writerow([time.time(), event, detail])

    def _set_state(self, new_state: str):
        self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        self._publish_state()
        self._log('state_change', new_state)

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def _publish_override_lock(self, locked: bool):
        msg = Bool()
        msg.data = locked
        self.override_lock_pub.publish(msg)

    def _command_callback(self, msg: String):
        cmd = msg.data.strip()

        if cmd.startswith('set_mode:'):
            new_mode = cmd.split(':', 1)[1]
            if new_mode in VALID_MODES:
                self.mode = new_mode
                self._publish_mode()
                self._log('mode_change', new_mode)
                self.get_logger().info(f'Mode set to {new_mode}')
            else:
                self.get_logger().warn(f'Unknown mode "{new_mode}"')
            return

        if cmd == 'manual_override_on':
            self._enter_manual_override()
            return

        if cmd == 'manual_override_off':
            self._exit_manual_override()
            return

        if self.state == 'MANUAL_OVERRIDE':
            self.get_logger().warn(
                f'Ignoring "{cmd}" while in MANUAL_OVERRIDE -- send manual_override_off first.')
            return

        if cmd == 'start_explore' and self.mode in ('explore_only', 'explore_then_navigate') \
                and self.state in ('IDLE', 'EXPLORE_DONE'):
            self.explore_start_time = time.time()
            self._set_state('EXPLORING')
            self._log('command', 'start_explore')

        elif cmd == 'start_navigate' and self.mode == 'navigate_only' \
                and self.state in ('IDLE', 'EXPLORE_DONE'):
            self._set_state('AWAITING_GOAL')
            self._log('command', 'start_navigate')

        elif cmd == 'cancel':
            self._cancel_current_nav_goal()
            self._set_state('IDLE')
            self._log('command', 'cancel')

        else:
            self.get_logger().warn(
                f'Ignoring command "{cmd}" (mode={self.mode}, state={self.state})')

    def _enter_manual_override(self):
        if self.state == 'MANUAL_OVERRIDE':
            return
        self._cancel_current_nav_goal()
        self._state_before_override = self.state
        self._set_state('MANUAL_OVERRIDE')
        self._publish_override_lock(True)
        self._log('manual_override', 'on')
        self.get_logger().warn('MANUAL OVERRIDE engaged -- autonomy paused, teleop has full control.')

    def _exit_manual_override(self):
        if self.state != 'MANUAL_OVERRIDE':
            return
        self._publish_override_lock(False)
        if self._state_before_override in ('AWAITING_GOAL', 'EXPLORE_DONE'):
            restored = 'AWAITING_GOAL'
        else:
            restored = 'IDLE'
        self._set_state(restored)
        self._log('manual_override', 'off')
        self.get_logger().info(f'Manual override released -- back to {restored}.')

    def _goal_pose_callback(self, msg: PoseStamped):
        if self.state != 'AWAITING_GOAL':
            self.get_logger().warn(
                f'Got goal pose but not in AWAITING_GOAL state (currently {self.state}); ignoring.')
            return
        self._send_nav_goal(msg, on_reach_state='GOAL_REACHED', on_fail_state='GOAL_FAILED')
        self._set_state('NAVIGATING')

    def _map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _explore_tick(self):
        if self.state != 'EXPLORING':
            return

        if self.explore_start_time and \
                (time.time() - self.explore_start_time) > self.explore_timeout_s:
            self.get_logger().warn('Explore timeout reached.')
            self._finish_exploration()
            return

        if self.latest_map is None:
            return

        if self._nav_active_goal_handle is not None:
            return

        frontier = self._find_best_frontier(self.latest_map)
        if frontier is None:
            self.get_logger().info('No frontiers left -- exploration complete.')
            self._finish_exploration()
            return

        goal_pose = self._frontier_to_pose(frontier, self.latest_map)
        self.frontier_target_pub.publish(goal_pose)
        self._send_nav_goal(
            goal_pose,
            on_reach_state=None,   # stay in EXPLORING, tick will pick next frontier
            on_fail_state=None,
        )

    def _finish_exploration(self):
        self._cancel_current_nav_goal()
        self._set_state('EXPLORE_DONE')
        self._log('exploration_complete', '')
        if self.mode == 'explore_then_navigate':
            self._set_state('AWAITING_GOAL')
        else:  # explore_only
            self._set_state('IDLE')

    def _find_best_frontier(self, grid_msg: OccupancyGrid):
        """Grid-based frontier detection: free cells adjacent to unknown cells,
        clustered via BFS, filtered by min size, closest cluster centroid wins.
        Skips clusters whose world position was recently blacklisted (Nav2
        rejected or failed to reach it during exploration -- see
        _blacklist_frontier)."""
        width = grid_msg.info.width
        height = grid_msg.info.height
        data = np.array(grid_msg.data, dtype=np.int16).reshape((height, width))

        FREE_THRESHOLD = 40   # occupancy < this = considered free (nav2 default free ~0)
        UNKNOWN = -1

        frontier_mask = np.zeros_like(data, dtype=bool)
        free_mask = (data >= 0) & (data < FREE_THRESHOLD)

        unknown_mask = (data == UNKNOWN)
        shifted_up = np.roll(unknown_mask, -1, axis=0)
        shifted_down = np.roll(unknown_mask, 1, axis=0)
        shifted_left = np.roll(unknown_mask, -1, axis=1)
        shifted_right = np.roll(unknown_mask, 1, axis=1)
        adjacent_unknown = shifted_up | shifted_down | shifted_left | shifted_right
        frontier_mask = free_mask & adjacent_unknown

        frontier_mask[0, :] = False
        frontier_mask[-1, :] = False
        frontier_mask[:, 0] = False
        frontier_mask[:, -1] = False

        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters = []

        ys, xs = np.where(frontier_mask)
        for y0, x0 in zip(ys, xs):
            if visited[y0, x0]:
                continue
            cluster_cells = []
            q = deque([(y0, x0)])
            visited[y0, x0] = True
            while q:
                y, x = q.popleft()
                cluster_cells.append((y, x))
                for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        if frontier_mask[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = True
                            q.append((ny, nx))
            if len(cluster_cells) >= self.min_frontier_size:
                clusters.append(cluster_cells)

        if not clusters:
            return None

        res = grid_msg.info.resolution
        origin = grid_msg.info.origin
        candidates = []
        for cluster in clusters:
            cy = sum(c[0] for c in cluster) / len(cluster)
            cx = sum(c[1] for c in cluster) / len(cluster)
            world_x = origin.position.x + (cx + 0.5) * res
            world_y = origin.position.y + (cy + 0.5) * res
            if not self._is_frontier_blacklisted(world_x, world_y):
                candidates.append((cx, cy, len(cluster)))

        if not candidates:
            return None

        cx, cy, _ = max(candidates, key=lambda c: c[2])
        return (cx, cy)

    def _is_frontier_blacklisted(self, world_x: float, world_y: float) -> bool:
        now = time.time()
        self._frontier_blacklist = [
            (x, y, t) for (x, y, t) in self._frontier_blacklist
            if now - t < self.frontier_blacklist_cooldown_s
        ]
        for bx, by, _t in self._frontier_blacklist:
            if math.hypot(world_x - bx, world_y - by) < self.frontier_blacklist_radius_m:
                return True
        return False

    def _blacklist_frontier(self, pose: PoseStamped):
        self._frontier_blacklist.append(
            (pose.pose.position.x, pose.pose.position.y, time.time()))

    def _frontier_to_pose(self, frontier_cell, grid_msg: OccupancyGrid) -> PoseStamped:
        cx, cy = frontier_cell
        res = grid_msg.info.resolution
        origin = grid_msg.info.origin

        world_x = origin.position.x + (cx + 0.5) * res
        world_y = origin.position.y + (cy + 0.5) * res

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = world_x
        pose.pose.position.y = world_y
        pose.pose.orientation.w = 1.0  # heading doesn't matter for frontier goals
        return pose

    def _check_nav_server_ready(self):
        ready = self.nav_client.wait_for_server(timeout_sec=0.0)
        if ready and not self._nav_server_ready:
            self.get_logger().info('navigate_to_pose action server is up.')
        self._nav_server_ready = ready

    def _send_nav_goal(self, pose: PoseStamped, on_reach_state, on_fail_state):
        if not self._nav_server_ready:
            self.get_logger().warn('navigate_to_pose action server not available yet -- will retry.')
            if on_fail_state:
                self._set_state(on_fail_state)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda fut: self._on_goal_response(fut, pose, on_reach_state, on_fail_state))

    def _on_goal_response(self, future, pose, on_reach_state, on_fail_state):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            if on_fail_state:
                self._set_state(on_fail_state)
            else:
                self._blacklist_frontier(pose)
            self._nav_active_goal_handle = None
            return

        self._nav_active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self._on_goal_result(fut, pose, on_reach_state, on_fail_state))

    def _on_goal_result(self, future, pose, on_reach_state, on_fail_state):
        self._nav_active_goal_handle = None
        status = future.result().status
        succeeded = (status == 4)

        self._log('nav_goal_result', f'succeeded={succeeded} status={status}')

        if succeeded and on_reach_state:
            self._set_state(on_reach_state)
        elif not succeeded and on_fail_state:
            self._set_state(on_fail_state)
        elif not succeeded and on_fail_state is None and on_reach_state is None:
            self._blacklist_frontier(pose)

    def _cancel_current_nav_goal(self):
        if self._nav_active_goal_handle is not None:
            self._nav_active_goal_handle.cancel_goal_async()
            self._nav_active_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
