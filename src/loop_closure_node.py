#!/usr/bin/env python3
# Copyright (c) 2026 BCK - MIT License
# Make Easy Loop Closures (MELC)

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from make_easy_loop_closures.path_tracker import PathTracker
from make_easy_loop_closures.loop_detector import LoopDetector
from make_easy_loop_closures.target_finder import TargetFinder
from make_easy_loop_closures.visualizer import Visualizer, State


class LoopClosureAssistant(Node):
    def __init__(self):
        super().__init__('melc')

        self._declare_parameters()
        self._load_config()
        self._log_config()

        self.path_tracker = PathTracker(
            min_travel_distance=self.minimum_travel_distance,
            max_history=5000
        )
        self.loop_detector = LoopDetector(
            min_edge_increase=self.min_edge_increase
        )
        self.target_finder = TargetFinder(
            min_distance=self.loop_closure_radius,
            max_distance=self.loop_search_dimension * 1.5,
            min_points=self.min_path_points,
            min_pose_age=self.min_pose_age
        )
        self.visualizer = Visualizer()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.state = State.FREE
        self.target = None
        self.loop_count = 0
        self.edge_count = 0

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)
        self.graph_sub = self.create_subscription(
            MarkerArray, '/slam_toolbox/graph_visualization',
            self._on_graph, 10)

        self.viz_pub = self.create_publisher(
            MarkerArray, '/loop_closure_assistant/markers', 10)
        self.status_pub = self.create_publisher(
            String, '/loop_closure_assistant/status', 10)

        self.create_timer(1.0, self._check_loop)
        self.create_timer(0.5, self._check_tf)

    def _declare_parameters(self):
        self.declare_parameter('minimum_travel_distance', 0.5)
        self.declare_parameter('loop_search_max_distance', 3.0)
        self.declare_parameter('loop_search_dimension', 8.0)
        self.declare_parameter('loop_chain_size', 10)
        self.declare_parameter('min_edge_increase', 3)
        self.declare_parameter('min_pose_age', 20)

    def _load_config(self):
        self.minimum_travel_distance = self.get_parameter('minimum_travel_distance').value
        self.loop_search_max_distance = self.get_parameter('loop_search_max_distance').value
        self.loop_search_dimension = self.get_parameter('loop_search_dimension').value
        self.loop_chain_size = self.get_parameter('loop_chain_size').value
        self.min_edge_increase = self.get_parameter('min_edge_increase').value
        self.min_pose_age = self.get_parameter('min_pose_age').value

    @property
    def loop_closure_distance(self) -> float:
        return self.loop_search_dimension * 3

    @property
    def loop_closure_radius(self) -> float:
        return self.loop_search_max_distance

    @property
    def min_path_points(self) -> int:
        return self.loop_chain_size * 2

    def _log_config(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('  Make Easy Loop Closures (MELC)')
        self.get_logger().info('  (c) 2026 BCK - MIT License')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Loop distance: {self.loop_closure_distance:.1f}m')
        self.get_logger().info(f'Target radius: {self.loop_closure_radius:.1f}m')
        self.get_logger().info('=' * 50)

    def _on_odom(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.robot_x = tf.transform.translation.x
            self.robot_y = tf.transform.translation.y
            self.path_tracker.update(self.robot_x, self.robot_y)
        except Exception:
            pass

    def _on_graph(self, msg):
        edges = [m for m in msg.markers if m.type == Marker.LINE_LIST]
        self.edge_count = sum(len(m.points) // 2 for m in edges) if edges else 0

    def _check_tf(self):
        if self.state != State.WAITING:
            return

        try:
            tf = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            x, y = tf.transform.translation.x, tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z))

            detected, change, edge_inc = self.loop_detector.check(x, y, yaw, self.edge_count)

            self.get_logger().info(f'Waiting... TF:{change:.3f}m Edges:{self.edge_count} (+{edge_inc})')

            if detected:
                self._confirm_loop(change, edge_inc)

        except Exception:
            pass

    def _check_loop(self):
        path = self.path_tracker.get_path_list()

        if self.path_tracker.point_count < self.min_path_points:
            self._publish(State.FREE, None, "Mapping started... Explore freely.")
            return

        if self.state == State.FREE:
            if self.path_tracker.distance_since_loop >= self.loop_closure_distance:
                self.target = self.target_finder.find(self.robot_x, self.robot_y, path)
                if self.target:
                    self.state = State.LOOP_NEEDED
                    dist = self._target_distance()
                    self.get_logger().warn(
                        f'LOOP CLOSURE NEEDED: Go to red target ({dist:.1f}m)')

        if self.state == State.LOOP_NEEDED and self.target:
            if self._target_distance() < self.loop_closure_radius:
                self.state = State.WAITING
                self.get_logger().info('Target reached. Waiting for SLAM loop closure...')

        self._publish_state()

    def _confirm_loop(self, tf_change: float, edge_inc: int):
        self.loop_count += 1
        self.path_tracker.reset_loop_distance()
        self.state = State.FREE
        self.target = None
        self.loop_detector.reset()

        self.get_logger().info(
            f'LOOP CLOSURE CONFIRMED! TF:{tf_change:.3f}m Edges:+{edge_inc} '
            f'Total:{self.loop_count}')

    def _target_distance(self) -> float:
        if not self.target:
            return float('inf')
        return math.sqrt((self.robot_x - self.target[0]) ** 2 +
                        (self.robot_y - self.target[1]) ** 2)

    def _publish_state(self):
        if self.state == State.FREE:
            pct = min(self.path_tracker.distance_since_loop /
                     self.loop_closure_distance * 100, 100)
            text = f"[{pct:.0f}%] {self.path_tracker.total_distance:.1f}m | LC:{self.loop_count}"
        elif self.state == State.WAITING:
            text = "Waiting for loop closure..."
        else:
            text = f"LOOP CLOSURE! {self._target_distance():.1f}m"

        self._publish(self.state, self.target, text)

    def _publish(self, state: State, target, text: str):
        stamp = self.get_clock().now().to_msg()

        markers = self.visualizer.create_markers(
            stamp=stamp,
            state=state,
            path=self.path_tracker.get_path_list(),
            robot_pos=(self.robot_x, self.robot_y),
            target=target,
            target_radius=self.loop_closure_radius,
            status_text=text,
            target_distance=self._target_distance() if target else 0
        )
        self.viz_pub.publish(markers)

        status = String()
        status.data = text
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureAssistant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
