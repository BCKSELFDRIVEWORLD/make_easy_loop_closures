#!/usr/bin/env python3
# Copyright (c) 2026 BCK - MIT License
# Make Easy Loop Closures (MELC)

import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from make_easy_loop_closures.loop_detector import LoopDetector
from make_easy_loop_closures.target_finder import TargetFinder
from make_easy_loop_closures.visualizer import Visualizer, State


class LoopClosureAssistant(Node):
    def __init__(self):
        super().__init__('melc')

        self._declare_parameters()
        self._load_config()
        self._fetch_slam_params()  # Get params from SLAM Toolbox
        self._calculate_density_limits()  # Calculate based on radius
        self._log_config()

        self.loop_detector = LoopDetector(
            min_edge_increase=self.min_edge_increase,
            proximity_threshold=self.edge_proximity_threshold
        )
        self.target_finder = TargetFinder(
            min_distance=self.loop_closure_radius,
            max_distance=self.loop_search_dimension * 1.5,
            min_nodes=self.min_graph_nodes,
            min_area_density=self.min_area_density,
            max_area_density=self.max_area_density,
            min_sectors=self.min_sectors
        )
        self.visualizer = Visualizer()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.state = State.FREE
        self.target = None
        self.loop_count = 0

        # SLAM graph data
        self.graph_nodes = []  # [(x, y), ...] all pose nodes from SLAM
        self.edge_points = []  # [(x1,y1,x2,y2), ...] all edges
        self.node_count_at_last_loop = 0  # Track nodes for loop timing
        self.old_nodes = []  # Nodes that existed before entering target area

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
        # MELC specific parameters only
        self.declare_parameter('slam_toolbox_node', 'slam_toolbox')
        self.declare_parameter('nodes_between_loops', 50)
        self.declare_parameter('min_graph_nodes', 20)
        self.declare_parameter('min_edge_increase', 3)
        self.declare_parameter('edge_proximity_threshold', 1.5)
        # Target area density limits (-1 = auto-calculate based on radius)
        self.declare_parameter('min_area_density', -1)
        self.declare_parameter('max_area_density', -1)
        # Density coefficient (k) for auto-calculation: density = k * radius²
        # Default: min_k=0.33, max_k=22 (calibrated for radius=3.0m → min=3, max=200)
        self.declare_parameter('density_k_min', 0.33)
        self.declare_parameter('density_k_max', 22.0)
        # Minimum filled sectors for valid target (out of 6 sectors, each 60°)
        # Prevents selecting corners/edges of mapped area
        self.declare_parameter('min_sectors', 4)

    def _load_config(self):
        # MELC parameters
        self.slam_toolbox_node = self.get_parameter('slam_toolbox_node').value
        self.nodes_between_loops = self.get_parameter('nodes_between_loops').value
        self.min_graph_nodes = self.get_parameter('min_graph_nodes').value
        self.min_edge_increase = self.get_parameter('min_edge_increase').value
        self.edge_proximity_threshold = self.get_parameter('edge_proximity_threshold').value
        self._min_area_density_param = self.get_parameter('min_area_density').value
        self._max_area_density_param = self.get_parameter('max_area_density').value
        self.density_k_min = self.get_parameter('density_k_min').value
        self.density_k_max = self.get_parameter('density_k_max').value
        self.min_sectors = self.get_parameter('min_sectors').value

        # Defaults (will be updated from SLAM Toolbox)
        self.loop_search_max_distance = 3.0
        self.loop_search_dimension = 8.0

    def _calculate_density_limits(self):
        """Calculate density limits based on radius. Use user value if set."""
        r = self.loop_search_max_distance
        r_sq = r * r

        # Auto-calculate based on radius: density = k * radius²
        if self._min_area_density_param < 0:
            self.min_area_density = max(3, int(self.density_k_min * r_sq))
        else:
            self.min_area_density = self._min_area_density_param

        if self._max_area_density_param < 0:
            self.max_area_density = max(20, int(self.density_k_max * r_sq))
        else:
            self.max_area_density = self._max_area_density_param

    def _fetch_slam_params(self):
        """Fetch parameters from SLAM Toolbox node."""
        client = self.create_client(
            GetParameters,
            f'/{self.slam_toolbox_node}/get_parameters'
        )

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f'SLAM Toolbox parameter service not available, using defaults'
            )
            return

        request = GetParameters.Request()
        request.names = ['loop_search_maximum_distance']

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            values = future.result().values
            if len(values) > 0 and values[0].type != 0:  # 0 = NOT_SET
                self.loop_search_max_distance = values[0].double_value
                self.get_logger().info(
                    f'Got from SLAM Toolbox: loop_search_maximum_distance={self.loop_search_max_distance}'
                )
        else:
            self.get_logger().warn('Failed to get SLAM Toolbox parameters')

    @property
    def loop_closure_radius(self) -> float:
        return self.loop_search_max_distance

    def _log_config(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('  Make Easy Loop Closures (MELC)')
        self.get_logger().info('  (c) 2026 BCK - MIT License')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Target radius: {self.loop_closure_radius:.1f}m (from SLAM Toolbox)')
        self.get_logger().info(f'Nodes between loops: {self.nodes_between_loops}')

        # Show if density is auto or user-set
        auto_min = self._min_area_density_param < 0
        auto_max = self._max_area_density_param < 0
        density_src = "auto" if (auto_min and auto_max) else "config"
        self.get_logger().info(f'Area density: {self.min_area_density}-{self.max_area_density} nodes ({density_src})')
        self.get_logger().info(f'Min sectors: {self.min_sectors}/6 (60° each)')

        self.get_logger().info(f'Edge proximity: {self.edge_proximity_threshold:.1f}m')
        self.get_logger().info('=' * 50)

    def _on_odom(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.robot_x = tf.transform.translation.x
            self.robot_y = tf.transform.translation.y
        except Exception:
            pass

    def _on_graph(self, msg):
        """Parse graph visualization and extract nodes and edges."""
        self.edge_points = []
        self.graph_nodes = []

        for marker in msg.markers:
            # Skip delete actions
            if marker.action == Marker.DELETE or marker.action == Marker.DELETEALL:
                continue

            # Extract edges from LINE_LIST
            if marker.type == Marker.LINE_LIST:
                points = marker.points
                for i in range(0, len(points) - 1, 2):
                    p1, p2 = points[i], points[i + 1]
                    self.edge_points.append((p1.x, p1.y, p2.x, p2.y))

            # Extract nodes from individual SPHERE markers (SLAM Toolbox style)
            elif marker.type == Marker.SPHERE:
                x = marker.pose.position.x
                y = marker.pose.position.y
                self.graph_nodes.append((x, y))

            # Also support SPHERE_LIST if used
            elif marker.type == Marker.SPHERE_LIST:
                for p in marker.points:
                    self.graph_nodes.append((p.x, p.y))

    def _check_tf(self):
        if self.state != State.WAITING:
            return

        if not self.target:
            return

        try:
            # map->odom transform for TF change detection
            tf = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            tf_x, tf_y = tf.transform.translation.x, tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z))

            # Use actual robot position (self.robot_x/y) for edge proximity check
            detected, change, new_edges = self.loop_detector.check(
                tf_x, tf_y, yaw,
                self.edge_points,
                self.target,
                self.loop_closure_radius,
                self.old_nodes,  # Nodes in target area from before robot arrived
                self.robot_x,   # Actual robot position in map frame
                self.robot_y,
                self.get_logger()
            )

            self.get_logger().info(
                f'Waiting... TF:{change:.3f}m Edges:{len(self.edge_points)} New:{new_edges} '
                f'OldNodes:{len(self.old_nodes)}'
            )

            if detected:
                self._confirm_loop(change, new_edges)

        except Exception:
            pass

    def _check_loop(self):
        node_count = len(self.graph_nodes)
        nodes_since_loop = node_count - self.node_count_at_last_loop

        if node_count < self.min_graph_nodes:
            self._publish(State.FREE, None, f"Mapping... Nodes: {node_count}/{self.min_graph_nodes}")
            return

        if self.state == State.FREE:
            if nodes_since_loop >= self.nodes_between_loops:
                self.target = self.target_finder.find(
                    self.robot_x, self.robot_y, self.graph_nodes,
                    self.loop_closure_radius, self.edge_points
                )
                if self.target:
                    self.state = State.LOOP_NEEDED
                    self._save_old_nodes()
                    dist = self._target_distance()
                    self.get_logger().warn(
                        f'LOOP CLOSURE NEEDED: Go to red target ({dist:.1f}m) '
                        f'OldNodes:{len(self.old_nodes)}'
                    )

        if self.state == State.LOOP_NEEDED and self.target:
            if self._target_distance() < self.loop_closure_radius:
                self.state = State.WAITING
                self.get_logger().info('Target reached. Waiting for SLAM loop closure...')

        self._publish_state()

    def _save_old_nodes(self):
        """Save nodes in target area that existed before robot arrived."""
        if not self.target:
            self.old_nodes = []
            return

        # Filter nodes within target radius
        self.old_nodes = [
            (nx, ny) for nx, ny in self.graph_nodes
            if math.sqrt((nx - self.target[0]) ** 2 +
                        (ny - self.target[1]) ** 2) < self.loop_closure_radius
        ]

    def _confirm_loop(self, tf_change: float, new_edges: int):
        self.loop_count += 1
        self.node_count_at_last_loop = len(self.graph_nodes)
        self.state = State.FREE
        self.target = None
        self.old_nodes = []
        self.loop_detector.reset()

        self.get_logger().info(
            f'LOOP CLOSURE CONFIRMED! TF:{tf_change:.3f}m NewEdges:{new_edges} '
            f'Total:{self.loop_count}')

    def _target_distance(self) -> float:
        if not self.target:
            return float('inf')
        return math.sqrt((self.robot_x - self.target[0]) ** 2 +
                        (self.robot_y - self.target[1]) ** 2)

    def _publish_state(self):
        nodes_since_loop = len(self.graph_nodes) - self.node_count_at_last_loop

        if self.state == State.FREE:
            pct = min(nodes_since_loop / self.nodes_between_loops * 100, 100)
            text = f"[{pct:.0f}%] Nodes:{len(self.graph_nodes)} | LC:{self.loop_count}"
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
            path=self.graph_nodes,  # Use SLAM graph nodes
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
