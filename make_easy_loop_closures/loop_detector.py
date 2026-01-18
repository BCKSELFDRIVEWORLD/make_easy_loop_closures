# Copyright (c) 2026 BCK - MIT License

import math
from dataclasses import dataclass
from typing import List, Tuple, Set

try:
    from scipy.spatial import KDTree
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


@dataclass
class TFState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class LoopDetector:
    def __init__(self, min_edge_increase: int = 3, proximity_threshold: float = 1.0):
        self.min_edge_increase = min_edge_increase
        self.proximity_threshold = proximity_threshold
        self.last_tf = TFState()
        self.last_edge_set: Set[Tuple[int, int, int, int]] = set()
        self.initialized = False

    def check(
        self,
        tf_x: float,
        tf_y: float,
        yaw: float,
        edge_points: List[Tuple[float, float, float, float]],
        target_area: Tuple[float, float],
        target_radius: float,
        old_nodes: List[Tuple[float, float]],
        robot_x: float,
        robot_y: float,
        logger=None
    ) -> Tuple[bool, float, int]:
        """Check for loop closure by analyzing new edges."""
        if not self.initialized:
            self.last_tf = TFState(tf_x, tf_y, yaw)
            self.last_edge_set = self._edges_to_set(edge_points)
            self.initialized = True
            return False, 0.0, 0

        dx = abs(tf_x - self.last_tf.x)
        dy = abs(tf_y - self.last_tf.y)
        position_change = math.sqrt(dx * dx + dy * dy)

        current_set = self._edges_to_set(edge_points)
        new_edges = self._find_new_edges(edge_points, current_set)
        new_edge_count = len(new_edges)

        detected = False
        if new_edge_count >= self.min_edge_increase and old_nodes:
            # Build KDTree for old nodes
            old_tree = self._build_tree(old_nodes) if old_nodes else None

            if logger:
                logger.info(f'Checking {new_edge_count} new edges against {len(old_nodes)} old nodes')

            for edge in new_edges:
                if self._is_valid_loop_closure(edge, robot_x, robot_y, target_area,
                                                target_radius, old_nodes, old_tree, logger):
                    detected = True
                    break

        self.last_tf = TFState(tf_x, tf_y, yaw)
        self.last_edge_set = current_set

        return detected, position_change, new_edge_count

    def _edges_to_set(self, edges: List[Tuple[float, float, float, float]]) -> Set[Tuple[int, int, int, int]]:
        """Convert edges to set of rounded tuples for fast comparison."""
        return {(int(e[0]*100), int(e[1]*100), int(e[2]*100), int(e[3]*100)) for e in edges}

    def _find_new_edges(
        self,
        current_edges: List[Tuple[float, float, float, float]],
        current_set: Set[Tuple[int, int, int, int]]
    ) -> List[Tuple[float, float, float, float]]:
        """Find edges that weren't in the previous edge set."""
        if not self.last_edge_set:
            return []

        new_set = current_set - self.last_edge_set
        result = []
        for e in current_edges:
            key = (int(e[0]*100), int(e[1]*100), int(e[2]*100), int(e[3]*100))
            if key in new_set:
                result.append(e)
        return result

    def _build_tree(self, nodes: List[Tuple[float, float]]):
        """Build KDTree for spatial queries."""
        if HAS_SCIPY and len(nodes) > 0:
            return KDTree(nodes)
        return None

    def _is_valid_loop_closure(
        self,
        edge: Tuple[float, float, float, float],
        robot_x: float,
        robot_y: float,
        target_area: Tuple[float, float],
        target_radius: float,
        old_nodes: List[Tuple[float, float]],
        old_tree,
        logger=None
    ) -> bool:
        """Check if edge represents a valid loop closure."""
        x1, y1, x2, y2 = edge

        dist1 = math.sqrt((x1 - robot_x) ** 2 + (y1 - robot_y) ** 2)
        dist2 = math.sqrt((x2 - robot_x) ** 2 + (y2 - robot_y) ** 2)

        if dist1 < self.proximity_threshold:
            other_end = (x2, y2)
        elif dist2 < self.proximity_threshold:
            other_end = (x1, y1)
        else:
            return False

        dist_to_target = math.sqrt(
            (other_end[0] - target_area[0]) ** 2 +
            (other_end[1] - target_area[1]) ** 2
        )
        if dist_to_target > target_radius:
            return False

        # Use KDTree for fast proximity check
        if old_tree is not None:
            dist, _ = old_tree.query([other_end[0], other_end[1]])
            if dist < self.proximity_threshold:
                if logger:
                    logger.info(f'VALID LOOP CLOSURE EDGE (dist to old node: {dist:.2f}m)')
                return True
        else:
            # Fallback
            for ox, oy in old_nodes:
                if math.sqrt((other_end[0] - ox) ** 2 + (other_end[1] - oy) ** 2) < self.proximity_threshold:
                    if logger:
                        logger.info('VALID LOOP CLOSURE EDGE')
                    return True

        return False

    def reset(self):
        self.initialized = False
        self.last_edge_set = set()
