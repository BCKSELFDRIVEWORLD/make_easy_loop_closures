# Copyright (c) 2026 BCK - MIT License

import math
from typing import List, Tuple, Optional

try:
    from scipy.spatial import KDTree
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class TargetFinder:
    """Find target location for loop closure using SLAM graph nodes."""

    def __init__(self, min_distance: float, max_distance: float, min_nodes: int,
                 min_area_density: int = 3, max_area_density: int = 200,
                 min_sectors: int = 4):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.min_nodes = min_nodes
        self.min_area_density = min_area_density
        self.max_area_density = max_area_density
        self.min_sectors = min_sectors
        self._kdtree = None
        self._cached_nodes = None
        self._edge_counts = {}  # node_idx -> edge count

    def find(self, robot_x: float, robot_y: float,
             nodes: List[Tuple[float, float]],
             target_radius: float,
             edges: List[Tuple[float, float, float, float]] = None) -> Optional[Tuple[float, float]]:
        """Find best target location from SLAM graph nodes."""
        if len(nodes) < self.min_nodes:
            return None

        self._build_tree(nodes)

        # Build edge count map for feature scoring
        if edges:
            self._build_edge_counts(nodes, edges)

        candidates = []
        candidate_indices = self._query_range(robot_x, robot_y,
                                               self.min_distance,
                                               self.max_distance)

        for i in candidate_indices:
            nx, ny = nodes[i]
            dist = math.sqrt((nx - robot_x) ** 2 + (ny - robot_y) ** 2)

            area_density, filled_sectors, area_edge_score = self._analyze_area_fast(
                nx, ny, target_radius
            )

            if area_density < self.min_area_density:
                continue
            if area_density > self.max_area_density:
                continue
            if filled_sectors < self.min_sectors:
                continue

            # Distance score (closer is better)
            dist_score = 1.0 - (dist / self.max_distance)

            # Density score (optimal density is best)
            optimal_density = (self.min_area_density + self.max_area_density) / 2
            density_diff = abs(area_density - optimal_density)
            density_range = (self.max_area_density - self.min_area_density) / 2
            density_score = 1.0 - (density_diff / density_range) if density_range > 0 else 0.5

            # Distribution score (more sectors is better)
            distribution_score = filled_sectors / 6.0

            # Feature score from edges (more edges = feature rich)
            feature_score = area_edge_score

            # Combined score with feature richness
            score = (dist_score * 0.2 +
                    density_score * 0.3 +
                    distribution_score * 0.2 +
                    feature_score * 0.3)

            candidates.append((nx, ny, score))

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[2], reverse=True)
        return (candidates[0][0], candidates[0][1])

    def _build_tree(self, nodes: List[Tuple[float, float]]):
        """Build or update KDTree for spatial queries."""
        if self._cached_nodes == nodes:
            return

        self._cached_nodes = nodes
        self._edge_counts = {}
        if HAS_SCIPY and len(nodes) > 0:
            self._kdtree = KDTree(nodes)
        else:
            self._kdtree = None

    def _build_edge_counts(self, nodes: List[Tuple[float, float]],
                           edges: List[Tuple[float, float, float, float]]):
        """Count edges connected to each node for feature scoring."""
        if not edges or self._kdtree is None:
            return

        self._edge_counts = {i: 0 for i in range(len(nodes))}

        for x1, y1, x2, y2 in edges:
            # Find nearest node to each edge endpoint
            _, idx1 = self._kdtree.query([x1, y1])
            _, idx2 = self._kdtree.query([x2, y2])

            self._edge_counts[idx1] = self._edge_counts.get(idx1, 0) + 1
            if idx1 != idx2:
                self._edge_counts[idx2] = self._edge_counts.get(idx2, 0) + 1

    def _query_range(self, cx: float, cy: float,
                     min_r: float, max_r: float) -> List[int]:
        """Find node indices within distance range from center."""
        if self._kdtree is not None:
            inner = set(self._kdtree.query_ball_point([cx, cy], min_r))
            outer = set(self._kdtree.query_ball_point([cx, cy], max_r))
            return list(outer - inner)
        else:
            # Fallback without scipy
            result = []
            min_sq, max_sq = min_r ** 2, max_r ** 2
            for i, (nx, ny) in enumerate(self._cached_nodes):
                d_sq = (nx - cx) ** 2 + (ny - cy) ** 2
                if min_sq < d_sq < max_sq:
                    result.append(i)
            return result

    def _analyze_area_fast(self, cx: float, cy: float,
                           radius: float) -> Tuple[int, int, float]:
        """Analyze area using KDTree for O(log n) queries.

        Returns:
            (node_count, filled_sectors, edge_score)
        """
        if self._kdtree is not None:
            indices = self._kdtree.query_ball_point([cx, cy], radius)
        else:
            r_sq = radius ** 2
            indices = []
            for i, (nx, ny) in enumerate(self._cached_nodes):
                if (nx - cx) ** 2 + (ny - cy) ** 2 < r_sq:
                    indices.append(i)

        sectors = [0] * 6
        total_edges = 0

        for i in indices:
            nx, ny = self._cached_nodes[i]
            dx, dy = nx - cx, ny - cy
            angle = math.atan2(dy, dx)
            angle_deg = (math.degrees(angle) + 360) % 360
            sector_idx = int(angle_deg // 60)
            sectors[sector_idx] += 1

            # Sum edge counts for feature score
            total_edges += self._edge_counts.get(i, 0)

        filled_sectors = sum(1 for s in sectors if s > 0)

        # Normalize edge score (typical node has 2-4 edges)
        avg_edges = total_edges / len(indices) if indices else 0
        edge_score = min(avg_edges / 4.0, 1.0)  # Normalize to 0-1

        return len(indices), filled_sectors, edge_score
