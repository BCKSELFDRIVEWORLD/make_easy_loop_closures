# Copyright (c) 2026 BCK - MIT License

import math
from typing import List, Tuple, Optional


class TargetFinder:
    def __init__(self, min_distance: float, max_distance: float, min_points: int, min_pose_age: int = 20):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.min_points = min_points
        self.min_pose_age = min_pose_age
        self.density_radius = 1.0

    def find(self, robot_x: float, robot_y: float,
             path: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:

        if len(path) < self.min_points:
            return None

        candidates = []
        search_end = len(path) - max(self.min_points, self.min_pose_age)

        if search_end <= 0:
            return None

        densities = self._calculate_densities(path[:search_end])
        max_density = max(densities) if densities else 1

        for i, (px, py) in enumerate(path[:search_end]):
            dist = math.sqrt((px - robot_x) ** 2 + (py - robot_y) ** 2)

            if self.min_distance < dist < self.max_distance:
                age_score = (len(path) - i) / len(path)
                dist_score = 1.0 - (dist / self.max_distance)
                density_score = densities[i] / max_density if max_density > 0 else 0

                score = age_score * 0.3 + dist_score * 0.3 + density_score * 0.4
                candidates.append((px, py, score, dist))

        if not candidates:
            return path[0] if path else None

        candidates.sort(key=lambda c: c[2], reverse=True)
        return (candidates[0][0], candidates[0][1])

    def _calculate_densities(self, path: List[Tuple[float, float]]) -> List[int]:
        densities = []
        r_sq = self.density_radius ** 2

        for i, (px, py) in enumerate(path):
            count = 0
            for j, (ox, oy) in enumerate(path):
                if i != j:
                    dx = px - ox
                    dy = py - oy
                    if dx * dx + dy * dy < r_sq:
                        count += 1
            densities.append(count)

        return densities
