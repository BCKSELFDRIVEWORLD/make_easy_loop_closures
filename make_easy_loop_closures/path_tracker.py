# Copyright (c) 2026 BCK - MIT License

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class PathTracker:
    min_travel_distance: float = 0.1
    max_history: int = 5000

    path: deque = field(default_factory=lambda: deque(maxlen=5000))
    total_distance: float = 0.0
    distance_since_loop: float = 0.0
    _last_pos: Optional[Tuple[float, float]] = None

    def __post_init__(self):
        self.path = deque(maxlen=self.max_history)

    def update(self, x: float, y: float) -> bool:
        if self._last_pos is None:
            self._last_pos = (x, y)
            return False

        dx = x - self._last_pos[0]
        dy = y - self._last_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > self.min_travel_distance:
            self.total_distance += dist
            self.distance_since_loop += dist
            self.path.append((x, y))
            self._last_pos = (x, y)
            return True

        return False

    def reset_loop_distance(self):
        self.distance_since_loop = 0.0

    def get_path_list(self) -> List[Tuple[float, float]]:
        return list(self.path)

    @property
    def point_count(self) -> int:
        return len(self.path)
