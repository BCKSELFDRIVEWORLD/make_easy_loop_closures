# Copyright (c) 2026 BCK - MIT License

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class TFState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class LoopDetector:
    def __init__(self, min_edge_increase: int = 3):
        self.min_edge_increase = min_edge_increase
        self.last_tf = TFState()
        self.last_edge_count = 0
        self.initialized = False

    def check(self, x: float, y: float, yaw: float, edge_count: int) -> Tuple[bool, float, int]:
        if not self.initialized:
            self.last_tf = TFState(x, y, yaw)
            self.last_edge_count = edge_count
            self.initialized = True
            return False, 0.0, 0

        dx = abs(x - self.last_tf.x)
        dy = abs(y - self.last_tf.y)
        position_change = math.sqrt(dx * dx + dy * dy)

        edge_increase = edge_count - self.last_edge_count

        detected = edge_increase >= self.min_edge_increase

        self.last_tf = TFState(x, y, yaw)
        self.last_edge_count = edge_count

        return detected, position_change, edge_increase

    def reset(self):
        self.initialized = False
