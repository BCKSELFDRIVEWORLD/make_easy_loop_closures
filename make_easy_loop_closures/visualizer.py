# Copyright (c) 2026 BCK - MIT License

from enum import Enum
from typing import List, Tuple, Optional
from dataclasses import dataclass

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Time


class State(Enum):
    FREE = 0
    LOOP_NEEDED = 1
    WAITING = 2


@dataclass
class Color:
    r: float
    g: float
    b: float
    a: float = 1.0

    def to_rgba(self) -> ColorRGBA:
        return ColorRGBA(r=self.r, g=self.g, b=self.b, a=self.a)


COLORS = {
    State.FREE: Color(0.0, 0.5, 1.0, 0.6),
    State.LOOP_NEEDED: Color(1.0, 0.5, 0.0, 0.7),
    State.WAITING: Color(1.0, 1.0, 0.0, 0.8),
}


class Visualizer:
    FRAME_ID = "map"
    NS_PATH = "path"
    NS_TARGET = "target"
    NS_TEXT = "status"

    def __init__(self):
        self._marker_id = 0

    def create_markers(
        self,
        stamp: Time,
        state: State,
        path: List[Tuple[float, float]],
        robot_pos: Tuple[float, float],
        target: Optional[Tuple[float, float]],
        target_radius: float,
        status_text: str,
        target_distance: float = 0.0
    ) -> MarkerArray:

        markers = MarkerArray()

        if len(path) >= 2:
            markers.markers.append(
                self._create_path(stamp, path, state)
            )

        markers.markers.append(
            self._create_target(stamp, target, target_radius, state)
        )

        markers.markers.append(
            self._create_text(stamp, robot_pos, status_text, state)
        )

        return markers

    def _create_path(self, stamp: Time, path: List[Tuple[float, float]],
                     state: State) -> Marker:
        m = Marker()
        m.header.frame_id = self.FRAME_ID
        m.header.stamp = stamp
        m.ns = self.NS_PATH
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color = COLORS[state].to_rgba()

        for x, y in path:
            m.points.append(Point(x=x, y=y, z=0.05))

        return m

    def _create_target(self, stamp: Time, target: Optional[Tuple[float, float]],
                       radius: float, state: State) -> Marker:
        m = Marker()
        m.header.frame_id = self.FRAME_ID
        m.header.stamp = stamp
        m.ns = self.NS_TARGET
        m.id = 1

        if target is None or state == State.FREE:
            m.action = Marker.DELETE
            return m

        m.type = Marker.CYLINDER
        m.action = Marker.ADD

        m.pose.position.x = target[0]
        m.pose.position.y = target[1]
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0

        m.scale.x = radius * 2
        m.scale.y = radius * 2
        m.scale.z = 1.0

        if state == State.WAITING:
            m.color = Color(1.0, 1.0, 0.0, 0.5).to_rgba()
        else:
            m.color = Color(1.0, 0.0, 0.0, 0.4).to_rgba()

        return m

    def _create_text(self, stamp: Time, pos: Tuple[float, float],
                     text: str, state: State) -> Marker:
        m = Marker()
        m.header.frame_id = self.FRAME_ID
        m.header.stamp = stamp
        m.ns = self.NS_TEXT
        m.id = 2
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 1.2
        m.pose.orientation.w = 1.0

        m.scale.z = 0.25
        m.text = text

        if state == State.FREE:
            m.color = Color(0.3, 1.0, 0.3).to_rgba()
        elif state == State.WAITING:
            m.color = Color(1.0, 1.0, 0.0).to_rgba()
        else:
            m.color = Color(1.0, 0.3, 0.3).to_rgba()

        return m

    def create_delete_all(self, stamp: Time) -> MarkerArray:
        m = Marker()
        m.header.stamp = stamp
        m.action = Marker.DELETEALL
        return MarkerArray(markers=[m])
