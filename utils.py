"""Utility functions.

Algorithms:
* bresenham_line
* euclidean_distance :-)
* path_length

Message converters:
* pose_to_point
* point_to_pose
* points_to_path
* path_to_points

Coordinate converters:
* robot_to_world
* world_to_robot
* world_to_map
* map_to_world
"""

from typing import List, Tuple, Optional

import numpy as np
from messages import Pose, Vector3, Quaternion, Path, Odometry, OccupancyGrid


def euclidean_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx, dy = a[0] - b[0], a[1] - b[1]
    return np.sqrt(dx * dx + dy * dy)


def pose_to_point(p: Pose) -> Tuple[float, float]:
    return p.position.x, p.position.y


def point_to_pose(x: float, y: float, phi: Optional[float] = None) -> Pose:
    position = Vector3(x, y, 0)
    orientation = Quaternion()
    if phi is not None:
        orientation.from_Euler(phi, 0, 0)
    return Pose(position, orientation)


def points_to_path(points: List[Tuple[float, float]]) -> Path:
    path = Path(poses=[point_to_pose(x, y) for x, y in points])
    return path


def path_to_points(path: Path) -> List[Tuple[float, float]]:
    return [pose_to_point(p) for p in path.poses]


def path_length(path: Path) -> float:
    points = path_to_points(path)
    if len(points) < 2:
        return np.inf
    return sum([euclidean_distance(p1, p2) for p1, p2 in zip(points[:-1], points[1:])])


def robot_to_world(points: List[Tuple[float, float]], odometry: Odometry) -> List[Tuple[float, float]]:
    points = np.asarray(points)

    R = odometry.pose.orientation.to_R()[:2, :2]
    T = np.array([pose_to_point(odometry.pose)])

    points = points @ R.T + T
    return [tuple(p) for p in points]


def world_to_robot(points: List[Tuple[float, float]], odometry: Odometry) -> List[Tuple[float, float]]:
    points = np.asarray(points)

    R = odometry.pose.orientation.to_R()[:2, :2]
    T = np.array([pose_to_point(odometry.pose)])

    points = (points - T) @ R
    return [tuple(p) for p in points]


def world_to_map(points: List[Tuple[float, float]], gridmap: OccupancyGrid) -> List[Tuple[int, int]]:
    points = np.asarray(points)

    origin = pose_to_point(gridmap.origin)
    resolution = gridmap.resolution

    points = np.round((points - origin) / resolution).astype(int)
    return [tuple(p) for p in points]


def map_to_world(points: List[Tuple[int, int]], gridmap: OccupancyGrid) -> List[Tuple[float, float]]:
    points = np.asarray(points)

    origin = pose_to_point(gridmap.origin)
    resolution = gridmap.resolution

    points = points * resolution + origin
    return [tuple(p) for p in points]


def bresenham_line(start: Tuple[int, int],
                   goal: Tuple[int, int]
                   ) -> List[Tuple[int, int]]:
    """Bresenham's line algorithm.

    Parameters
    ----------
    start : (int, int)
        x,y coordinates of a starting point.
    goal : (int, int)
        x,y coordinates of a goal point.

    Returns
    -------
    line: list of (int, int)
        List of x,y coordinates of points representing a line
        that connects start and goal points.
    """
    line: List[Tuple[int, int]] = []
    (x0, y0) = start
    (x1, y1) = goal
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            line.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            line.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    return line
