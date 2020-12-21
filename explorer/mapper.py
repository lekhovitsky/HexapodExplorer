"""Mapper implementation."""

from typing import Tuple, List

import numpy as np
from messages import LaserScan, OccupancyGrid, Odometry

from utils import pose_to_point, bresenham_line, robot_to_world, world_to_map


class Mapper:
    """Hexapod mapper.

    Public methods:
    * constructor
    * update_odometry

    Mapping methods:
    * fuse_laser_scan
    """

    map: OccupancyGrid
    odometry: Odometry
    _map_initialized: bool

    def __init__(self, resolution: float = .1):
        gridmap = OccupancyGrid()
        gridmap.resolution = resolution
        gridmap.height = gridmap.width = 0
        self.map = gridmap
        self._map_initialized = False

    def update_odometry(self, odometry: Odometry):
        self.odometry = odometry.copy()
        if not self._map_initialized:
            self.map.origin.position.x = odometry.pose.position.x
            self.map.origin.position.y = odometry.pose.position.y
            self._map_initialized = True

    @property
    def position(self) -> Tuple[float, float]:
        return pose_to_point(self.odometry.pose)

    @property
    def orientation(self):
        return self.odometry.pose.orientation.to_Euler()[0]

    def fuse_laser_scan(self, laser_scan: LaserScan):
        """Fuse the laser scan data into the probabilistic occupancy grid map."""
        assert isinstance(laser_scan, LaserScan)

        # laser scan points in map coordinates
        points_r = self._read_laser_data(laser_scan)
        points_w = robot_to_world(points_r, self.odometry)
        points = world_to_map(points_w, self.map)

        # extend the map if needed
        extended = self._extend_map(points)
        if extended:
            points = world_to_map(points_w, self.map)

        # robot position in map coordinates
        robot = world_to_map([self.position], self.map)[0]

        # raytrace the points and update the map accordingly
        free, occupied = self._raytrace(robot, points)
        self._update_map(free, occupied)

    @staticmethod
    def _read_laser_data(laser_scan: LaserScan) -> List[Tuple[float, float]]:
        theta_left, theta_step, dist_min, dist_max, distances = (
            laser_scan.angle_min, laser_scan.angle_increment,
            laser_scan.range_min, laser_scan.range_max,
            np.array(laser_scan.distances)
        )

        # generate the range of angles
        thetas = theta_left + theta_step * np.arange(len(distances))

        # filter out the angles with inappropriate distances
        valid = (distances >= dist_min) & (distances <= dist_max)

        xs: List[float] = distances[valid] * np.cos(thetas[valid])
        ys: List[float] = distances[valid] * np.sin(thetas[valid])
        return list(zip(xs, ys))

    @staticmethod
    def _raytrace(robot: Tuple[int, int],
                  points: List[Tuple[int, int]]
                  ) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
        free, occupied = [], []
        for point in points:
            # raytrace the points
            line = bresenham_line(robot, point)

            # save the coordinate of free space cells
            free.extend(line)

            # save the coordinate of occupied cell
            occupied.append(point)

        return free, occupied

    def _extend_map(self, points: List[Tuple[int, int]]) -> bool:
        x_min, x_max = min([x for x, _ in points]), max([x for x, _ in points])
        y_min, y_max = min([y for _, y in points]), max([y for _, y in points])

        width = max(x_max + 1, self.map.width) - min(x_min, 0)
        height = max(y_max + 1, self.map.height) - min(y_min, 0)
        if width == self.map.width and height == self.map.height:
            return False

        data = np.full((height, width), 0.5)
        x_shift, y_shift = max(-x_min, 0), max(-y_min, 0)
        if self.map.data is not None:
            data[y_shift : y_shift + self.map.height, x_shift : x_shift + self.map.width] = self.map.data

        self.map.data = data
        self.map.height = height
        self.map.width = width
        self.map.origin.position.x -= x_shift * self.map.resolution
        self.map.origin.position.y -= y_shift * self.map.resolution
        return True

    def _update_map(self, free: List[Tuple[int, int]], occupied: List[Tuple[int, int]]):
        for x, y in free:
            self._update_free_cell(x, y)

        for x, y in occupied:
            self._update_occupied_cell(x, y)

    def _update_free_cell(self, x: int, y: int):
        p = self.map.get(x, y)
        # prior probabilities of cell being occupied/free
        p_occupied, p_free = p, 1 - p
        # conditional probabilities of observing a free cell, given it is occupied/free
        pz_occupied, pz_free = 0.025, 0.975
        # posterior probability of cell being occupied, given observed to be free
        p = pz_occupied * p_occupied / (pz_occupied * p_occupied + pz_free * p_free)
        self.map.set(x, y, np.clip(p, 0.01, 0.99))

    def _update_occupied_cell(self, x: int, y: int):
        p = self.map.get(x, y)
        # prior probabilities of cell being occupied/free
        p_occupied, p_free = p, 1 - p
        # conditional probabilities of observing an occupied cell, given it is occupied/free
        pz_occupied, pz_free = 0.975, 0.025
        # posterior probability of cell being occupied, given observed to be so
        p = pz_occupied * p_occupied / (pz_occupied * p_occupied + pz_free * p_free)
        self.map.set(x, y, np.clip(p, 0.01, 0.99))
