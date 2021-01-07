"""Hexapod motion planner implementation."""

from typing import List, Tuple, Optional, Union

import numpy as np
import skimage.measure as skm
from sklearn.cluster import KMeans
from scipy.ndimage import distance_transform_edt, convolve

from messages import OccupancyGrid, Pose, Path, Odometry

from .astar import AStar
from .const import (
    SENSOR, OBSTACLE_PENALTY_FACTOR,
    MIN_CELLS_PER_CENTROID,
    ASTAR_TOLERANCE_FACTOR,
    OBSTACLE_GROWTH_FACTOR,
)
from utils import (
    pose_to_point, point_to_pose, points_to_path, path_to_points,
    bresenham_line, path_length, euclidean_distance,
    map_to_world, world_to_map,
)


class Planner:
    """Hexapod motion planner.

    Public methods:
    * constructor
    * update_gridmap
    * is_pose_reachable
    * is_path_executable

    Planning methods:
    * find_frontiers
    * find_frontiers_inf_gain
    * plan_path
    """
    astar: AStar
    robot_size: float

    odometry: Odometry
    gridmap: OccupancyGrid
    maze: List[List[int]]
    dist_to_obstacles: np.ndarray

    def __init__(self, robot_size: float):
        self.astar = AStar()
        self.robot_size = robot_size

    def update_gridmap(self, gridmap: OccupancyGrid):
        assert isinstance(gridmap.data, np.ndarray)
        assert gridmap.data.shape == (gridmap.height, gridmap.width)
        self.gridmap = gridmap.copy()
        self.maze = self._make_maze(self.gridmap, OBSTACLE_GROWTH_FACTOR * self.robot_size)
        self.dist_to_obstacles = distance_transform_edt(self.gridmap.data < 0.5)

    def is_pose_reachable(self, pose: Pose) -> bool:
        """Test if a given pose can be reached (lies within the map and is far enough from obstacles)."""
        x, y = world_to_map([pose_to_point(pose)], self.gridmap)[0]
        return self.gridmap.is_valid_coordinate(x, y) and not self.maze[y][x]

    def is_path_executable(self, path: Path) -> bool:
        return all([self.is_pose_reachable(pose) for pose in path.poses])

    def find_frontiers(self) -> List[Pose]:
        """Find free edge frontiers in a probabilistic grid map.

        Returns
        -------
        frontiers : list of Pose
            A (possibly empty) List of free-edge frontiers.
        """

        # construct a free edge cell mask
        free_edge_cells = self._find_free_edge_cells(self.gridmap)

        # find frontiers representatives
        frontiers = self._find_free_edge_centroids(free_edge_cells, self.gridmap.resolution)
        if not frontiers:
            return []

        # convert to world coordinates
        frontiers = map_to_world(frontiers, self.gridmap)
        return [point_to_pose(x, y) for x, y in frontiers]

    @staticmethod
    def _find_free_edge_cells(gridmap: OccupancyGrid) -> np.ndarray:
        # free edge cells are unexplored cells that contain at least one more unexplored
        # cell and at least one free cell in their 8-neighborhood.
        # the very weird but efficient way to detect them using convolutions follows:

        mask = np.array([
            [1,  1, 1],
            [1, 10, 1],
            [1,  1, 1]
        ])
        data = np.where(gridmap.data < 0.5, 100, np.where(gridmap.data == 0.5, 10, 1))
        conv = convolve(data, mask, mode='constant', cval=0.)

        def digit(x, n):
            return (x // 10 ** n) % 10

        return np.logical_and(conv > 1000, digit(conv, 1) >= 1, digit(conv, 2) >= 1)

    @staticmethod
    def _find_free_edge_centroids(free_edge_cells: np.ndarray, 
                                  map_resolution: float
                                  ) -> List[Tuple[int, int]]:
        x = np.arange(free_edge_cells.shape[1])
        y = np.arange(free_edge_cells.shape[0])
        xx, yy = np.meshgrid(x, y)

        # partition free-edge cells into multiple frontiers
        labels, num_labels = skm.label(free_edge_cells, connectivity=2, return_num=True)

        centroids = []
        for label in range(1, num_labels+1):
            # all points belonging to a given frontier
            X = np.c_[
                xx[labels == label].flatten(),
                yy[labels == label].flatten()
            ]

            # compute the number of representatives for a given frontier
            f = len(X)  # number of cells
            D = SENSOR.MAX_RANGE / map_resolution  # some magic constant
            num_centroids = 1 + int(np.floor(f / D + .5))

            if num_centroids == 1:
                if f >= MIN_CELLS_PER_CENTROID:
                    # simple average of all cells
                    centroids.append(tuple(X.mean(axis=0)))
            else:
                # cluster cells
                clf = KMeans(n_clusters=num_centroids).fit(X.astype(np.float32))
                for cluster, num_cells in zip(*np.unique(clf.labels_, return_counts=True)):
                    if num_cells >= MIN_CELLS_PER_CENTROID:
                        # centroid of a given cluster
                        centroids.append(tuple(clf.cluster_centers_[cluster]))

        return centroids

    def find_frontiers_inf_gain(self,
                                pose: Pose,
                                frontiers: List[Pose],
                                return_path_lengths: bool = False
                                ) -> Union[List[float], Tuple[List[float], List[float]]]:
        """Compute the information gain at presented frontiers.

        The current approach lies in determining the path to a given frontier, deducing the robot's
        orientation at the last point, and using a simple sensor model to find which cells can be
        observed from that position.

        Even more precise approach would consider observations from all the points along the path
        to a given frontier, not just the last one. It is rather tedious (and slow) though, since
        we must be careful to not double-count some points.

        Parameters
        ----------
        pose : Pose
            Current pose in world coordinates.
        frontiers : list of Pose
            Frontier centroids in world coordinates.
        return_path_lengths : bool, optional
            If True, return the lengths of paths to the given frontiers.
            numpy.inf indicates that there is no path. Default False.

        Returns
        -------
        inf_gain : list of float
            Information gain at each of the given frontiers.
        path_lengths : list of float
            Only when return_path_lengths = True.
        """

        # (approximate) information gain at individual map cells
        p = self.gridmap.data
        entropy = -p * np.log(p) - (1 - p) * np.log(1 - p)

        inf_gain = [0] * len(frontiers)
        path_lengths = [np.inf] * len(frontiers)
        for idx, frontier in enumerate(frontiers):
            # plan the path to a given frontier
            path, path_lengths[idx] = self.plan_path(pose, frontier)
            if len(path.poses) < 2:
                continue

            # calculate the assumed robot's orientation at the last point
            (xr, yr), (xf, yf) = path_to_points(path)[-2:]
            orientation = np.arctan2(yf - yr, xf - xr)

            # all angles captured by the sensor
            thetas = orientation + np.linspace(
                SENSOR.MIN_ANGLE,
                SENSOR.MAX_ANGLE,
                SENSOR.NUM_RAYS
            )

            # raytrace each individual angle until we hit an obstacle or the map edge
            for theta in thetas:
                # all map points covered by the sensor ray from a given point
                ray = self._ray(xf, yf, theta, self.gridmap)
                for x, y in ray:
                    # can't see beyond the map or behind an obstacle
                    if not self.gridmap.is_valid_coordinate(x, y) or self.gridmap.get(x, y) > 0.5:
                        break
                    inf_gain[idx] += entropy[y, x]

        if return_path_lengths:
            return inf_gain, path_lengths
        return inf_gain

    @staticmethod
    def _ray(x: float, y: float, angle: float, gridmap: OccupancyGrid) -> List[Tuple[int, int]]:
        start = x + np.cos(angle) * SENSOR.MIN_RANGE, y + np.sin(angle) * SENSOR.MIN_RANGE
        end = x + np.cos(angle) * SENSOR.MAX_RANGE, y + np.sin(angle) * SENSOR.MAX_RANGE
        start, end = world_to_map([start, end], gridmap)
        ray = bresenham_line(start, end)
        return ray

    def plan_path(self,
                  start: Pose,
                  goal: Pose,
                  radius: Optional[float] = None,
                  simplify: bool = True,
                  robust: bool = True,
                  ) -> Tuple[Path, float]:
        """Plan a path from start to goal over a given map using A* algorithm.

        Parameters
        ----------
        start : Pose
            Start position in world coordinates.
        goal : Pose
            Goal position in world coordinates.
        radius : float, optional
            The final point in the found path is within the neighborhood of
            a given radius from the goal. Default `2 * robot_size`.
        simplify : bool, optional
            If True (default), simplifies the found path by reducing the amount
            of waypoints to the minimal necessary so that the path still doesn't
            collide with obstacles.
        robust : bool, optional
            If True (default), steers A* heuristic to avoid paths that are close to obstacles.

        Returns
        -------
        path : path
            The found path.
        length : float
            Path length (sum of distances between consecutive points).
        """
        if radius is None:
            radius = ASTAR_TOLERANCE_FACTOR * self.robot_size
        radius /= self.gridmap.resolution

        # transform start and goal to the useful format
        start, goal = pose_to_point(start), pose_to_point(goal)

        # transform start and goal from world coordinates to map coordinates
        start, goal = world_to_map([start, goal], self.gridmap)

        # steer the A* heuristic for robust planning
        def steered_heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
            return euclidean_distance(a, b) + OBSTACLE_PENALTY_FACTOR / self.dist_to_obstacles[a[1], a[0]]

        self.astar.heuristic = steered_heuristic if robust else euclidean_distance

        # find a path using A* search
        path = self.astar.search(
            self.maze, start, goal,
            neighborhood="N8",
            radius=radius
        )

        # no path found
        if len(path) < 2:
            return Path(), np.inf

        # simplify the found path
        if simplify:
            path = self._simplify_path(path, self.maze)

        # transform the path back to the world coordinates
        path = map_to_world(path, self.gridmap)

        # create the Path message
        path = points_to_path(path)

        # compute the path length
        length = path_length(path)

        return path, length

    @staticmethod
    def _make_maze(gridmap: OccupancyGrid, distance: float) -> List[List[int]]:
        # grow obstacles using l2 distance transform
        maze = np.where(gridmap.data > 0.5, 1, 0)
        dist = distance_transform_edt(1 - maze, sampling=gridmap.resolution)
        maze[(maze == 0) & (dist < distance)] = 1

        # don't plan through unexplored cells
        # (would be nice to grow them as well, but it leaves too little empty space)
        maze = np.logical_or(gridmap.data >= 0.5, maze).astype(int)
        return maze.tolist()

    @staticmethod
    def _simplify_path(path: List[Tuple[int, int]], maze: List[List[int]]) -> List[Tuple[int, int]]:
        simple_path = [path[0]]
        while simple_path[-1] != path[-1]:
            # drop every pose after the current until the line connecting
            # the former and the latter intersects an obstacle
            current_pose = simple_path[-1]
            current_loc = path.index(current_pose)
            for next_pose in path[current_loc + 1:]:
                line = bresenham_line(simple_path[-1], next_pose)
                if not any([maze[y][x] for x, y in line]):
                    current_pose = next_pose
                    if current_pose == path[-1]:
                        simple_path.append(current_pose)
                        break
                else:
                    simple_path.append(current_pose)
                    break
        return simple_path
