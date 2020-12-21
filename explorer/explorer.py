"""Hexapod Explorer implementation."""

import sys
import time
import threading
from typing import Optional
from itertools import chain

from messages import OccupancyGrid, Path, Pose, LaserScan, Odometry
from controller import HexapodController

from .mapper import Mapper
from .planner import Planner
from .const import ROBOT_SIZE, MAP_RESOLUTION, STEP_SIZE


class Explorer:
    done: bool
    lock: threading.Lock

    mapper: Mapper
    planner: Planner
    controller: HexapodController

    goal: Optional[Pose]
    planned_path: Path
    executed_path: Path

    def __init__(self, robot_id: int = 0):
        self.done = False
        self.lock = threading.Lock()

        self.mapper = Mapper(resolution=MAP_RESOLUTION)
        self.planner = Planner(robot_size=ROBOT_SIZE)
        self.controller = HexapodController(robot_id)

        self.goal = None
        self.planned_path = Path()
        self.executed_path = Path()

    def start(self):
        # turn on the robot
        self.controller.turn_on()
        self.controller.start_navigation()

        # start mapping, planning and navigation threads
        try:
            mapping_thread = threading.Thread(target=self.mapping)
            mapping_thread.start()
        except:
            print("Cannot start the mapping thread")
            sys.exit(1)

        try:
            planning_thread = threading.Thread(target=self.planning)
            planning_thread.start()
        except:
            print("Cannot start the planning thread")
            sys.exit(1)

        try:
            navigation_thread = threading.Thread(target=self.navigation)
            navigation_thread.start()
        except:
            print("Cannot start the navigation thread")
            sys.exit(1)

    def stop(self):
        # stop navigation thread and turn off the robot
        self.controller.stop_navigation()
        self.controller.turn_off()

    @property
    def initialized(self) -> bool:
        return self.map.data is not None and self.odometry is not None

    @property
    def map(self) -> OccupancyGrid:
        return self.mapper.map

    @property
    def odometry(self) -> Odometry:
        return self.controller.odometry

    @property
    def pose(self) -> Pose:
        return self.controller.odometry.pose

    @property
    def scan(self) -> LaserScan:
        return self.controller.laser_scan

    @property
    def _last_valid_pose(self) -> Optional[Pose]:
        """Get robot's last valid pose.

        Since it might happen that the robot entries an area considered as grown
        obstacle by the planner due to imperfect robot maneuverability, this point
        should be used to plan from instead of self.pose.
        """
        for pose in chain([self.pose], reversed(self.executed_path.poses)):
            if self.planner.is_pose_reachable(pose):
                return pose

    def _record_pose(self):
        """Save the current robot's pose.

        Happens after reaching some distance from the last previously recorded pose.
        """
        if (
            not self.executed_path.poses
            or self.pose.dist(self.executed_path.poses[-1]) > STEP_SIZE
        ):
            self.executed_path.poses.append(self.pose.copy())

    def mapping(self):
        while not self.done:
            while not (self.odometry and self.scan):
                time.sleep(0.5)

            # update the mapper state
            self.mapper.update_odometry(self.odometry)

            # fuse the laser scan data into the map
            self.mapper.fuse_laser_scan(self.scan)
            time.sleep(0.5)

    def planning(self):
        time.sleep(5)
        while not self.done:
            while not self.initialized:
                time.sleep(1)

            self.planner.update_gridmap(self.map)

            if not self._is_goal_valid():
                print("Planning the new goal")
                self._plan_goal()

            if not self._is_path_valid():
                print("Planning the path to the goal")
                self._plan_path()

            time.sleep(1)

    def navigation(self):
        time.sleep(5)
        while not self.done:
            while not self.initialized:
                time.sleep(0.5)

            self._record_pose()

            if self.controller.collision:
                print(f"Collision at {self.pose} :(")
                self.done = True
                break

            if self.controller.navigation_goal is None:
                if self.planned_path.poses:
                    self.lock.acquire()
                    self.planned_path.poses.pop(0)  # arrived to this waypoint
                    if self.planned_path.poses:
                        waypoint = self.planned_path.poses[0]  # new waypoint
                        print(f"Next waypoint: {waypoint.position}")
                        self.controller.goto(waypoint)
                    else:
                        print("Goal reached")
                        self.goal = None  # signal that we need a new goal
                    self.lock.release()

            time.sleep(0.5)

    def _is_goal_valid(self) -> bool:
        if self.done or self.goal is None:
            return False
        # return self.planner.is_pose_reachable(self.pose)
        return True

    def _is_path_valid(self) -> bool:
        if self.done or self.goal is None or not self.planned_path.poses:
            return False
        return self.planner.is_path_executable(self.planned_path)

    def _plan_goal(self):
        # compute utility of a frontier as the ratio of information gain and path length
        frontiers = self.planner.find_frontiers()
        inf_gain, path_lengths = self.planner.find_frontiers_inf_gain(
            self._last_valid_pose, frontiers,
            return_path_lengths=True
        )
        utilities = [ig / pl for ig, pl in zip(inf_gain, path_lengths)]

        self.lock.acquire()
        if any(utilities):
            # select the frontier with maximum utility
            frontier = frontiers[utilities.index(max(utilities))]
            print(f"New goal: {frontier.position}")
            self.goal = frontier
        else:
            print("No more reachable frontiers. Finishing exploration")
            self.done = True
        self.lock.release()

    def _plan_path(self):
        if self.done:
            return

        # plan the new path to the goal
        path, _ = self.planner.plan_path(self._last_valid_pose, self.goal)
        if path.poses:
            self.lock.acquire()
            print(f"Path found, {len(path.poses)-1} waypoints")
            self.controller.stop()
            self.planned_path = path
            self.lock.release()
        else:
            self.lock.acquire()
            print("Path not found dropping the goal")
            self.controller.stop()
            self.goal = None  # signal that we need a new goal
            self.lock.release()
