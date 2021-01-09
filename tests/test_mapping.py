#!/usr/bin/env python3
"""
Test the mapping component by executing the hard-coded path in a given environment.

Open assets/scene1_plain.ttt in CoppeliaSim and run the script.
"""

import matplotlib.pyplot as plt

from explorer import Mapper
from controller import HexapodController
from messages import Pose, Vector3, Quaternion


if __name__ == "__main__":
    robot = HexapodController(0)

    # turn on the robot
    robot.turn_on()

    # start navigation thread
    robot.start_navigation()

    # assign goal for navigation
    goals = [
        Pose(Vector3(1, -1, 0), Quaternion(1, 0, 0, 0)),
        Pose(Vector3(1, 1, 0), Quaternion(1, 0, 0, 0)),
        Pose(Vector3(-1, 0, 0), Quaternion(1, 0, 0, 0)),
        Pose(Vector3(-3, 0, 0), Quaternion(1, 0, 0, 0)),
    ]

    # prepare the online plot
    plt.ion()
    fig, ax = plt.subplots()

    try:
        mapper = Mapper()

        # go from goal to goal
        for goal in goals:
            robot.goto(goal)
            while robot.navigation_goal is not None:
                plt.cla()

                mapper.update_odometry(robot.odometry)
                mapper.fuse_laser_scan(robot.laser_scan)

                # plot the map
                mapper.map.plot(ax)
                plt.xlabel('x[m]')
                plt.ylabel('y[m]')
                plt.axis('square')
                plt.show()
                plt.pause(0.1)
    except Exception as e:
        print(f"During execution, an error occurred: {e}")
    finally:
        robot.stop_navigation()
        robot.turn_off()
