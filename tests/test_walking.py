#!/usr/bin/env python3
"""
Test the trajectory following component by executing the hard-coded path
in a given environment.

Open assets/scene1_plain.ttt in CoppeliaSim and run the script.
"""

import time
import matplotlib.pyplot as plt

import controller
from messages import Path, Pose, Vector3, Quaternion


if __name__ == "__main__":
    robot = controller.HexapodController(0)

    # turn on the robot
    robot.turn_on()

    # start navigation thread
    robot.start_navigation()

    try:
        # assign goal for navigation
        goals = [
            Pose(Vector3(1, -1, 0), Quaternion(1, 0, 0, 0)),
            Pose(Vector3( 1, 1, 0), Quaternion(1, 0, 0, 0)),
            Pose(Vector3(-1, 0, 0), Quaternion(1, 0, 0, 0)),
            Pose(Vector3(-3, 0, 0), Quaternion(1, 0, 0, 0)),
        ]

        path = Path()
        # go from goal to goal
        for goal in goals:
            robot.goto(goal)
            while robot.navigation_goal is not None:
                # sample the current odometry
                if robot.odometry is not None:
                    path.poses.append(robot.odometry.pose)
                # wait
                time.sleep(0.1)

            # check the robot distance to goal
            odom = robot.odometry.pose
            # compensate for the height of the robot as we are interested only in achieved planar distance
            odom.position.z = 0
            # calculate the distance
            dist = goal.dist(odom)
            print("[t1a_eval] distance to goal: %.2f" % dist)

        # plot the robot path
        fig, ax = plt.subplots()
        path.plot(ax, 30)
        plt.xlabel('x[m]')
        plt.ylabel('y[m]')
        plt.axis('equal')
        plt.show()
    except Exception as e:
        print(f"During execution, an exception occurred: {e}")
    finally:
        robot.stop_navigation()
        robot.turn_off()
