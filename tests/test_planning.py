#!/usr/bin/env python3
"""
Test the path-planning component by building the paths in four different maps.

Outputs the optimal, robust and simplified paths for every map, given the path exists.
"""

import pickle

import matplotlib.pyplot as plt

from explorer import Planner

ROBOT_SIZE = 0.3


if __name__ == "__main__":
    planner = Planner(ROBOT_SIZE)

    with open("../assets/scenarios_planning.bag", "rb") as f:
        scenarios = pickle.load(f)

    for gridmap, start, goal in scenarios:
        planner.update_gridmap(gridmap)
        path, _ = planner.plan_path(start, goal, radius=0, simplify=False, robust=False)
        robust_path, _ = planner.plan_path(start, goal, radius=0, simplify=False)
        simple_path, _ = planner.plan_path(start, goal, radius=0, robust=False)

        fig, ax = plt.subplots()
        gridmap.plot(ax)
        path.plot(ax, line=True, color="green")
        robust_path.plot(ax, line=True, color="blue")
        simple_path.plot(ax, line=True, color="red")

        plt.xlabel('x[m]')
        plt.ylabel('y[m]')
        plt.axis('square')
        plt.show()
