#!/usr/bin/env python3

import sys
import pickle

import numpy as np
import matplotlib.pyplot as plt

from explorer import Planner
from utils import point_to_pose


if __name__ == "__main__":
    planner = Planner(robot_size=0)

    dataset = pickle.load(open("../assets/scenarios_frontiers.bag", "rb"))
    for gridmap, (x, y) in zip(dataset, [(4, 2), (30, 15), (15, 25)]):
        data = gridmap.data
        data = np.where(data == 1, 0.95, np.where(data == 0, 0.05, 0.5))
        gridmap.data = data.reshape((gridmap.height, gridmap.width))

        planner.update_gridmap(gridmap)

        pose = point_to_pose(x, y)

        # find the frontiers
        frontiers = list(planner.find_frontiers())

        # compute information gain
        inf_gain = list(planner.find_frontiers_inf_gain(pose, frontiers))

        # plot the map
        fig, ax = plt.subplots()

        # plot the gridmap
        gridmap.plot(ax)

        # plot the cluster centers
        for p, inf_gain in zip(frontiers, inf_gain):
            plt.plot([p.position.x], [p.position.y], '.', markersize=20)
            plt.annotate(f"{inf_gain:.2f}", (p.position.x, p.position.y))

        plt.xlabel('x[m]')
        plt.ylabel('y[m]')
        ax.set_aspect('equal', 'box')
        plt.show()
