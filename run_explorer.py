#!/usr/bin/env python3

from typing import Optional

import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from explorer import Explorer
from messages import OccupancyGrid, Path


def main():
    # instantiate the robot
    ex0 = Explorer(0)
    # start the locomotion
    ex0.start()

    # continuously plot the map, planned and executed path
    fig, ax = plt.subplots()
    plt.ion()

    while not ex0.done:
        plot(ax, ex0.map.copy(),
             planned=ex0.planned_path.copy(),
             executed=ex0.executed_path.copy())
        plt.show()
        plt.pause(1)

    ex0.stop()
    plot(ax, ex0.map.copy(), executed=ex0.executed_path.copy())
    plt.savefig("assets/map.png")
    print("Done.")


def plot(ax: Axes, gridmap: OccupancyGrid, planned: Optional[Path] = None, executed: Optional[Path] = None):
    ax.clear()
    if gridmap.data is not None:
        gridmap.plot(ax)
    if planned and planned.poses:
        planned.plot(ax)
    if executed and executed.poses:
        executed.plot(ax, line=True)
    ax.set(xlabel="x[m]", ylabel="y[m]")
    plt.ylabel('y[m]')
    ax.set_aspect('equal', 'box')


if __name__ == "__main__":
    main()
