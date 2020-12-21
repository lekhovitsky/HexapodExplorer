"""Exploration-related constants."""

import numpy as np

ROBOT_SIZE = 0.3
PLOTTING_STEP = 0.1
MAP_RESOLUTION = 0.1
ASTAR_TOLERANCE_FACTOR = 2.0
OBSTACLE_GROWTH_FACTOR = 1.5
MIN_CELLS_PER_CENTROID = 20


class SENSOR:
    """Simple model of robot's sensor."""
    MIN_RANGE: float = .1
    MAX_RANGE: float = 10
    MIN_ANGLE: float = -np.pi / 4
    MAX_ANGLE: float = np.pi / 4
    NUM_RAYS: int = 64
