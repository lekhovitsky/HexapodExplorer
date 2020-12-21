"""Locomotion-related constants."""

import math


TIME_FRAME = 1  # the length of the simulation step and real robot control frame [ms]

SERVOS_BASE = [0, 0, -math.pi / 6., math.pi / 6., math.pi / 3., -math.pi / 3.,
               0, 0, -math.pi / 6., math.pi / 6., math.pi / 3., -math.pi / 3.,
               0, 0, -math.pi / 6., math.pi / 6., math.pi / 3., -math.pi / 3.]

# robot construction constants
COXA_SERVOS = [1, 13, 7, 2, 14, 8]
COXA_OFFSETS = [math.pi / 8, 0, -math.pi / 8, -math.pi / 8, 0, math.pi / 8]
FEMUR_SERVOS = [3, 15, 9, 4, 16, 10]
FEMUR_OFFSETS = [-math.pi / 6, -math.pi / 6, -math.pi / 6, math.pi / 6, math.pi / 6, math.pi / 6]
TIBIA_SERVOS = [5, 17, 11, 6, 18, 12]
TIBIA_OFFSETS = [math.pi / 3, math.pi / 3, math.pi / 3, -math.pi / 3, -math.pi / 3, -math.pi / 3]
SIGN_SERVOS = [-1, -1, -1, 1, 1, 1]

FEMUR_MAX = 2.1
TIBIA_MAX = 0.1
COXA_MAX = 0.01

SPEEDUP = 4

DELTA_DISTANCE = 0.15
SLOWDOWN_DISTANCE = 1
TURNING_SPEED = 16
