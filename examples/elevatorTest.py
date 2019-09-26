import sys

import frccontrol as frccnt
import matplotlib.pyplot as plt
import numpy as np
import math

# Number of motors
num_motors = 1.0
# Elevator carriage mass in kg
m = 35
# Radius of pulley in meters
r = 0.019
# Gear ratio
G = 42.0 / 1.0

gearbox = frccnt.models.gearbox(frccnt.models.MOTOR_NEO, num_motors)

# V = Tm / Kt * R + Wm / Kv

Kt = gearbox.Kt
Kv = gearbox.Kv
R = gearbox.R

Tm = 6.5 / G # newton meters
Wm = 0.1 * G
Voltage = Tm / Kt * R + Wm / Kv
print(Voltage)