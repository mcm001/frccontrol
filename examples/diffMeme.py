#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")

import frccontrol as frccnt
import matplotlib.pyplot as plt
import numpy as np
import control as cnt
# import control as cnt
# import numpy as np
import scipy as sp
import math

# def lqr(A, B, Q, R, dt):
#     """Solves for the optimal linear-quadratic regulator (LQR).

#     For a continuous system:
#         xdot = A * x + B * u
#         J = int(0, inf, x.T * Q * x + u.T * R * u)
#     For a discrete system:
#         x(n+1) = A * x(n) + B * u(n)
#         J = sum(0, inf, x.T * Q * x + u.T * R * u)

#     Keyword arguments:
#     A -- numpy.array(states x states), The A matrix.
#     B -- numpy.array(inputs x states), The B matrix.
#     Q -- numpy.array(states x states), The state cost matrix.
#     R -- numpy.array(inputs x inputs), The control effort cost matrix.

#     Returns:
#     numpy.array(states x inputs), K
#     """
#     m = A.shape[0]

#     controllability_rank = np.linalg.matrix_rank(cnt.ctrb(A, B))
#     if controllability_rank != m:
#         print(
#             "Warning: Controllability of %d != %d, uncontrollable state"
#             % (controllability_rank, m)
#         )

#     if dt == 0.0:
#         P = sp.linalg.solve_continuous_are(a=A, b=B, q=Q, r=R)
#         return np.linalg.inv(R) @ B.T @ P
#     else:
#         P = sp.linalg.solve_discrete_are(a=A, b=B, q=Q, r=R)
#         return np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A

def __make_cost_matrix(elems):
        """Creates a cost matrix from the given vector for use with LQR.

        The cost matrix is constructed using Bryson's rule. The inverse square
        of each element in the input is taken and placed on the cost matrix
        diagonal.

        Keyword arguments:
        elems -- a vector. For a Q matrix, its elements are the maximum allowed
                 excursions of the states from the reference. For an R matrix,
                 its elements are the maximum allowed excursions of the control
                 inputs from no actuation.

        Returns:
        State excursion or control effort cost matrix
        """
        return np.diag(1.0 / np.square(elems))

v = 1.0

# States: x, y, theta
# Inputs: v, omega

A = np.array([
    [0, 0, 0],
    [0, 0, v],
    [0, 0, 0]
])

B = np.array([
    [1, 0],
    [0, 0],
    [0, 1]
])

C = np.array([
    [0, 0, 1]
])

D = np.array([
    [0, 0]
])

qX = 0.05
qY = 0.05
qTheta = 0.1

Q =  __make_cost_matrix([qX, qY, qTheta])

print(A)
print(Q)

vMax = 3.96
wMax = 1.0

R = __make_cost_matrix([vMax, wMax])

sys = cnt.StateSpace(A, B, C, D)

sysd = sys.sample(1.0/50.0)

K = frccnt.lqr(sysd, Q, R)
print(K)
