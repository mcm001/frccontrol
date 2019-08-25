#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")

import frccontrol as frccnt
import math
import matplotlib.pyplot as plt
import numpy as np


class SingleJointedArm(frccnt.System):
    def __init__(self, dt):
        """Single-jointed arm subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Angle", "rad"), ("Angular velocity", "rad/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        frccnt.System.__init__(
            self, np.zeros((2, 1)), np.array([[-12.0]]), np.array([[12.0]]), dt
        )

    def create_model(self, states):
        # Number of motors
        num_motors = 1.0
        # Mass of arm in kg
        m = 4
        # Length of arm in m
        l = 0.2
        # Arm moment of inertia in kg-m^2
        J = 1 / 3 * m * l ** 2
        # Gear ratio
        G = 480.0 / 1.0

        return frccnt.models.single_jointed_arm(
            frccnt.models.MOTOR_775PRO, num_motors, J, G
        )

    def design_controller_observer(self):
        q_pos = 0.01745
        q_vel = 0.08726
        self.design_lqr([q_pos, q_vel], [12.0])
        self.design_two_state_feedforward([q_pos, q_vel], [12.0])

        q_pos = 0.01745
        q_vel = 0.1745329
        r_pos = 0.01
        r_vel = 0.01
        self.design_kalman_filter([q_pos, q_vel], [r_pos])

    def design_Talon_PID(self):
        kp = self.K[0, 0]
        kd = self.K[0, 1]

        # TY oblarg
        # Scale gains to output
        kp = kp / 12 * 1023
        kd = kd / 12 * 1023

        # Rescale kD if not time-normalized
        if not False:
            kd = kd/0.001#self.dt

        # Get correct conversion factor for rotations
        # if STATE.units.get() == 'Degrees':
        # rotation = 360
        # elif STATE.units.get() == 'Radians':
        rotation = 2*math.pi
        # elif STATE.units.get() == 'Rotations':
            # rotation = 1

        # Scale by gearing if using Talon
        # if STATE.controller_type.get() == 'Talon':
        kp = kp * rotation / (4096 * 8)
        kd = kd * rotation / (4096 * 8)
            # if STATE.loop_type.get() == 'Velocity':
            #     kp = kp * 10

        return [kp, kd]


def main():
    dt = 0.001
    single_jointed_arm = SingleJointedArm(dt)
    single_jointed_arm.export_cpp_coeffs("SingleJointedArm", "subsystems/")
    single_jointed_arm.export_java_coeffs("SingleJointedArm")

    if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
        try:
            import slycot

            plt.figure(1)
            single_jointed_arm.plot_pzmaps()
        except ImportError:  # Slycot unavailable. Can't show pzmaps.
            pass
    if "--save-plots" in sys.argv:
        plt.savefig("single_jointed_arm_pzmaps.svg")

    t, xprof, vprof, aprof = frccnt.generate_s_curve_profile(
        max_v=0.5, max_a=1, time_to_max_a=0.5, dt=dt, goal=1.04
    )

    # Generate references for simulation
    refs = []
    for i in range(len(t)):
        r = np.array([[xprof[i]], [vprof[i]]])
        refs.append(r)

    pid = single_jointed_arm.design_Talon_PID()
    print("kp %f kd %f" % (pid[0], pid[1]))

    if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
        plt.figure(2)
        x_rec, ref_rec, u_rec = single_jointed_arm.generate_time_responses(t, refs)
        single_jointed_arm.plot_time_responses(t, x_rec, ref_rec, u_rec)
    if "--save-plots" in sys.argv:
        plt.savefig("single_jointed_arm_response.svg")
    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
