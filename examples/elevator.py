#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")

import frccontrol as frccnt
import matplotlib.pyplot as plt
import numpy as np
import math


class Elevator(frccnt.System):
    def __init__(self, dt):
        """Elevator subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Position", "m"), ("Velocity", "m/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        frccnt.System.__init__(
            self, np.zeros((2, 1)), np.array([[-12.0]]), np.array([[12.0]]), dt
        )

    def create_model(self, states):
        # Number of motors
        num_motors = 4.0
        # Elevator carriage mass in kg
        m = 30
        # Radius of pulley in meters
        r = 0.019
        # Gear ratio
        G = 14.67 / 1.0

        self._gearbox = frccnt.models.gearbox(frccnt.models.MOTOR_775PRO, num_motors)
        return frccnt.models.elevatorVelocityMode(frccnt.models.MOTOR_775PRO, num_motors, m, r, G)

    def design_controller_observer(self):

        print("desiging lqr")

        q = [0.01]
        r = [12.0]
        self.design_lqr(q, r)
        self.design_two_state_feedforward(q, r)

        # q_pos = 0.05
        q_vel = 0.04
        r_pos = 0.0001
        self.design_kalman_filter([q_vel], [r_pos])

    def calcGains(self):
        kp = self.K[0, 0] # volts per meter
        kp = kp / 12 * 1023
        rotation = 0.019 * math.pi
        # volts per meter times meters per rot is volts per rot
        kp = kp * rotation
        # volts per rotation div native unit per rotation is volts per native unit
        kp = kp / (4096)

        # raw units per 100ms
        kp = kp * 10
        print(kp)
        # print(self.Kff)

        # voltage to hold us up
        # print(_gearbox.free_speed / 12.0)
        # print(_gearbox.stall_torque / 12.0)

        return kp

def main():
    dt = 0.001
    elevator = Elevator(dt)
    # elevator.export_cpp_coeffs("Elevator", "subsystems/")
    # elevator.export_java_coeffs("Elevator")

    elevator.calcGains()

    if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
        try:
            import slycot

            plt.figure(1)
            elevator.plot_pzmaps()
        except ImportError:  # Slycot unavailable. Can't show pzmaps.
            pass
    if "--save-plots" in sys.argv:
        plt.savefig("elevator_pzmaps.svg")

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 5.0, dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0.0], [0.0]])
        elif t[i] < l1:
            r = np.array([[1.524], [0.0]])
        else:
            r = np.array([[0.0], [0.0]])
        refs.append(r)

    if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
        plt.figure(2)
        x_rec, ref_rec, u_rec = elevator.generate_time_responses(t, refs)
        elevator.plot_time_responses(t, x_rec, ref_rec, u_rec)
    if "--save-plots" in sys.argv:
        plt.savefig("elevator_response.svg")
    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
