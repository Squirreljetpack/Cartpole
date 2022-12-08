from math import *
import numpy as np
import scipy.integrate as si
import matplotlib.pyplot as plt
import random
from sim import CartPole

#Configures 
T = 20
N = T * 120

class CartPole2(CartPole):
    def dstate(self, t, state):
        x, dx, t, dt = state
        ddt = self.ddt_calc(t, dt, 0)
        ddx = self.ddx_calc(t, dt, 0)
        return np.array([dx, ddx, dt, ddt])

    def dstate_linear(self, t, state):
        return self.A_lin @ state

times = np.linspace(0, T, N)
t_span = (0.0, T)

# Define constants and initial conditions here
system = CartPole2(
    init_state=[0, 0.0, np.pi, 0.2],
    constants={
        "massCart": 1,
        "massPole": 0.15,
        "poleLength": 2.5,
        "dissipation": 0,
        "downwards": 1,
    },
)

result_solve_ivp = si.solve_ivp(
    system.dstate_linear, t_span, system.state, t_eval=times
)
y1 = result_solve_ivp.y
result_solve_ivp = si.solve_ivp(system.dstate, t_span, system.state, t_eval=times)


def td(l):
    return list(map(lambda x: degrees(x) % 360, l[2]))


def tr(l):
    return list(map(lambda x: radians(degrees(x)), l[2]))


if __name__ == "__main__":
    # Plot time-series of solution
    f = plt.figure()
    f.set_figwidth(20)
    plt.plot(times, td(y1), "-r", lw=1)
    # plt.plot(times, y[0, :], "-b", times, tr, "-r", lw=1)
    plt.title("Linearization near θ=0 time series")
    plt.xlabel("time (seconds)")
    plt.ylabel("solution")
    plt.legend(["x (meters)", "θ (degrees)"])
    plt.grid()
    plt.savefig("TimeSeriesSolution1.png")

    # Plot phase-space
    f = plt.figure()
    f.set_figwidth(20)
    plt.plot(y1[0, :], td(y1), "-b", lw=1)
    plt.title("Linearization near θ=0 phase space")
    plt.xlabel("x")
    plt.ylabel("θ (degrees)")
    plt.grid()
    plt.savefig("PhaseSpace.png")
