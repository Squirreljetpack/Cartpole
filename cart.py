from math import *
import numpy as np
import scipy.integrate as si
import matplotlib.pyplot as plt
import random

T = 20
N = T * 120


class CartPole:
    g = 9.81
    massCart = 1
    massPole = 0.1
    poleLength = 2.5
    tau = 0.01
    forceMag = 30
    # variables in the linear approximation
    dissipation = 0
    downwards = -1

    def __init__(self, init_state=None, constants=None, sim="default"):
        if constants:
            self.massCart = constants["massCart"]
            self.massPole = constants["massPole"]
            self.poleLength = constants["poleLength"]
            self.dissipation = constants["dissipation"]
        if init_state:
            self.state = init_state
        else:
            self.state = [
                random.random() - 0.5,
                (random.random() - 0.5) * 1,
                radians(random.random() * 360),
                (random.random() - 0.5) * 0.5,
            ]
        self.sim = sim

        # Matrices of the linear approximation of state-space model
        self.A_lin = np.array(
            [
                [0, 1, 0, 0],
                [
                    0,
                    -self.dissipation / self.massCart,
                    -self.massPole * self.g / self.massCart,
                    0,
                ],
                [0, 0, 0, 1],
                [
                    0,
                    -self.downwards
                    * self.dissipation
                    / (self.massCart * self.poleLength),
                    -self.downwards
                    * (self.massPole + self.massCart)
                    * self.g
                    / (self.poleLength * self.massCart),
                    0,
                ],
            ]
        )
        # nx1
        self.B_lin = np.array(
            [
                0,
                1 / self.massCart,
                0,
                self.downwards * -1 / (self.massCart * self.poleLength),
            ]
        )[:, np.newaxis]

    def ddt_calc(self, t, dt, f):
        num = (
            f * cos(t)
            - self.massPole * self.poleLength * cos(t) * sin(t) * dt**2
            + (self.massPole + self.massCart) * self.g * sin(t)
        )
        denum = self.poleLength * (self.massCart + self.massPole * sin(t) ** 2)
        return num / denum

    def ddx_calc(self, t, dt, f):
        num = f + self.massPole * sin(t) * (self.g * cos(t) - self.poleLength * dt**2)
        denum = self.massCart + self.massPole * sin(t) ** 2
        return num / denum

    def dstate_linear(self, tau, state, force=0):
        A = np.array(
            [
                [0, 1, 0, 0],
                [0, 0, -self.massPole * self.g / self.massCart, 0],
                [0, 0, 0, 1],
                [
                    0,
                    0,
                    -(self.massPole + self.massCart)
                    * (self.g / self.poleLength * self.massCart),
                    0,
                ],
            ]
        )
        B = np.array([0, self.massCart, 0, -1 / self.massCart * self.poleLength])
        print(A)
        print(state)
        return A @ state + B * force

    def dstate(self, tau, state, force=0):
        x, dx, t, dt = state
        ddt = self.ddt_calc(t, dt, force)
        ddx = self.ddx_calc(t, dt, force)

        return np.array([dx, ddx, dt, ddt])


times = np.linspace(0, T, N)
t_span = (0.0, T)

system = CartPole(
    init_state=[0, 0.0, 0, 0.2],
    constants={
        "massCart": 1,
        "massPole": 0.15,
        "poleLength": 2.5,
        "dissipation": 0,
    },
)
system2 = CartPole(
    init_state=[0, 0.5, np.pi, 1],
    constants={
        "massCart": 1,
        "massPole": 0.15,
        "poleLength": 2.5,
        "dissipation": 0,
    },
)
system3 = CartPole(
    init_state=[0, 0.0, np.pi, 4.5],
    constants={
        "massCart": 1,
        "massPole": 0.15,
        "poleLength": 2.5,
        "dissipation": 0,
    },
)

result_solve_ivp = si.solve_ivp(
    system.dstate_linear, t_span, system.state, t_eval=times
)
y1 = result_solve_ivp.y
result_solve_ivp = si.solve_ivp(system2.dstate, t_span, system2.state, t_eval=times)
y2 = result_solve_ivp.y
result_solve_ivp = si.solve_ivp(system3.dstate, t_span, system3.state, t_eval=times)
y3 = result_solve_ivp.y


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

    # # Plot phase-space
    # f = plt.figure()
    # f.set_figwidth(20)
    # plt.plot(y1[0, :], td(y1), "-b", lw=1)
    # plt.plot(y2[0, :], td(y2), "-r", lw=1)
    # plt.plot(y3[0, :], td(y3), "-g", lw=1)
    # plt.title("Initial velocity comparison")
    # plt.xlabel("x")
    # plt.ylabel("θ (degrees)")
    # plt.legend(
    #     [
    #         "w0 = 0.2",
    #         "w0 = 1.0",
    #         "w0 = 4.5",
    #     ]
    # )
    # plt.grid()
    # plt.savefig("PhaseSpace.png")

    # f = plt.figure()
    # f.set_figwidth(20)
    # # plt.plot(times, t, "-r", lw=1)
    # # plt.plot(times, y[0, :], "-b", times, tr, "-r", lw=1)
    # plt.plot(
    #     times,
    #     y1[0],
    #     "-b",
    #     times,
    #     tr(y1),
    #     "-r",
    #     lw=1,
    # )
    # plt.plot(
    #     times,
    #     y2[0, :],
    #     "--b",
    #     times,
    #     tr(y2),
    #     "--r",
    #     lw=1,
    # )
    # plt.plot(
    #     times,
    #     y3[0, :],
    #     ":b",
    #     times,
    #     tr(y3),
    #     ":r",
    #     lw=1,
    # )
    # plt.title("Bob mass time series comparison")
    # plt.xlabel("time (seconds)")
    # plt.ylabel("solution")
    # plt.legend(
    #     [
    #         "x (meters) (Bob mass = .15)",
    #         "θ (degrees)",
    #         "x (Bob mass = .02)",
    #         "θ",
    #         "x (Bob mass = .5)",
    #         "θ",
    #     ]
    # )
    # plt.grid()
    # plt.savefig("TimeSeriesSolution1.png")
