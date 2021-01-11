from roblib import *

fig = figure()
ax = Axes3D(fig)

g = 9.81
ω1, ω2 = 100, 100
β1, β4 = 0.02, 0.002
β2, β3 = β1 / 10, β1 / 10
δ1 = β1 / 5


class Helico:

    def __init__(self):
        self.l = 2
        self.I = array([[10, 0, 0],
                        [0, 20, 0],
                        [0, 0, 20]])
        self.B = array([[β1 * ω1 ** 2, 0, 0, 0],
                        [0, β2 * ω1 ** 2, 0, 0],
                        [0, 0, β3 * ω1 ** 2, 0],
                        [-δ1 * ω1 ** 2, 0, 0, -β4 * self.l * ω2 ** 2]])
        self.p = array([[0], [0], [-10]])
        self.R = eulermat(0, 0, 0.4)
        self.vr = array([[13], [0], [0]])
        self.wr = array([[0], [0], [0]])
        self.α = array([[0, 0]]).T
        self.m = 10
        self.dt = 0.01

        self.state = "stationary"

    def draw(self, ax):
        Ca1 = hstack((circle3H(0.7 * self.l), [[0.7 * self.l, -0.7 * self.l], [0, 0], [0, 0], [1, 1]]))  
        # the disc + the blade
        Ca2 = hstack((circle3H(0.2 * self.l), [[0.2 * self.l, -0.2 * self.l], [0, 0], [0, 0], [1, 1]]))  
        # the disc + the blades
        T = tran3H(*self.p) @ ToH(self.R)
        C1 = T @ tran3H(0, 0, -self.l / 4) @ eulerH(0, 0, -self.α[0]) @ Ca1
        C2 = T @ tran3H(-self.l, 0, 0) @ eulerH(pi / 2, 0, 0) @ eulerH(0, 0, self.α[1]) @ Ca2
        M = T @ add1(array([[-self.l, 0, 0], [0, 0, 0], [0, 0, -self.l / 4]]))
        draw3H(ax, M, 'black', True, -1)  # body
        draw3H(ax, C2, 'green', True, -1)
        draw3H(ax, C1, 'blue', True, -1)

    def clock(self, u):
        τ = self.B @ u.flatten()
        self.p = self.p + self.dt * self.R @ self.vr
        self.vr = self.vr + self.dt * (-adjoint(self.wr) @ self.vr + self.R.T @ array([[0], [0], [g]])
                                       + array([[0], [0], [-τ[0] / self.m]]))
        self.R = self.R @ expm(self.dt * adjoint(self.wr))
        self.R = projSO3(self.R)
        self.wr = self.wr + self.dt * (inv(self.I) @ (-adjoint(self.wr) @ self.I @ self.wr + τ[1:4].reshape(3, 1)))

    def advanced_control(self, τ0d, φd, θd, ψd):
        Rd = eulermat(φd, θd, ψd)
        wrd = self.R.T @ adjoint_inv(logm(Rd @ self.R.T))
        τrd = self.I @ (100 * (wrd - self.wr)) + adjoint(self.wr) @ (self.I @ self.wr)
        τd = vstack((τ0d, τrd))
        u = inv(self.B) @ τd
        return u

    def phase_1(self):
        print("Phase 1")
        φd, θd, ψd = 0, -0.1, 0.4
        τ0d = 300
        for i in arange(0, 0.5, self.dt):
            u = self.advanced_control(τ0d, φd, θd, ψd)
            self.clock(u)
            clean3D(ax, -20, 20, -20, 20, 0, 40)
            self.draw(ax)
            pause(0.001)

    def looping_2(self):
        print("Looping")
        τ0d = 0
        φd, θd, ψd = 0, 0, 0.4
        for t in arange(0, 1, self.dt):
            u = self.advanced_control(τ0d, φd, θd, ψd)
            u[2, 0] = 10
            self.clock(u)
            clean3D(ax, -20, 20, -20, 20, 0, 40)
            self.draw(ax)
            pause(0.001)
        θ = eulermat2angles(self.R)[1]
        τ0d = 100
        φd, θd, ψd = 0, 0.6, 0.4
        while θ < 0:
            u = self.advanced_control(τ0d, φd, θd, ψd)
            self.clock(u)
            θ = eulermat2angles(self.R)[1]
            clean3D(ax, -20, 20, -20, 20, 0, 40)
            self.draw(ax)
            pause(0.001)

    def stationary(self):
        print("Stationary")
        τ0d = 100
        φd, θd, ψd = 0, 0, 0.4
        for t in arange(0, 3, self.dt):
            u = self.advanced_control(τ0d, φd, θd, ψd)
            self.clock(u)
            clean3D(ax, -20, 20, -20, 20, 0, 40)
            self.draw(ax)
            pause(0.001)


h = Helico()
t0 = time.time()
h.phase_1()
# print("end_phase1 after", time.time() - t0, 'second at', h.p[2, 0], "m")
h.looping_2()
h.stationary()
pause(10)
