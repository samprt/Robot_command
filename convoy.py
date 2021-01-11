from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def f(x, u):
    x, u = x.flatten(), u.flatten()
    xdot = array([[x[3] * cos(x[2])], [x[3] * sin(x[2])],
                  [u[0]], [u[1]], [x[3]]])
    return xdot


def control(x, w, dw, ddw):
    x1, x2, x3, x4 = x.flatten()[0:4]
    A = array([[-x4 * sin(x3), cos(x3)],
               [x4 * cos(x3), sin(x3)]])
    xdot = array([[x4 * cos(x3)], [x4 * sin(x3)]])
    v = 5 * (w - x[0:2]) + 2 * (dw - xdot) + ddw
    u = inv(A) @ v
    return u


ax = init_figure(-30, 30, -30, 30)
xa = array([[10], [0], [1], [1], [0]])
m = 6
X = array([4 * arange(0, m), zeros(m), ones(m), 3 * ones(m), zeros(m)])
Lx, Ly, omega, ds = 20, 5, 0.1, 0.1
e = np.linspace(0., 2 * pi, 30)
p = array([[Lx * cos(e)], [Ly * sin(e)]])
S = [zeros((3, 1)) for t in range(int(m * 5 / ds))]
print(len(S))
dt = 0.05
w = zeros((2, 1))
dw = w
ddw = w
for t in arange(0, 20, dt):
    clear(ax)
    wa = array([[Lx * sin(omega * t)], [Ly * cos(omega * t)]])
    dwa = array([[Lx * omega * cos(omega * t)], [-Ly * omega * sin(omega * t)]])
    ddwa = -omega ** 2 * wa
    ua = control(xa, wa, dwa, ddwa)
    plot(wa[0][0], wa[1][0], 'ro')
    plot(p[0][0], p[1][0])
    draw_tank(xa, 'blue')
    xa = xa + dt * f(xa, ua)
    xa[-1, 0] = xa[-1, 0] + xa[-2, 0] * dt
    if xa[-1, 0] > ds:
        xa[-1, 0] = 0
    for i in range(m):
        if xa[-1, 0] == 0:
            for j in range(1, len(S)):
                S[j - 1] = S[j]
            S[len(S) - 1] = xa[0:3]
        w = S[-int(5 / ds * i)][0:2]
        dw = zeros((2, 1))
        ddw = zeros((2, 1))
        x = X[:, i].reshape(5, 1)
        ui = control(x, w, dw, ddw)
        draw_tank(x, 'black')
        x = x + f(x, ui) * dt
        X[:, i] = x.flatten()
pause(1)
