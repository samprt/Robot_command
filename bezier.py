from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def f(x, u):
    x = x.flatten()
    u = u.flatten()
    return array([[x[3] * cos(x[2])], [x[3] * sin(x[2])], [u[0]], [u[1]]])


def control(x, w, dw):
    x = x.flatten()
    A = array([[-x[3] * sin(x[2]), cos(x[2])],
               [x[3] * cos(x[2]), sin(x[2])]])
    y = array([[x[0]], [x[1]]])
    dy = array([[x[3] * cos(x[2])], [x[3] * sin(x[2])]])
    return inv(A) @ ((w - y) + 2 * (dw - dy))


def setpoint(t):
    waypoint = zeros((2, 1))
    for i in range(n + 1):
        waypoint += comb(n, i) * (1 - t / tmax) ** (n - i) * (t / tmax) ** i * P[:, i].reshape((2, 1))
    return waypoint


def dsetpoint(t):
    dwaypoint = zeros((2, 1))
    if t > 0:
        for i in range(n + 1):
            dwaypoint += comb(n, i) * (
                    -((n - i) / tmax) * (1 - t / tmax) ** (n - i - 1) * (t / tmax) ** i + (1 - t / tmax) ** (
                    n - i) * (i / tmax) * (t / tmax) ** (i - 1)) * P[:, i].reshape((2, 1))
    return dwaypoint


P = array([[1, 1, 1, 1, 2, 3, 4, 5, 5, 7, 7, 9, 10, 8],
           [1, 4, 7, 9, 10, 8, 6, 4, 1, 0, 0, 1, 3, 8]])

n = len(P[0]) - 1

dt, tmax = 0.1, 50

ax = init_figure(-1, 11, -1, 11)

A1 = array([[2, 0], [4, 2], [2, 7]])
A2 = array([[7, 2], [8, 3], [3, 10]])
draw_polygon(A1, ax, 'green')
draw_polygon(A2, ax, 'green')

plot(P[0], P[1], 'or')

x = array([[0, 0, 0, 1]]).T
for t in arange(0, tmax, dt):
    w = setpoint(t)
    dw = dsetpoint(t)
    u = control(x, w, dw)
    plot(w[0], w[1], 'm.')
    x = x + f(x, u) * dt
    draw_tank(x, 'darkblue', 0.2)
    pause(0.001)
pause(10)
