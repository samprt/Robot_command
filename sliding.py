from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def f(x, u):
    x = x.flatten()
    u = u.flatten()
    return array([[x[3] * cos(x[2])], [x[3] * sin(x[2])], [u[0]], [u[1]]])


def control(x, w, dw, ddw):
    x1, x2, x3, x4 = x.flatten()
    A = array([[-x4 * sin(x3), cos(x3)],
               [x4 * cos(x3), sin(x3)]])
    xdot = array([[x4*cos(x3)], [x4*sin(x3)]])
    v = 5 * (w - x[0:2]) + 2 * (dw - xdot) + ddw
    u = inv(A) @ v
    return u


def sliding(x, w, dw):
    x1, x2, x3, x4 = x.flatten()
    K = 100
    A = array([[-x4 * sin(x3), cos(x3)],
               [x4 * cos(x3), sin(x3)]])
    xdot = array([[x4*cos(x3)], [x4*sin(x3)]])
    s = dw - xdot + w - x[0:2]
    v = K * sign(s)
    u = inv(A) @ v
    return u


ax = init_figure(-30, 30, -30, 30)
dt = 0.02
x1 = array([[10], [0], [1], [1]])
x2 = array([[10], [0], [1], [1]])
u1 = array([[0], [0]])
u2 = array([[0], [0]])
L = 10
s = arange(0, 2 * pi, 0.01)
for t in arange(0, 10, dt):
    clear(ax)
    n = 3
    plot(L * cos(s), L * sin(n * s), color='magenta')
    draw_tank(x1, 'red')
    draw_tank(x2, 'green')
    w = 10 * array([[cos(t)], [sin(n * t)]])
    dw = 10 * array([[-sin(t)], [n * cos(n * t)]])
    ddw = 10 * array([[-cos(t)], [-n ** 2 * sin(n * t)]])
    u1 = control(x1, w, dw, ddw)
    u2 = sliding(x2, w, dw)
    draw_disk(w, 0.5, ax, "red")
    x1 = x1 + dt * f(x1, u1)
    x2 = x2 + dt * f(x2, u2)
