from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def draw(x):
    draw_tank(x, 'darkblue', 0.3)
    # a, b = array([[-30], [0]]), array([[30], [0]])
    # draw_segment(a, b, 'red', 2)


def f(x, u):
    θ = x[2, 0]
    return array([[cos(θ)], [sin(θ)], [u]])


def control_1(x):
    y = x[2, 0] + arctan(x[1, 0])
    u = -y - sin(x[2, 0]) / (1 + x[1, 0] ** 2)
    return u


def control_vdp(x):
    a, b = vdp(x[0, 0], x[1, 0])
    adot = sin(x[2, 0])
    bdot = -cos(x[2, 0]) - 0.02 * cos(x[2, 0]) * x[0, 0] * x[1, 0] - (0.01 * x[0, 0] ** 2 - 1) * sin(x[2, 0])
    y = sawtooth(x[2, 0] - arctan2(b, a))
    u = -y - ((b * adot - a * bdot) / (a ** 2 + b ** 2))
    return u


x = array([[-2], [-2], [10]])
dt = 0.05
s = 10


def f1(x1, x2):
    return cos(-arctan(x2)), sin(-arctan(x2))


def vdp(x1, x2):
    return x2, -(0.01 * (x1 ** 2) - 1) * x2 - x1


ax = init_figure(-s, s, -s, s)

for t in arange(0, 8, dt):
    clear(ax)
    draw_field(ax, vdp, -s, s, -s, s, 1)
    draw(x)
    pause(0.001)
    u = control_vdp(x)
    x = x + dt * f(x, u)
