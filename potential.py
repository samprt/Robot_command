from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def f(x, u):
    x, u = x.flatten(), u.flatten()
    v, θ = x[2], x[3]
    return array([[v * cos(θ)], [v * sin(θ)], [u[0]], [u[1]]])


def f1(x1, x2):
    dqx = abs(x1 - qhat[0, 0])
    dqy = abs(x2 - qhat[1, 0])
    x = 5 * (x1 - qhat[0, 0]) / (sqrt(dqx ** 2 + dqy ** 2) ** 3) - vhat[0, 0] - 2 * (x1 - phat[0, 0])
    y = 5 * (x2 - qhat[1, 0]) / (sqrt(dqx ** 2 + dqy ** 2) ** 3) - vhat[1, 0] - 2 * (x2 - phat[1, 0])
    return x, y


def control(x):
    x = x.flatten()
    v, θ = x[2], x[3]
    kv = 0.05
    dir = f1(x[0], x[1])
    theta_bar = arctan2(dir[1], dir[0])
    u1 = kv * (norm(dir) - v)
    u2 = 5 * sawtooth(theta_bar - θ)
    return array([[u1], [u2]])


x = array([[4, 4, 1, 2]]).T  # x,y,v,θ
dt = 0.1
s = 5
ax = init_figure(-s, s, -s, s)

# Goal
phat = array([[2], [2]])
vhat = array([[-1], [-1]])

# Obstacle
qhat = array([[-1], [0]])

for t in arange(0, 8, dt):
    clear(ax)

    draw_disk(ax, phat, 0.2, "green")  # Draw goal

    draw_disk(ax, qhat, 0.3, "magenta")  # Draw obstacle

    draw_tank(x[[0, 1, 3]], 'red', 0.2)  # Draw robot

    draw_field(ax, f1, -s, s, -s, s, 0.4)  # Draw field

    u = control(x)
    x = x + dt * f(x, u)

    phat = phat + dt * vhat

pause(1)
