from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def f(x, u):
    x = x.flatten()
    θ = x[2]
    return array([[cos(θ)], [sin(θ)], [u]])


def control(x, theta_bar, direction):
    if direction == 'both':
        u = sawtooth(theta_bar - x[2, 0])
    if direction == 'left':
        u = sawtooth(theta_bar - x[2, 0] - pi) + pi
    if direction == 'right':
        u = sawtooth(theta_bar - x[2, 0] + pi) - pi
    return u


x = array([[0], [0], [0.1]])
dt = 0.1
ax = init_figure(-10, 10, -10, 10)

directions = ['both', 'left', 'right']

for i in range(3):
    theta_bar = pi / 2
    if i == 1:
        theta_bar = -pi / 2
    for t in arange(0, 5, dt):
        clear(ax)
        u = control(x, theta_bar, directions[i])
        x = x + dt * f(x, u)
        draw_tank(x, 'red', 0.3)
    pause(1)
    x = array([[0], [0], [0.1]])
