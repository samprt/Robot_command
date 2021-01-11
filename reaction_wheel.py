from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def draw(ap, aw):
    aw = -aw - ap
    c = 2 * array([-sin(ap), cos(ap)], dtype=object)
    plot([0, c[0]], [0, c[1]], 'magenta', linewidth=2)
    for i in arange(0, 8):
        plot(c[0] + array([0, cos(aw + i * pi / 4)], dtype=object),
             c[1] + array([0, sin(aw + i * pi / 4)], dtype=object),
             'blue')
    pause(0.01)


def f(x, u):
    x = x.flatten()
    return array([[x[1]], [a1 * sin(x[0]) - b1 * u], [-a1 * sin(x[0]) + c1 * u]])


a1, b1, c1 = 10, 1, 2
dt = 0.02
x = array([[0.5], [0], [0]])
aw = 0  # wheel angle
ax = init_figure(-3, 3, -3, 3)
for t in arange(0, 10, dt):
    # u = (1/b1)*(a1*sin(x[0, 0]) + x[0, 0])
    u = 30*x[0, 0]/(2*c1 - 2*b1)
    x = x + f(x, u) * dt
    aw = aw + dt * x[2]
    clear(ax)
    draw(x[0], aw)
