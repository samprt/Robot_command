from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def f(x, u):
    θ = x[2, 0]
    return array([[cos(θ)], [sin(θ)], [u]])


def control(x, θd):
    e = sawtooth(θd - x[2, 0])
    u = e
    return u


x = array([[-20], [-10], [4]])
u = 1
dt = 0.1
a, b = array([[-30], [-4]]), array([[30], [6]])
ax = init_figure(-40, 40, -40, 40)

for t in arange(0, 15, dt):
    clear(ax)
    draw_tank(x, 'darkblue')
    plot2D(hstack((a, b)), 'red')
    plot2D(a, 'ro')
    plot2D(b, 'ro')
    cap = float(arctan2((b - a)[1], (b - a)[0]) -
                arctan(det(hstack(((b - a)/norm(b - a), x[0:2, 0].reshape(2, 1) - a)))))
    draw_arrow(x[0, 0], x[1, 0], cap, 4, 'red')
    u = control(x, cap)
    x = x + dt * f(x, u)
