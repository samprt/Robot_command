from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def draw_crank(x):
    θ1 = x[0, 0]
    θ2 = x[1, 0]
    z = L1 * array([[cos(θ1)], [sin(θ1)]])
    y = z + L2 * array([[cos(θ1 + θ2)], [sin(θ1 + θ2)]])
    plot([0, z[0, 0], y[0, 0]], [0, z[1, 0], y[1, 0]], 'magenta', linewidth=2)
    draw_disk(c, r, ax, "cyan")


L1, L2 = 4, 3
c = array([[1], [2]])
r = 4
dt = 0.05

x = array([[-1], [1]])


def f(x, u):
    θ1 = x[0, 0]
    θ2 = x[1, 0]
    dθ1 = u[0, 0]
    dθ2 = u[1, 0]
    return array([[dθ1], [dθ2]])


ax = init_figure(-4, 8, -4, 8)

for t in arange(0, 10, dt):
    clear(ax)
    draw_crank(x)
    x1, x2, c1, c2 = x[0, 0], x[1, 0], c[0, 0], c[1, 0]
    y = array([[L1 * cos(x1) + L2 * cos(x1 + x2)], [L1 * sin(x1) + L2 * sin(x1 + x2)]])
    v1 = c1 + r*(cos(t)-sin(t)) - y[0, 0]
    v2 = c2 + r*(cos(t)+sin(t)) - y[1, 0]
    v = array([[v1], [v2]])
    A = array([[-y[1, 0], -L2*sin(x1+x2)],
               [y[0, 0], L2*cos(x1+x2)]])
    u = inv(A)@v
    x = x + dt * f(x, u)
