from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def draw_pools(x):
    x = x.flatten()
    plot([0, 0], [10, 1], 'black', linewidth=2)
    plot([-7, 23], [0, 0], 'black', linewidth=5)
    plot([16, 16], [1, 10], 'black', linewidth=2)
    plot([4, 4, 6, 6], [10, 1, 1, 10], 'black', linewidth=2)
    plot([10, 10, 12, 12], [10, 1, 1, 10], 'black', linewidth=2)
    P = array([[0, x[0]], [0, 1], [-6, 0], [22, 0], [16, 1], [16, x[2]], [12, x[2]], [12, 1],
               [10, 1], [10, x[1]], [6, x[1]], [6, 1], [4, 1], [4, x[0]]])
    draw_polygon(P, ax, 'blue')
    P = array([[1, 10], [1, x[0]], [1 + 0.1 * u[0], x[0]], [1 + 0.1 * u[0], 10]], dtype=object)
    draw_polygon(P, ax, 'blue')
    P = array([[13, 10], [13, x[2]], [13 + 0.1 * u[1], x[2]], [13 + 0.1 * u[1], 10]], dtype=object)
    draw_polygon(P, ax, 'blue')


def alpha(h):
    return sign(h) * sqrt(20 * norm(h))


def f(x, u):
    h1, h2, h3 = x[0, 0], x[1, 0], x[2, 0]
    h1dot = -alpha(h1) - alpha(h1 - h2) + u[0, 0]
    h2dot = alpha(h1 - h2) - alpha(h2 - h3)
    h3dot = -alpha(h3) + alpha(h2 - h3) + u[1, 0]
    return array([[h1dot], [h2dot], [h3dot]])


dt = 0.05
x = array([[4], [5], [2]])
w = array([[3], [3]])
z = array([[0], [0]])
ax = init_figure(-10, 25, -2, 12)

for t in arange(0, 5, dt):
    clear(ax)
    wold = w
    w = 3 + array([[cos(t)], [sin(t)]])
    y1, y2, w1, w2 = x[0, 0], x[2, 0], w[0, 0], w[1, 0]
    y = array([[y1], [y2]])
    z = z + dt * (w - y)
    dw = w - wold
    v = 2 * (w - y) + z + dw
    u1 = alpha(x[0, 0]) + alpha(x[0, 0] - x[1, 0]) + v[0, 0]
    u2 = alpha(x[2, 0]) - alpha(x[1, 0] - x[2, 0]) + v[1, 0]
    u = array([[u1], [u2]])
    draw_pools(x)
    x = x + dt * f(x, u)
