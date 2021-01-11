from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

d = 1


def f(x, u):
    x = x.flatten()
    u = u.flatten()
    return array([[u[0] * cos(x[2])], [u[0] * sin(x[2])], [u[1]]])


def control(xa, xb, v):
    x1, x2, y1, y2, θ1, θ2 = xa[0, 0], xb[0, 0], xa[1, 0], xb[1, 0], xa[2, 0], xb[2, 0]
    w = array([[x2 - d * cos(θ2)],
               [y2 - d * sin(θ2)]])
    dxb = f(xb, v)
    dx2, dy2 = dxb[0, 0], dxb[1, 0]
    dw = array([[dx2 + d * sin(θ2) * v[1, 0]],
                [dy2 - d * cos(θ2) * v[1, 0]]])

    return u


ax = init_figure(-30, 30, -30, 30)
dt = 0.1

xa = array([[-10], [-10], [0]])
xb = array([[-5], [-5], [0]])

for t in arange(0, 10, dt):
    clear(ax)
    v = array([[3], [0.5 * sin(0.2 * t)]])
    u = control(xa, xb, v)
    draw_tank(xa, 'blue')
    draw_tank(xb, 'red')
    xa = xa + dt * f(xa, u)
    xb = xb + dt * f(xb, v)
# show()
