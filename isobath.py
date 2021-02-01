# https://www.ensta-bretagne.fr/jaulin/robmooc.html
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def h(x, y):
    return 2 * exp(-((x + 2) ** 2 + (y + 2) ** 2) / 10) + 2 * exp(-((x - 2) ** 2 + (y - 2) ** 2) / 10) - 10


def grad_h(x, y):
    d = 0.1
    grad_h_x = (h(x + d, y) - h(x - d, y)) / (2 * d)
    grad_h_y = (h(x, y + d) - h(x, y - d)) / (2 * d)
    return array([[grad_h_x, grad_h_y]]).T


def draw_mesh():
    Mx = arange(-L, L, 1.5)
    X, Y = meshgrid(Mx, Mx)
    H = h(X, Y)
    ax.plot_surface(X, Y, H)
    # ax.contour(X,Y,H)
    return ()


def f(x, u):
    v = 5
    x, y, z, psi = x.flatten()
    x_dot = v * cos(psi)
    y_dot = v * sin(psi)
    v_dot = u[0]
    psi_dot = u[1]
    return array([[x_dot, y_dot, v_dot, psi_dot]]).T


def g(x):
    x, y, z, psi = x.flatten()
    y1 = z - h(x, y)
    y2 = angle(grad_h(x, y)) - psi
    y3 = -z
    return array([[y1, y2, y3]]).T


def control(x, depth, altitude):
    psi = x[3, 0]
    y1, y2, y3 = g(x).flatten()
    u1 = 1 * (y3 - depth)
    u2 = sawtooth(y2 + pi / 2) + y2 * (altitude - y1)
    return array([u1, u2])


ax = Axes3D(figure())
x = array([[0.0, -1.0, -2.0, 0.0]]).T  # x,y,z,ψ
L = 10  # size of the world
dt = 0.01
depth, altitude = 3, 6
for t in arange(0, 10, dt):
    if t == 4:
        depth = 4
    u = control(x, depth, altitude)
    print("Depth =", g(x)[2, 0], "   Altitude =", g(x)[0, 0])
    x += dt * f(x, u)
    clean3D(ax)
    draw_mesh()
    draw_robot3D(ax, x[0:3], eulermat(0, 0, x[3, 0]), 'red', 0.1)
    pause(0.001)
pause(10)
