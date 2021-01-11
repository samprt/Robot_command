from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def draw_buoy(x, d0):
    clear(ax)
    x = x.flatten()
    plot([-10, 10], [0, 0], 'black', linewidth=1)
    d = x[0]
    P = array([[-ech, -1.8 * ech], [ech, -1.8 * ech], [ech, 0], [-ech, 0]])
    draw_polygon(P, ax, 'blue')
    plot([0, L, L, L / 2, L / 2, L / 2, 0, 0],
         [-L - d, -L - d, -d, -d, 2 - d, -d, -d, -L - d], 'black', linewidth=3)
    b = -x[2]
    P = array([[0, -L - d + L], [L, -L - d + L], [L, -L / 2 - L * b / 2 - d], [0, -L / 2 - L * b / 2 - d]])
    draw_polygon(P, ax, 'white')
    plot(-1, -d0, 'o', color='red', linewidth=3)


def clip(val, min, max):
    if val > max:
        return max
    if val < min:
        return min
    return val


def f(x, u):
    d, v, b = x[0, 0], x[1, 0], x[2, 0]
    vdot = g - (g/(L * (1 + beta * b)))*max(0, L + min(d, 0)) - (v*abs(v)*Cx)/(2*L*(1 + beta * b))
    return array([[v], [vdot], [u]])


def sliding_mode_control(x, d0, dd0):
    d, v = x[0, 0], x[1, 0]
    s = (dd0 - v) + (d0 - d)
    u = K*sign(s)
    return u


ech = 5
x = array([[3], [0], [0]])
L = 1  # length of the cube
g, beta, Cx, dt = 9.81, 0.1, 1, 0.05
K = 10
ax = init_figure(-ech, ech, -1.8 * ech, 0.2 * ech)
for t in arange(0, 10, dt):
    # d0 = 3 + sin(t/2)
    # dd0 = cos(t)/2
    d0 = 5
    dd0 = 0
    u = sliding_mode_control(x, d0, dd0)
    x = x + dt * f(x, u)
    x[2, 0] = clip(x[2, 0], -1, 1)
    draw_buoy(x, d0)
    pause(0.001)
pause(1)
