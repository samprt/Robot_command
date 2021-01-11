from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def f(x, u):
    x, u = x.flatten(), u.flatten()
    xdot = array([[x[3] * cos(x[2])], [x[3] * sin(x[2])], [u[0]], [u[1]]])
    return (xdot)


def control(x, w, dw, ddw):
    x1, x2, x3, x4 = x.flatten()
    A = array([[-x4 * sin(x3), cos(x3)],
               [x4 * cos(x3), sin(x3)]])
    xdot = array([[x4*cos(x3)], [x4*sin(x3)]])
    v = 5 * (w - x[0:2]) + 2 * (dw - xdot) + ddw
    u = inv(A) @ v
    return u


ax = init_figure(-50, 50, -50, 50)
m = 20
X = 10 * randn(4, m)
a, dt = 0.1, 0.1

for t in arange(0, 15, dt):
    clear(ax)
    for i in range(m):
        θ = a*t
        w = array([[cos(θ + (2*i*pi)/m)], [sin(θ + (2*i*pi)/m)]])
        dw = array([[-a*sin(θ + (2*i*pi)/m)], [a*cos(θ + (2*i*pi)/m)]])
        ddw = array([[-a**2*cos(θ + (2*i*pi)/m)], [-a**2*sin(θ + (2*i*pi)/m)]])
        R = array([[cos(θ), -sin(θ)],
                   [sin(θ), cos(θ)]])
        Rd = a*array([[-sin(θ), -cos(θ)],
                      [cos(θ), -sin(θ)]])
        Rdd = a**2*array([[-cos(θ), sin(θ)],
                          [-sin(θ), -cos(θ)]])
        E = diag((20+15*sin(θ), 20))
        Ed = a*diag((15*cos(θ), 0))
        Edd = a**2*diag((-15*sin(θ), 0))
        c = R@E@w
        cd = Rd@E@w + R@Ed@w + R@E@dw
        cdd = Rdd@E@w + R@Edd@w + R@E@ddw + 2*(Rd@Ed@w + Rd@E@dw + R@Ed@dw)
        x = X[:, i].reshape(4, 1)
        u = control(x, c, cd, cdd)
        x = X[:, i].reshape(4, 1)
        draw_tank(x, 'b')
        x = x + f(x, u) * dt
        X[:, i] = x.flatten()
        plot([c[0][0]], [c[1][0]], 'r+')
