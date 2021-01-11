from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


# La deuxième voiture n'a pas le comportement voulu. Cela vient d'erreurs dans mon calcul de la commande ub.


def f(x, u):
    xr, yr, θr, vr = x.flatten()
    u1, u2 = u.flatten()
    return array([[vr * cos(θr)], [vr * sin(θr)], [u1], [u2]])


ax = init_figure(-30, 30, -30, 30)

dt = 0.1
x1 = array([[0], [1], [pi / 3], [1]])
x2 = array([[2], [-2], [pi], [1]])
u = array([[1], [1]])
w, Lx, Ly, l = 0.1, 15, 7, 1

for t in arange(0, 30, dt):
    clear(ax)
    xa, ya, θa, va = x1.flatten()
    xb, yb, θb, vb = x2.flatten()

    xda = Lx * sin(w * t)
    dxda = Lx * w * cos(w * t)
    d2xda = -Lx * w ** 2 * sin(w * t)

    yda = Ly * cos(w * t)
    dyda = -Lx * w * sin(w * t)
    d2yda = -Lx * w ** 2 * cos(w * t)

    dxa = va * cos(θa)
    dya = va * sin(θa)

    v1 = xda - xa + 2 * (dxda - dxa) - d2xda
    v2 = yda - ya + 2 * (dyda - dya) - d2yda
    va = array([[v1], [v2]])

    Aa = array([[-dya, cos(θa)],
                [dxa, sin(θa)]])
    ua = inv(Aa) @ va

    xdb = xa - l * cos(θa)
    dxdb = dxa + l * sin(θa) * ua[0, 0]

    ydb = ya - l * sin(θa)
    dydb = dya - l * cos(θa) * ua[0, 0]

    wb = array([[xa - cos(θa)], [ya - sin(θa)]])
    dwb = array([[dxa + dya], [dya - dxa]])

    Ab = array([[-vb * sin(θb), cos(θb)],
                [vb * cos(θb), sin(θa)]])

    vb = (wb - array([[xb], [yb]])) + 2 * (dwb - array([[xdb], [ydb]]))

    ub = inv(Ab) @ vb

    draw_tank(x1)
    draw_tank(x2)
    x1 = x1 + dt * f(x1, ua)
    x2 = x2 + dt * f(x2, ub)

pause(1)
