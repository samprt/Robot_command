from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def draw_rob(x, col):
    x = x.flatten()
    lx, ly, ψ = x[0], x[1], x[2]
    R = Rlatlong(lx, ly) @ eulermat(0, 0, ψ)
    draw_robot3D(ax, latlong2cart(ρ, lx, ly), R, col, 1)


def f(x, u, escaper):
    x = x.flatten()
    lx, ly, ψ = x[0], x[1], x[2]
    if escaper:
        return array([[cos(ψ) / (ρ * cos(ly))], [sin(ψ) / ρ], [u]], dtype=object)
    return array([[2 * cos(ψ) / (ρ * cos(ly))], [2 * sin(ψ) / ρ], [u]], dtype=object)


def control(x, xa):
    dx = xa - x
    a1 = array([[cos(x[2, 0])],
                [sin(x[2, 0])]])
    a1 = a1 / norm(a1)
    a2 = array([[dx[0, 0] * cos(x[1, 0])],
                [dx[1, 0]]])
    a2 = a2 / norm(a2)
    a = hstack((a1, a2))
    u = det(a)
    return u


ρ = 30
ax = Axes3D(figure())
x = array([[-2], [0], [0.3]])
xa = 2 * pi * rand(3, 1) - pi
dt = 0.1

for t in arange(0, 100, dt):
    clean3D(ax, -ρ, ρ, -ρ, ρ, -ρ, ρ)
    draw_earth3D(ax, ρ, eye(3), 'gray')
    u = control(x, xa)
    x = x + dt * f(x, u, False)
    xa = xa + dt * f(xa, 0.1 * float(randn(1)), True)
    draw_rob(x, "red")
    draw_rob(xa, "blue")
    # draw_earth3D(ax,ρ,eye(3),'gray')
    pause(0.001)

pause(1)
