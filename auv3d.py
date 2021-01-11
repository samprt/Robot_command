from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def draw(x, yd):
    clean3D(ax, -40, 40, -40, 40, -40, 40)
    draw_axis3D(ax, 0, 0, 0, eye(3, 3), 10)
    ax.scatter(yd[0, 0], yd[1, 0], yd[2, 0], color='red')
    draw_robot3D(ax, x[0:3], eulermat(*x[4:7, 0]), 'blue')


def f(x, u):
    x, u = x.flatten(), u.flatten()
    v, φ, θ, ψ = x[3], x[4], x[5], x[6]
    cφ, sφ, cθ, sθ, cψ, sψ = cos(φ), sin(φ), cos(θ), sin(θ), cos(ψ), sin(ψ)
    return array([[v * cθ * cψ], [v * cθ * sψ], [-v * sθ], [u[0]],
                  [-0.1 * sφ * cθ + tan(θ) * v * (sφ * u[1] + cφ * u[2])],
                  [cφ * v * u[1] - sφ * v * u[2]],
                  [(sφ / cθ) * v * u[1] + (cφ / cθ) * v * u[2]]])


def A(x):
    x = x.flatten()
    v, φ, θ, ψ = x[3], x[4], x[5], x[6]
    cφ, sφ, cθ, sθ, cψ, sψ = cos(φ), sin(φ), cos(θ), sin(θ), cos(ψ), sin(ψ)
    A1 = array([[cθ * cψ, -v * sθ * cψ, -v * cθ * sψ],
                [cθ * sψ, -v * sθ * cψ, v * cθ * cψ],
                [-sθ, -v * cθ, 0]])
    A2 = array([[1, 0, 0],
                [0, v * cφ, -v * sφ],
                [0, v * (sφ / cθ), v * (cφ / cθ)]])
    return A1 @ A2


x = array([[0, 0, 10, 15, 1, 1, 1]]).T
u = array([[0, 0, 0]]).T
f1, f2, f3, R = 0.01, 0.06, 0.03, 20
dt = 0.05
ax = Axes3D(figure())
for t in arange(0, 10, dt):
    yd = array([[R*sin(f1*t) + R*sin(f2*t)],
                [R*cos(f1*t) + R*cos(f2*t)],
                [R*sin(f3*t)]])
    dyd = array([[R*f1*cos(f1*t) + R*f2*cos(f2*t)],
                 [-R*f1*sin(f1*t) - R*f2*sin(f2*t)],
                 [R*f3*cos(f3*t)]])
    d2yd = array([[-R*f1**2*sin(f1*t) - R*f2**2*sin(f2*t)],
                  [-R*f1**2*cos(f1*t) - R*f2**2*cos(f2*t)],
                  [-R*f3**2*sin(f3*t)]])
    xdot = f(x, u)
    v = 0.04*(yd - x[0:3]) + 0.4*(dyd - xdot[0:3]) + d2yd
    u = inv(A(x))@v

    x = x + dt * xdot
    draw(x, yd)
    pause(0.001)
pause(1)
