from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
import sympy as sy
from sympy.diffgeom import *


def L(F, g, i=1):
    if size(g) == 2:
        return sy.Matrix([[L(F, g[0], i)], [L(F, g[1], i)]])
    if i == 1:
        return LieDerivative(F, g)
    return L(F, L(F, g, i - 1))


def ψ(y1, y2):
    return y2, -((y1 ** 2) - 1) * y2 - y1


def build_p():
    C = CoordSystem('C', Patch('P', Manifold('M', 5)), ["x1", "x2", "x3", "x4", "x5"])
    x1, x2, x3, x4, x5 = C.coord_functions()
    E = C.base_vectors()
    v1, a2 = sy.symbols("v1 a2")
    Fx = x5 * sy.cos(x3) * E[0] + x5 * sy.sin(x3) * E[1] + x5 * sy.sin(x3 - x4) * E[3]
    Gx1, Gx2 = E[2], E[4]
    Hx1, Hx2 = x1 - sy.cos(x4), x2 - sy.sin(x4)
    z3 = x5 * sy.cos(x3 - x4)
    A = sy.Matrix([[L(Gx1, z3), L(Gx2, z3)], [L(Gx1, L(Fx, x4)), L(Gx2, L(Fx, x4))]])
    b = sy.Matrix([L(Fx, z3), L(Fx, x4, 2)])
    u = A.inv() * (sy.Matrix([v1, a2]) - b)
    p = sy.lambdify((x1, x2, x3, x4, x5, v1, a2), u)
    phi = sy.lambdify((x1, x2, x3, x4, x5, v1), sy.Matrix([Hx1, Hx2, z3, v1, x4, x5 * sy.sin(x3 - x4)]))
    return p, phi


def build_beta():
    C = CoordSystem('C', Patch('P', Manifold('M', 6)), ["z1", "z2", "z3", "z4", "z5", "z6"])
    z1, z2, z3, z4, z5, z6 = C.coord_functions()
    E = C.base_vectors()
    Fz = z3 * sy.cos(z5) * E[0] + z3 * sy.sin(z5) * E[1] + z4 * E[2] + z6 * E[4]
    Gz1, Gz2 = E[3], E[5]
    Hz = sy.Matrix([z1, z2])
    e = L(Fz, Hz) - sy.Matrix(ψ(z1, z2))
    de = L(Fz, Hz, 2) - L(Fz, ψ(z1, z2))
    Lgz = L(Gz1, L(Fz, Hz, 2)).row_join(L(Gz2, L(Fz, Hz, 2)))
    beta = sy.lambdify((z1, z2, z3, z4, z5, z6), -Lgz.inv() * (L(Fz, Hz, 3) - L(Fz, ψ(z1, z2), 2) + e + 2*de))
    return beta


p, phi = build_p()
beta = build_beta()

x = np.array([[0], [0], [0], [0], [1]])
v1, dt, sc = 0, 0.02, 3

ax = init_figure(-sc, sc, -sc, sc)

for t in arange(0, 10, dt):
    clear(ax)
    x1, x2, x3, x4, x5 = x[0:5, 0]
    draw_field(ax, ψ, -sc, sc, -sc, sc, 0.3)
    draw_tank_trailer(x1, x2, x3, x4, x5)
    z = phi(x1, x2, x3, x4, x5, v1)
    z1, z2, z3, z4, z5, z6 = z[0:6, 0]
    a = beta(z1, z2, z3, z4, z5, z6)
    a1, a2 = a[0:2, 0]
    v1 += a1 * dt
    u = p(x1, x2, x3, x4, x5, v1, a2)
    u1, u2 = u[0:2, 0]
    x = x + array([[x5 * cos(x3)], [x5 * sin(x3)], [u1], [x5 * sin(x3 - x4)], [u2]]) * dt
    pause(0.001)
pause(2)
