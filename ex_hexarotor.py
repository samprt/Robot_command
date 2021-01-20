from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

static_platform = False

fig = figure()
ax = Axes3D(fig)
m, b, d, l = 10, 2, 1, 1
g = array([[0], [0], [9.81]])
I = array([[10, 0, 0],
           [0, 10, 0],
           [0, 0, 20]])
dt = 0.02
Q = array([[0, -l, 0, l, l / 2, 0],
           [l, 0, -l, 0, 0, l / 2],
           [0, 0, 0, 0, 0, 0]])  # positions of the rotors, all blades have the same pitch
D = array([[1, 0, 0, 0, 1, 0],
           [0, 0, 0, 0, 0, 1],
           [0, 1, 1, 1, 0, 0]])  # orientation of the forces
N = D.shape[1]
C = array([[1, 0, 0, 0, 1, 0],
           [0, 0, 0, 0, 0, 1],
           [0, -1, -1, -1, 0, 0],
           [0, 0, 1, 0, 0, 0],
           [0, -1, 0, 1, 0, 0],
           [-1, 0, 0, 0, 0, 0]])  # Concentrateur


def draw_hexarotor3D(ax, p, R, α, col):
    lz = 5 * l
    Ca = hstack((circle3H(0.3 * lz), [[0.3 * lz, -0.3 * lz], [0, 0], [0, 0], [1, 1]]))  # the disc + the blades
    Ca = expwH([0, pi / 2, 0]) @ Ca
    T = tran3H(*p) @ ToH(R)
    for i in range(0, N):
        ai = pi / 2 * adjoint([[1], [0], [0]]) @ D[:, i]
        Ci = T @ tran3H(*(lz * Q[:, i])) @ expwH(ai) @ eulerH(α[i], 0, 0) @ Ca
        draw3H(ax, Ci, col[i], True, -1)
    M = T @ add1([[lz, -lz, 0, 0, 0], [0, 0, 0, lz, -lz], [0, 0, 0, 0, 0]])
    draw3H(ax, M, 'grey', True, -1)


def draw_platform(ax, p, R):
    lz = 5 * l
    Ca = expwH([0, pi / 2, 0]) @ circle3H(0.3 * lz)
    T = tran3H(*p) @ ToH(R)
    for i in range(0, N - 2):
        ai = pi / 2 * adjoint([[1], [0], [0]]) @ D[:, i]
        Ci = T @ tran3H(*(lz * Q[:, i])) @ expwH(ai) @ Ca
        draw3H(ax, Ci, 'black', True, -1)
    M = T @ add1([[lz, -lz, -lz, lz, lz], [lz, lz, -lz, -lz, lz], [0, 0, 0, 0, 0]])
    draw3H(ax, M, 'grey', True, -1)


def clock_hexa(p, R, vr, wr, f):
    # Calcule de fr et τr à partir de f
    fr_τr = C @ f
    fr = fr_τr[0:3]
    τr = fr_τr[3:6]

    # Calcul des dérivées
    pdot = R @ vr
    vrdot = R.T @ g + 1 / m * fr - adjoint(wr) @ vr
    wrdot = inv(I) @ (τr - adjoint(wr) @ (I @ wr))

    # Euler
    p = p + dt * pdot
    R = R @ expm(dt * adjoint(wr))
    R = projSO3(R)
    vr = vr + dt * vrdot
    wr = wr + dt * wrdot
    return p, R, vr, wr


def pd(t):
    if static_platform:
        return array([[0], [0], [-10]])
    return array([[sin(0.3 * t)], [cos(0.4 * t)], [-10 + 0.1 * sin(0.3 * t)]])


def pd_d(t):
    if static_platform:
        return zeros((3, 1))
    return array([[0.3 * cos(0.3 * t)], [-0.4 * sin(0.4 * t)], [0.03 * cos(0.3 * t)]])


def pd_dd(t):
    if static_platform:
        return zeros((3, 1))
    return array([[-0.09 * sin(0.3 * t)], [-0.16 * cos(0.4 * t)], [-0.009 * sin(0.3 * t)]])


def Rd(t):
    if static_platform:
        return eye(3)
    return expw([[sin(t)], [cos(2 * t)], [t]])


def Rd_d(t):
    if static_platform:
        return zeros((3, 3))
    Rd_d = (1 / (2 * dt)) * (Rd(t + dt) - Rd(t - dt))
    # w = array([[sin(t)], [cos(2 * t)], [t]])
    # Rd_d = adjoint(w) @ Rd(t)
    return Rd_d


def Rd_dd(t):
    if static_platform:
        return zeros((3, 3))
    Rd_dd = (1 / (2 * dt)) * (Rd_d(t + dt) - Rd_d(t - dt))
    # w = array([[sin(t)], [cos(2 * t)], [t]])
    # Rd_dd = adjoint(w) @ adjoint(w) @ Rd(t)
    return Rd_dd


def back_stepping(vrd_dot, wrd_dot, R, vr, wr):
    frd = m * (vrd_dot - R.T @ g + adjoint(wr) @ vr)
    τrd = I @ wrd_dot + adjoint(wr) @ (I @ wr)
    frd_τrd = vstack((frd, τrd))
    f = inv(C) @ frd_τrd
    return f


def positionneur(pd, pd_d, pd_dd, p, R, vr, wr):
    vrd_dot = R.T @ pd_dd + 2 * (R.T @ pd_d - vr) + R.T @ (pd - p) - adjoint(wr) @ vr
    return vrd_dot


def orientateur(Rd, Rd_d, Rd_dd, R, wr):
    wd = adjoint_inv(Rd_d @ Rd.T)
    wd_d = adjoint_inv(Rd_d @ Rd_d.T + Rd_dd @ Rd.T)
    er = R.T @ adjoint_inv(logm(Rd @ R.T))
    er_d = -wr + Rd.T @ wd
    wrd_dot = (adjoint(wr) @ Rd.T + Rd_d.T) @ wd + Rd.T @ wd_d + 2 * er_d + er
    return wrd_dot


def control(t, p, R, vr, wr):
    vrd_dot = positionneur(pd(t), pd_d(t), pd_dd(t), p, R, vr, wr)
    wrd_dot = orientateur(Rd(t), Rd_d(t), Rd_dd(t), R, wr)
    f = back_stepping(vrd_dot, wrd_dot, R, vr, wr)
    return f


p = array([[10], [0], [-20]])  # x,y,z (front,right,down)
R = eulermat(0.2, 0.3, 0.4)
vr = array([[0], [0], [0]])
wr = array([[0], [0], [0]])
α = zeros((N, 1))

for t in arange(0, 10, dt):
    f = control(t, p, R, vr, wr)
    p, R, vr, wr = clock_hexa(p, R, vr, wr, f)
    clean3D(ax, -20, 20, -20, 20, 0, 25)
    draw_hexarotor3D(ax, p, R, α, ['green', 'black', 'red', 'blue', 'orange', 'brown'])
    draw_platform(ax, pd(t), Rd(t))
    α = α + dt * 30 * f
    pause(0.001)
pause(10)
