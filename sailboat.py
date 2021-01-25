from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


p0, p1, p2, p3, p4, p5, p6, p7, p8, p9 = 0.1, 1, 6000, 1000, 2000, 1, 1, 2, 300, 10000
q = 1

def f(x, u):
    x, u = x.flatten(), u.flatten()
    θ = x[2]
    v = x[3]
    w = x[4]
    δr = u[0]
    δsmax = u[1]
    w_ap = array([[awind*cos(ψ-θ) - v],
                  [awind*sin(ψ-θ)]])
    ψ_ap = angle(w_ap)
    a_ap = norm(w_ap)
    sigma = cos(ψ_ap) + cos(δsmax)
    if sigma < 0:
        δs = pi + ψ_ap
    else:
        δs = -sign(sin(ψ_ap))*δsmax
    fr = p4*v*sin(δr)
    fs = p3*a_ap * sin(δs - ψ_ap)
    dx = v*cos(θ) + p0*awind*cos(ψ)
    dy = v*sin(θ) + p0*awind*sin(ψ)
    dv = (fs*sin(δs)-fr*sin(δr)-p1*v**2)/p8
    dw = (fs*(p5-p6*cos(δs)) - p7*fr*cos(δr) - p2*w*v)/p9
    xdot = array([[dx], [dy], [w], [dv], [dw]])
    return xdot, δs


def régulateur(m, θ, ψ, a, b, q):
    e = det(hstack(((b - a)/norm(b - a), m - a)))
    if abs(e) > r:
        q = sign(e)
    phi = arctan2(b[1] - a[1], b[0] - a[0])
    theta_bar = phi - arctan(e/r)
    if cos(ψ - theta_bar) + cos(zeta) < 0 or (abs(e) < r and cos(ψ - phi) + cos(zeta) < 0):
        theta_bar = pi + ψ - q * zeta
    δr = (2*δr_max)/pi * arctan(tan((θ - theta_bar)/2))
    δsmax = pi/2 * ((cos(ψ - theta_bar) + 1)/2)**(log(pi/(2*beta))/log(2))
    return array([[δr], [δsmax]]), q


x = array([[10, -40, -3, 0, 0]]).T  # x=(x,y,θ,v,w)

dt = 0.1
awind, ψ = 2, -2
zeta = pi/4
δr_max = 1
r = 10
beta = 1

a = array([[-50], [-100]])
b = array([[50], [100]])

ax = init_figure(-100, 100, -60, 60)

tmax = 100

for t in arange(0, tmax, dt):
    clear(ax)
    plot([a[0, 0], b[0, 0]], [a[1, 0], b[1, 0]], 'red')
    m = x[0:2]
    θ = x[2, 0]
    u, q = régulateur(m, θ, ψ, a, b, q)
    xdot, δs = f(x, u)
    x = x + dt*xdot
    draw_sailboat(x, δs, u[0, 0], ψ, awind)
