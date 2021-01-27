from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
from scipy.integrate import solve_ivp

def f(x, u):
  alpha = 1
  return array([[x[2, 0]],
                [x[3, 0]],
                [-(alpha * x[0, 0]/norm(x[0:2])**3) + x[2, 0] * u],
                [-(alpha * x[1, 0]/norm(x[0:2])**3) + x[3, 0] * u]])


def control(x, radius):
  x1, x2, x3, x4 = x.flatten()
  e_p = x1**2 + x2**2 - radius**2
  kp = -1
  e_d = x1*x3 + x2*x4
  kd = -10
  e_dd = x3**2 + x4**2 - 1/radius
  kdd = -1
  u = kp * e_p + kd * e_d + kdd * e_dd
  return u


s, dt = 2, 0.1
ax = init_figure(-s, s, -s, s)
radius = 1.2
Cx, Cy = radius * np.cos(arange(0, 2*pi, 0.1)), radius * np.sin(arange(0, 2*pi, 0.1))
x = array([[1.0], [0], [0], [1]])
for t in arange(0, 100, dt):
  clear(ax)
  u= control(x, radius)
  x += dt * (f(x, u)/4 + (3/4)*f(x + (2/3)*dt*f(x, u), u))
  ax.plot(Cx, Cy)
  draw_disk(ax, array([[0], [0]]), 0.2, "blue", 1, 1)
  draw_disk(ax, array([[x[0]], [x[1]]]), 0.1, "red", 1)
  pause(0.001)
pause(0.1)
