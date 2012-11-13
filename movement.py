import cmath
from math import *

FORWARD = 1+1j
BACK = -1-1j
LEFT = -1+1j
RIGHT = 1-1j


def ControlForwardSpeed(control):
  result = 0
  for track in control.real, control.imag:
    if track > 0:
      result += 2*track
    else:
      result += 1.35*track
  return result


def ControlAngularSpeed(control):
  result = 0
  for track, factor in (control.real, 1), (control.imag, -1):
    if track > 0:
      result += 0.022*track*factor
    else:
      result += 0.011*track*factor
  return result


def PerimeterControls(granularity=10):
  for i in range(granularity):
    x = 2.0*i/granularity - 1.0
    yield complex(1, x)
    yield complex(-x, 1)
    yield complex(-1, -x)
    yield complex(x, -1)


def ArrivalTimeHelper(x, y, v_max, w_max):
  d2 = x**2+y**2
  if d2 < 1e-3:
    return 0
  y = abs(y)
  if y < 1e-3:
    if x >= 0:
      return x/v_max
    else:
      return 1e4

  a = complex(x, y)
  a /= abs(a)
  alpha = cmath.phase(a*a)
  if x < 0:
    alpha += 2*pi

  w_inv = (2*v_max*y+d2*w_max) / (2*y*v_max*w_max)

  return alpha*w_inv


def ArrivalTime(pos):
  t1 = ArrivalTimeHelper(pos.real, pos.imag, 4, 0.022)
  t2 = ArrivalTimeHelper(-pos.real, pos.imag, 2.7, 0.022)
  return min(t1, t2)


def PredictMovement(control, pos0, a0, v0=0, w0=0, efficiency=1, step=1):
  f = ControlForwardSpeed(control) * efficiency
  w = ControlAngularSpeed(control) * efficiency
  x = pos0
  v = f
  a = a0
  data = []

  actual_v = v0
  actual_w = w0

  Q = 0.05*step
  W = 0.03*step

  for i in range(1000):
    yield x, a

    actual_v = actual_v * (1-Q) + Q * cmath.rect(v, a)
    actual_w = actual_w * (1-W) + W * w

    x += actual_v*step
    a += actual_w*step

