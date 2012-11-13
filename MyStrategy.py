from __future__ import division
from math import *
from pprint import pprint
from copy import copy
import time
from collections import namedtuple, defaultdict

from model.Move import Move
from model.FireType import FireType
from model.TankType import TankType
from model.ShellType import ShellType
from model.Shell import Shell

from movement import *


#try:
#  import numpy
#  from matplotlib.pyplot import *
#except ImportError:
#  pass


PLANNING_INTERVAL = 75
STEP = 3


def PreprocessUnit(unit):
  unit.pos = complex(unit.x, unit.y)
  unit.v = complex(unit.speedX, unit.speedY)
  unit.r = 0.5 * abs(complex(unit.width, unit.height))


def IsStuck(x, r, world):
  return not (r < x.real < world.width - r and
              r < x.imag < world.height - r)


def PredictShells(me, world):
  shells_future = {}
  shells_future[0] = world.shells
  for t in range(STEP, PLANNING_INTERVAL, STEP):
    shells_future[t] = []
    for shell in shells_future[t-STEP]:
      new_shell = copy(shell)
      new_shell.pos += shell.v * STEP
      decel = {ShellType.REGULAR: 0.08/16.6,
               ShellType.PREMIUM: 0.13/13.3}
      new_shell.v *= exp(-STEP*decel[shell.type])
      shells_future[t].append(new_shell)
  return shells_future


def IsAlive(tank):
  return tank.crew_health > 0 and tank.hull_durability > 0


def Vulnerability(shooter_pos):
  d, phi = cmath.polar(shooter_pos)
  phi = abs(phi)
  phi /= pi / 2

  D = 350
  side_vul = 1 if d < D else D / d
  front_vul = 0.9
  back_vul = 2

  if phi < 1:
    vul = front_vul + (side_vul - front_vul) * phi
  else:
    vul = side_vul + (back_vul - side_vul) * (phi - 1)

  vul *= 1 - d / 3000
  return vul


def PositionDanger(me, tanks):
  result = 0
  for tank in tanks:
    if not IsAlive(tank):
      continue
    if me.player_name == tank.player_name:
      continue
    rel_pos = (tank.pos - me.pos) * cmath.rect(1, -me.angle)
    result += Vulnerability(rel_pos)

  return result


MIN_SHELL_R = -17
MAX_SHELL_R = 4.5
HIT_PENALTY = 20


def BonusTimeFunction(t):
  return 1 / (1 + t / 75)


def EvaluateControl(me, world, control):
  score = 0

  bonuses = [bonus.pos for bonus in world.bonuses]

  reliability = 1.0
  trace = PredictMovement(
      control,
      me.pos, me.angle,
      me.v, me.angular_speed,
      step=STEP)

  collected = set()
  dist_to_shell = {}
  for t, (x, a) in enumerate(trace):
    t *= STEP
    if t >= PLANNING_INTERVAL:
      break

    for shell in world.shells_future[t]:
      d = abs(x - shell.pos)
      if d < me.r + MAX_SHELL_R:
        if shell.id not in dist_to_shell:
          dist_to_shell[shell.id] = 1e3
        dist_to_shell[shell.id] = min(dist_to_shell[shell.id], d)

    if IsStuck(x, 0.5*me.r, world):
      reliability *= 0.95**STEP
      score -= 0.001 * STEP

  for tank in world.tanks:
    if tank.id == me.id:
      continue
    d = abs(tank.pos + tank.v * t - x)
    D = (me.r + tank.r) * 0.7
    if d > D:
      continue
    if d < 0.5 * D:
      q = 1
    else:
      q = (1 - d / D) * 2
    reliability *= 0.96**(STEP * q * ((1 - 1 / PLANNING_INTERVAL)))
    score -= 0.001 * STEP * (1 - 1 / PLANNING_INTERVAL)

    if reliability < 0.1:
      continue

    for bonus in bonuses:
      if bonus in collected:
        continue
      if abs(x - bonus) < 0.9*me.r:
        score += BonusTimeFunction(t) * reliability
        collected.add(bonus)

  for _, d in dist_to_shell.items():
    if d < me.r + MIN_SHELL_R:
      score -= HIT_PENALTY
    else:
      score -= HIT_PENALTY * (me.r + MAX_SHELL_R - d) / (MAX_SHELL_R - MIN_SHELL_R)

  new_me = copy(me)
  new_me.pos = x
  new_me.angle = a
  score -= 0.02 * PositionDanger(new_me, world.tanks)

  if reliability > 0.1:
    closest = 1e10
    for bonus in bonuses:
      if bonus in collected:
        continue
      rel_b = (bonus - x) * cmath.rect(1, -a)
      tt = ArrivalTime(rel_b)
      closest = min(closest, tt + t)

    score += BonusTimeFunction(closest) * reliability

  return score


def ChooseControl(me, world):
  world.shells_future = PredictShells(me, world)

  def f(c):
    return EvaluateControl(me, world, c)

  possible_controls = (
      [0] +
      #[0.75*c for c in PerimeterControls(1)] +
      list(PerimeterControls(12)))
  control = max(possible_controls, key=f)
  score = f(control)
  default_score = f(0)
  print '{:.3f}'.format(score)
  if score > -5 and default_score < -5:
    print 'evaded!!!!!!!!!!!!!!!!!'

  return control


Attack = namedtuple('Attack', 'rel_angle fire_type value')

def CollectAttacks(me, world):
  '''Return list of pairs (relative angle, fire type, value).'''
  turret_correction = cmath.rect(1, -me.angle - me.turret_relative_angle)
  for is_premium in False, True:
    if is_premium and me.premium_shell_count == 0:
      continue
    for tank in world.tanks:
      if tank.teammate:
        continue

      shell_v = 13.3 if is_premium else 16.6
      shell_decel = 13.3 / 0.13 if is_premium else 0.08 / 16.6

      dist = abs(me.pos - tank.pos)
      t = dist / shell_v

      target = tank.pos + tank.v * t

      rel_angle = cmath.phase((target - me.pos) * turret_correction)
      value = 30 if is_premium else 20
      fire_type = FireType.PREMIUM_PREFERRED if is_premium else FireType.REGULAR

      if not IsAlive(tank):
        value = 0

      yield Attack(rel_angle=rel_angle, fire_type=fire_type, value=value)


current_control = defaultdict(complex)

start = time.clock()
class MyStrategy:
  def move(self, me, world, move):
    global current_control

    print world.tick / (time.clock() - start + 1e-6), 'frames per second'

    PreprocessUnit(me)
    for unit in world.tanks + world.bonuses + world.shells:
      PreprocessUnit(unit)

    if world.tick > 30e10:
      def f(pos):
        new_me = copy(me)
        new_me.pos = pos
        return PositionDanger(new_me, world.tanks)
      z = numpy.array([[f(complex(x, y)) for x in range(0, 1280, 4)] for y in range(0, 800, 4)])

      fig = figure()
      ax = fig.add_subplot(111)
      ax.imshow(z)
      show()
      exit()


    attacks = CollectAttacks(me, world)
    def CurrentAttackValue(attack):
      result = attack.value
      t = abs(attack.rel_angle) / me.turret_turn_speed - me.remaining_reloading_time
      if t > 0:
        result *= max(1 - t / 150, 0.1)
      return result

    attack = max(attacks, key=CurrentAttackValue)

    freq = 1 if world.shells else 8
    if world.tick % freq == 0:
      current_control[me.id] = ChooseControl(me, world)

    move.left_track_power = current_control[me.id].real
    move.right_track_power = current_control[me.id].imag

    move.turret_turn = 1 if attack.rel_angle > 0 else -1
    if abs(attack.rel_angle) < 0.02 and world.tick >= 7:
      move.fire_type = attack.fire_type
    else:
      move.fire_type = FireType.NONE
    prev_move = copy(move)


  def select_tank(self, tank_index, team_size):
    return TankType.MEDIUM
