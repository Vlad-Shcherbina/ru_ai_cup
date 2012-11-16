from __future__ import division
from math import *
from pprint import pprint
from copy import copy
import time
from collections import namedtuple, defaultdict
import random

from model.Move import Move
from model.Tank import Tank
from model.FireType import FireType
from model.TankType import TankType
from model.ShellType import ShellType
from model.BonusType import BonusType

from movement import *


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

  reliability = 1.0
  trace = PredictMovement(
      control,
      me.pos, me.angle,
      me.v, me.angular_speed,
      efficiency=me.efficiency,
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

    if IsStuck(x, 0.4*me.r, world):
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

    for bonus in world.bonuses:
      if bonus in collected:
        continue
      if abs(x - bonus.pos) < 0.9*me.r:
        score += BonusTimeFunction(t) * bonus.value * reliability
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
    best_bonus_value = 0
    for bonus in world.bonuses:
      if bonus in collected:
        continue
      rel_b = (bonus.pos - x) * cmath.rect(1, -a)
      tt = ArrivalTime(rel_b) / me.efficiency
      best_bonus_value = max(best_bonus_value,
                             BonusTimeFunction(t + tt) * bonus.value)

    score += best_bonus_value * reliability

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
  #print '{:.3f}'.format(score)
  #if score > -5 and default_score < -5:
  #  print 'evaded!!!!!!!!!!!!!!!!!'

  return control


def ShellTravelTime(dist, is_premium):
  """Return time for a shell to pass given distance (or -1)"""
  shell_v = 13.3 if is_premium else 16.6
  shell_decel = 0.13 / 13.3 if is_premium else 0.08 / 16.6

  dist = max(dist, 0)

  if shell_v <= shell_decel * dist:
    return -1

  return log(shell_v / (shell_v - shell_decel * dist)) / shell_decel


def TraceShot(angle, is_premium, me, world):
  obstacles = [tank for tank in world.tanks if tank.id != me.id]
  obstacles += world.bonuses
  events = {}
  for o in obstacles:
    dist = abs(me.pos - o.pos) - me.virtual_gun_length - 0.3 * o.r
    t = ShellTravelTime(dist, is_premium)
    if t >= 0:
      events[o] = t

  score = 0
  reliability = 1
  for e in sorted(events, key=events.get):
    t = events[e]
    rel_pos = (e.pos + e.v * t - me.pos) * cmath.rect(1, -angle)
    if rel_pos.real < 0:
      continue
    d = abs(rel_pos.imag)
    d1 = 0.5 * (e.r + 3.75)
    d2 = e.r + 3.75
    if d < d1:
      w = 1
    elif d < d2:
      w = (d2 - d) / (d2 - d1)
    else:
      w = 0

    chance_to_evade = 0
    if isinstance(e, Tank) and IsAlive(e) and not e.teammate:
      evasion_speed = 0.15 + 0.85 * abs(sin(angle - e.angle))
      q = e.r * 1.0 / evasion_speed / (t + 1e-3) / e.efficiency
      if q < 2:
        chance_to_evade = 1
      elif q < 4:
        chance_to_evade = (q - 2) / 2
      else:
        chance_to_evade = 0
    chance_to_evade = min(chance_to_evade, 0.9)

    w *= 1 - chance_to_evade

    def HitValue(e):
      if isinstance(e, Tank) and IsAlive(e):
        backness = 0.5 + 0.5 * cos(e.angle - angle)
        backness *= exp(-t / 40 * e.efficiency)
        if is_premium:
          dmg = 45 + 25 * backness
        else:
          dmg = 23 + 17 * backness
        return -dmg if e.teammate else dmg
      return 0

    score += reliability * w * HitValue(e)
    reliability *= 1 - w
    if reliability < 0.01:
      break

  return score


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

      dist = abs(me.pos - tank.pos) - me.virtual_gun_length - 0.3 * tank.r
      t = ShellTravelTime(dist, is_premium)
      if t < 0:
        continue

      target = tank.pos + tank.v * t

      clip = 40
      target = complex(max(clip, min(target.real, world.width - clip)),
                       max(clip, min(target.imag, world.height - clip)))

      rel_angle = cmath.phase((target - me.pos) * turret_correction)

      abs_angle = rel_angle + me.angle + me.turret_relative_angle
      value = TraceShot(abs_angle, is_premium, me, world)

      fire_type = FireType.PREMIUM_PREFERRED if is_premium else FireType.REGULAR

      yield Attack(rel_angle=rel_angle, fire_type=fire_type, value=value)


current_control = defaultdict(complex)


def DrawPlot(me, world):
  #for TraceShot(angle, is_premium, me, world):
  xs = [0.5 * i for i in range(720)]
  ys = [TraceShot(x * pi / 180 * 0.5, False, me, world) for x in range(720)]
  from matplotlib.pyplot import *
  plot(xs, ys, 'o')
  show()
  exit()


start = time.clock()
class MyStrategy:
  def move(self, me, world, move):
    global current_control

    if world.tick % 100 == 0:
      print world.tick / (time.clock() - start + 1e-6), 'frames per second'

    PreprocessUnit(me)
    for unit in world.tanks + world.bonuses + world.shells:
      PreprocessUnit(unit)

    for tank in [me] + world.tanks:
      tank.efficiency = 0.5 + 0.5 * tank.crew_health / tank.crew_max_health

    #if world.tick == 500:
    #  DrawPlot(me, world)

    for bonus in world.bonuses:
      if bonus.type == BonusType.MEDIKIT:
        # relative increase in efficiency
        d = min(35, me.crew_max_health - me.crew_health)
        d /= me.crew_health + me.crew_max_health
        bonus.value = 0.05 + 7 * d
        if me.crew_health <= 20:
          bonus.value *= 1.5
      elif bonus.type == BonusType.REPAIR_KIT:
        bonus.value = 0.05 + 1.5 * (1 - me.hull_durability / me.hull_max_durability)
        if me.hull_durability <= 20:
          bonus.value *= 1.5
      elif bonus.type == BonusType.AMMO_CRATE:
        bonus.value = 1 / (1 + me.premium_shell_count / 2)
      else:
        print 'UNKNOWN BONUS TYPE!!!'
        bonus.value = 1

    attacks = CollectAttacks(me, world)
    def CurrentAttackValue(attack):
      result = attack.value
      t = (abs(attack.rel_angle) / (me.turret_turn_speed * me.efficiency) -
           me.remaining_reloading_time)
      if t > 0:
        result *= max(1 - t / me.reloading_time, 0.1)
      return result

    attack = max(attacks, key=CurrentAttackValue)

    freq = 1 if world.shells else 8
    if world.tick % freq == 0:
      current_control[me.id] = ChooseControl(me, world)

    move.left_track_power = current_control[me.id].real
    move.right_track_power = current_control[me.id].imag

    my_index = sorted(t.id for t in world.tanks if t.teammate).index(me.id)
    if (abs(attack.rel_angle) < 0.005 and
        world.tick >= 6 + 2 * my_index and
        attack.value > random.randrange(1, 40)):
      move.fire_type = attack.fire_type
    else:
      move.fire_type = FireType.NONE

    needed_turn = attack.rel_angle - me.angular_speed
    needed_turn /= me.turret_turn_speed * me.efficiency
    needed_turn = max(-1, min(needed_turn, 1))
    move.turret_turn = needed_turn


  def select_tank(self, tank_index, team_size):
    return TankType.MEDIUM
