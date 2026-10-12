import math


def target15(x0, y0):
    return x0 - 4.6, y0 + 0.4


def velocity(x, y, target_x, target_y, kp=0.6, max_speed=0.65):
    dx, dy = target_x - x, target_y - y
    distance = math.hypot(dx, dy)
    scale = min(kp, max_speed / distance) if distance else 0.0
    return dx * scale, dy * scale, distance
