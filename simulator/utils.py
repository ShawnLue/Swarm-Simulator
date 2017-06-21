import math


def addVectors((angle1, length1), (angle2, length2)):
    """ Returns the sum of two vectors """
    x = math.sin(angle1) * length1 + math.sin(angle2) * length2
    y = math.cos(angle1) * length1 + math.cos(angle2) * length2
    angle = 0.5 * math.pi - math.atan2(y, x)
    length = math.hypot(x, y)
    return angle, length


def collide(p1, p2, only_return_if_collision=False):
    """ Tests whether two particles overlap
        If they do, make them bounce
        i.e. update their angle, speed and position """

    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dist = math.hypot(dx, dy)
    if dist < p1.size + p2.size:    # collide
        if only_return_if_collision:
            return True
        angle = math.atan2(dy, dx) + 0.5 * math.pi
        total_mass = p1.mass + p2.mass
        (p1.angle, p1.speed) = \
            addVectors((p1.angle, p1.speed * (p1.mass-p2.mass) / total_mass),
                       (angle, 2 * p2.speed * p2.mass / total_mass))
        (p2.angle, p2.speed) = \
            addVectors((p2.angle, p2.speed * (p2.mass-p1.mass) / total_mass),
                       (angle + math.pi, 2 * p1.speed * p1.mass / total_mass))

        elasticity = p1.elasticity * p2.elasticity
        p1.speed *= elasticity
        p2.speed *= elasticity

        overlap = 0.5 * (p1.size + p2.size - dist + 1)
        p1.x += math.sin(angle) * overlap
        p1.y -= math.cos(angle) * overlap
        p2.x -= math.sin(angle) * overlap
        p2.y += math.cos(angle) * overlap
    return False
