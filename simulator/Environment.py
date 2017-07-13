# -*- coding:utf-8 -*-
# __author__ = 'shawnlue'

import math
import random
from settings import *
from utils import addVectors, collide
import copy


class Entity(object):

    """Base"""
    def __init__(self, (x, y), size, **kargs):
        self.x = x
        self.y = y
        self.size = size

        self.colour = kargs.get('colour')
        self.speed = kargs.get('speed', 0.0)
        self.angle = kargs.get('angle', 0.0)
        self.mass = kargs.get('mass', self.size * 10)
        self.thickness = 0
        self.drag = 1
        self.elasticity = kargs.get('elasticity', 0.9)

    def mouseMove(self, x, y):
        """ Change angle and speed to move towards a given point """

        dx = x - self.x
        dy = y - self.y
        self.angle = 0.5 * math.pi + math.atan2(dy, dx)
        self.speed = math.hypot(dx, dy) * 0.1


class Obstacle(Entity):
    """ A square object with only size and mass """
    def __init__(self, (x, y), size=OB_DEFAULT_SIZE, elasticity=0.9, **kargs):
        super(Obstacle, self).__init__((x, y), size,
                                       colour=OBSTACLE_COLOUR, elasticity=elasticity, **kargs)

    def move(self):
        self.x += math.sin(self.angle) * self.speed
        self.y -= math.cos(self.angle) * self.speed
        self.speed *= self.drag


class Robot(Entity):

    """ A circular object with a velocity, size and mass """
    def __init__(self, (x, y), size=ROBOT_DEFAULT_SIZE, elasticity=0.9, **kargs):
        super(Robot, self).__init__((x, y), size,
                                    colour=ROBOT_COLOUR, elasticity=elasticity, **kargs)

    def move(self, **kargs):
        """
        Update position based on speed, angle
        Update speed based on drag
        """
        self.angle = kargs.get('angle', self.angle)
        self.speed = kargs.get('speed', self.speed)
        # (self.angle, self.speed) = addVectors((self.angle, self.speed), gravity)
        self.x += math.sin(self.angle) * self.speed
        self.y -= math.cos(self.angle) * self.speed
        self.speed *= self.drag


class Environment:

    """ Defines the boundary of a simulation and its properties """

    def __init__(self, (width, height)):
        self.width = width
        self.height = height
        self.colour = ENV_COLOUR
        self.mass_of_air = 0.05  # 0.05, bigger value, more resistance
        self.elasticity = 0.75   # 0.75
        self.vel_bound = 1
        self.entities = {'swarm': [], 'obstacles': []}

    def reset(self):
        pass

    def get_map_1(self):    # mainly used for simulate RVO
        info = {'swarm': 14, 'swarm_size': ROBOT_DEFAULT_SIZE, 'obstacle': 4, 'obstacle_size': OB_DEFAULT_SIZE}
        for i in range(info['swarm']):
            if i % 2 == 0:
                x = 60 * (i / 2 + 1)
                y = self.height - info['swarm_size'] - 100
            else:
                x = 60 * (i / 2 + 1)
                y = 100
            angle = 20
            speed = 1
            robot = Robot((x, y), angle=angle, speed=speed)
            robot.drag = (robot.mass / (robot.mass + self.mass_of_air)) ** robot.size
            self.entities['swarm'].append(robot)

        for j in range(info['obstacle']):
            x = (j + 1) * self.width / (info['obstacle'] + 1) - info['obstacle_size'] / 2
            y = self.height / 2 - info['obstacle_size'] / 2
            obstacle = Obstacle((x, y), info['obstacle_size'])
            self.entities['obstacles'].append(obstacle)

    def add_Random_Obstacles(self, n=1, **kargs):
        for i in range(n):
            size = kargs.get('size', OB_DEFAULT_SIZE)
            ob = None
            while True:
                x = kargs.get('x', random.uniform(size + 5, self.width - size - 5))
                y = kargs.get('y', random.uniform(size + 5, self.height - size - 5))
                ob = Obstacle((x, y), size)
                ob.drag = (ob.mass/(ob.mass + self.mass_of_air)) ** ob.size
                has_collision = False
                entities_list = reduce(lambda p, q: p+q, [j for j in self.entities.itervalues()])
                for entity in entities_list:
                    if collide(entity, ob, only_return_if_collision=True):
                        has_collision = True
                        break
                if not has_collision:
                    break
            self.entities['obstacles'].append(ob)

    def add_Random_Robots(self, n=1, **kargs):
        """ Add n robots with properties given by keyword arguments """
        for i in range(n):
            size = kargs.get('size', ROBOT_DEFAULT_SIZE)
            robot = None
            while True:
                x = kargs.get('x', random.uniform(size + 5, self.width - size - 5))
                y = kargs.get('y', random.uniform(size + 5, self.height - size - 5))
                robot = Robot((x, y), size)
                robot.speed = kargs.get('speed', 10)
                robot.angle = kargs.get('angle', random.uniform(0, math.pi*2))
                robot.drag = (robot.mass/(robot.mass + self.mass_of_air)) ** robot.size
                has_collision = False
                entities_list = reduce(lambda p, q: p+q, [j for j in self.entities.itervalues()])
                for entity in entities_list:
                    if collide(entity, robot, only_return_if_collision=True):
                        has_collision = True
                        break
                if not has_collision:
                    break
            self.entities['swarm'].append(robot)

    def update(self):
        """  Moves particles and tests for collisions with the walls and each other """

        # for i, entity in enumerate(self.entities['swarm']):
        #     if entity.speed > self.vel_bound:
        #         entity.speed = self.vel_bound
        #     entity.move()
        #     self.bounce(entity)
        #     for entity2 in self.entities['swarm'][i+1:]:
        #         collide(entity, entity2)

        entities_list = reduce(lambda x, y: x+y, [i for i in self.entities.itervalues()])
        for i, entity in enumerate(entities_list):
            if entity.speed > self.vel_bound:
                entity.speed = self.vel_bound
            entity.move()
            self.bounce(entity)
            for entity2 in entities_list[i+1:]:
                collide(entity, entity2)

    def bounce(self, particle):
        """ Tests whether a particle has hit the boundary of the environment """

        if particle.x > self.width - particle.size:
            particle.x = 2*(self.width - particle.size) - particle.x
            particle.angle = - particle.angle
            particle.speed *= self.elasticity

        elif particle.x < particle.size:
            particle.x = 2*particle.size - particle.x
            particle.angle = - particle.angle
            particle.speed *= self.elasticity

        if particle.y > self.height - particle.size:
            particle.y = 2*(self.height - particle.size) - particle.y
            particle.angle = math.pi - particle.angle
            particle.speed *= self.elasticity

        elif particle.y < particle.size:
            particle.y = 2*particle.size - particle.y
            particle.angle = math.pi - particle.angle
            particle.speed *= self.elasticity

    def findParticle(self, x, y):
        """ Returns any particle that occupies position x, y """

        for particle in self.entities['swarm']:
            if math.hypot(particle.x - x, particle.y - y) <= particle.size:
                return particle
        return None
