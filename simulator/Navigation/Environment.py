# -*- coding:utf-8 -*-
# __author__ = 'shawnlue'

import math
import pygame as pg
import random
import os
from ..default_settings import *
from utils import addVectors, collide, distance, bounce


class Entity(object):

    """Base"""
    def __init__(self, (x, y), index, size, **kargs):
        self.x = x
        self.y = y
        self.id = index
        self.size = size

        self.colour = kargs.get('colour')
        self.speed = kargs.get('speed', 0.0)
        self.angle = kargs.get('angle', 0.0)
        self.mass = kargs.get('mass', self.size * 10)
        self.vel_bound = kargs.get('vel_bound', 0.0)
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
    def __init__(self, (x, y), index, size=OB_DEFAULT_SIZE, elasticity=0.9, **kargs):
        super(Obstacle, self).__init__((x, y), index, size,
                                       colour=OBSTACLE_COLOUR, elasticity=elasticity, **kargs)
        self._info_flow = {'id': self.id, 'pos': [self.x, self.y], 'size': self.size, 'angle': self.angle, 'speed': self.speed}

    def update_info_flow(self):
        self._info_flow.update({'pos': [self.x, self.y], 'size': self.size, 'angle': self.angle, 'speed': self.speed})

    @property
    def info_flow(self):
        self.update_info_flow()
        return self._info_flow

    def move(self):
        self.x += math.sin(self.angle) * self.speed
        self.y -= math.cos(self.angle) * self.speed
        self.speed *= self.drag
        self.update_info_flow()


class Robot(Entity):

    """ A circular object with a velocity, size and mass """
    def __init__(self, (x, y), index, sensing_range=LOCAL_SENSING_RANGE,
                 size=ROBOT_DEFAULT_SIZE, elasticity=0.9, **kargs):
        super(Robot, self).__init__((x, y), index, size,
                                    colour=ROBOT_COLOUR, elasticity=elasticity, **kargs)
        self._sensing_range = sensing_range
        self.goal = [0.0, 0.0]
        self._info_flow = {'pos': [self.x, self.y], 'size': self.size, 'angle': self.angle,
                           'speed': self.speed, 'goal': self.goal, 'vel_bound': self.vel_bound}

    def update_info_flow(self):
        self._info_flow.update({'id': self.id, 'pos': [self.x, self.y], 'size': self.size,'angle': self.angle, 'speed': self.speed, 'goal': self.goal, 'vel_bound': self.vel_bound})

    @property
    def sensing_range(self):
        return self._sensing_range

    @property
    def info_flow(self):
        self.update_info_flow()
        return self._info_flow

    def move(self, **kargs):
        # TODO: For now, the control model is implemented as: (speed, angle). This may be problematic due to the cycle of angle radians when using learning based method on this dimension. Future edition should have a try on (x-axis, y-axis).
        """
        Update position based on speed, angle
        Update speed based on drag
        """
        self.speed = max(0, min(kargs.get('speed', self.angle), self.vel_bound))
        self.angle = max(-math.pi, min(kargs.get('angle', self.speed), math.pi))
        # (self.angle, self.speed) = addVectors((self.angle, self.speed), gravity)
        self.x += math.sin(self.angle) * self.speed
        self.y -= math.cos(self.angle) * self.speed
        # print self.angle
        # print math.sin(self.angle) * self.speed, math.cos(self.angle) * self.speed
        self.speed *= self.drag
        self.update_info_flow()


class Environment:

    """ Defines the boundary of a simulation and its properties """

    def __init__(self, width, height, name='CIL-Navigation-Simulator', centered=False, gui=True, comm_render=False, **kargs):
        # Vis part
        if centered:
            os.environ["SDL_VIDEO_CENTERED"] = '1'
        self.gui = gui
        self.comm_render = comm_render
        if not self.gui:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
        pg.init()
        pg.display.set_caption(name)
        self._screen = pg.display.set_mode((width, height), 0, 8)
        self._running = True
        self._use_mouse = kargs.get('use_mouse', False)

        # Data part
        self.width = width
        self.height = height
        self.colour = ENV_COLOUR
        self.mass_of_air = 0.05  # 0.05, bigger value, more resistance
        self.elasticity = 0.75   # 0.75
        self.vel_bound = MAX_SPEED
        self.entities = {'swarm': [], 'obstacles': []}
        self._initialized = False   # if this env has been initialized
        self._robot_count, self._ob_count = 0, 0
        self._neighborhood_set = {}

    @property
    def running(self):
        return self._running

    @property
    def robot_count(self):
        return self._robot_count

    @property
    def ob_count(self):
        return self._ob_count

    @property
    def neighborhood_set(self):
        return self._neighborhood_set

    def assign_goals(self):
        # assign the goal pos to each robot
        if not self._initialized:
            raise NotImplementedError("The Env has not been initialized.")
        count = self.robot_count
        for i in range(count / 2):
            self.entities['swarm'][i].goal = [self.entities['swarm'][count - i - 1].x,\
                                             self.entities['swarm'][count - i - 1].y]
            self.entities['swarm'][count - i - 1].goal = [self.entities['swarm'][i].x, \
                                             self.entities['swarm'][i].y]
            self.entities['swarm'][i].update_info_flow()
            self.entities['swarm'][count - i - 1].update_info_flow()


    def reset(self, ob_num, robot_num):

        self._robot_count, self._ob_count = 0, 0
        self.entities = {'swarm': [], 'obstacles': []}

        self.add_Random_Obstacles(ob_num)
        self.add_Random_Robots(robot_num)
        self.assign_goals()

        self.update_neighborhood_set()
        self.render()
        return self.get_state()

    def step(self, command=None):
        selected_particle = None
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self._running = False
            elif event.type == pg.MOUSEBUTTONDOWN:
                (mouseX, mouseY) = pg.mouse.get_pos()
                selected_particle = self.findParticle(mouseX, mouseY)
            elif event.type == pg.MOUSEBUTTONUP:
                selected_particle = None

        if self._use_mouse and selected_particle:
            (mouseX, mouseY) = pg.mouse.get_pos()
            selected_particle.mouseMove(mouseX, mouseY)

        """  Moves particles and tests for collisions with the walls and each other """

        # for i, entity in enumerate(self.entities['swarm']):
        #     if entity.speed > self.vel_bound:
        #         entity.speed = self.vel_bound
        #     entity.move()
        #     self.bounce(entity)
        #     for entity2 in self.entities['swarm'][i+1:]:
        #         collide(entity, entity2)
        entities_list = reduce(lambda x, y: x + y, [i for i in self.entities.itervalues()])
        # Todo: ugly code
        if command is not None:
            for i, entity in enumerate(entities_list):
                if entity.__class__.__name__ == 'Robot':
                    entity.move(speed=command[entity.id][0], angle=command[entity.id][1])
                else:
                    entity.move()
                bounce(entity, self.width, self.height, self.elasticity)
                for entity2 in entities_list[i + 1:]:
                    collide(entity, entity2)
        else:
            for i, entity in enumerate(entities_list):
                if entity.speed > self.vel_bound:
                    entity.speed = self.vel_bound
                entity.move()
                bounce(entity, self.width, self.height, self.elasticity)
                for entity2 in entities_list[i + 1:]:
                    collide(entity, entity2)
        ##
        self.update_neighborhood_set()
        self.render()
        return self.get_state()

    def update_neighborhood_set(self):
        if not self._initialized:
            raise NotImplementedError("The Environment has not been initialized.")

        for i in range(len(self.entities['swarm'])):
            self._neighborhood_set[str(i)] = {'swarm': [], 'obstacles': []}

        assert len(self.entities['swarm']) == self.robot_count
        assert len(self.entities['obstacles']) == self.ob_count

        for i in range(self.robot_count):
            robot_i = self.entities['swarm'][i]
            for j in range(i + 1, self.robot_count):
                robot_j = self.entities['swarm'][j]
                if distance((robot_i.x, robot_i.y), (robot_j.x, robot_j.y)) < robot_i.sensing_range:
                    self._neighborhood_set[str(i)]['swarm'].append(j)
                    self._neighborhood_set[str(j)]['swarm'].append(i)
            for k in range(self.ob_count):
                ob_k = self.entities['obstacles'][k]
                if distance((robot_i.x, robot_i.y), (ob_k.x, ob_k.y)) < robot_i.sensing_range:
                    self._neighborhood_set[str(i)]['obstacles'].append(k)

    def findParticle(self, x, y):
        """ Returns any particle that occupies position x, y """

        for particle in self.entities['swarm']:
            if math.hypot(particle.x - x, particle.y - y) <= particle.size:
                return particle
        return

    # Map Settings
    def get_map_1(self):    # mainly used for simulate RVO
        info = {'swarm': 14, 'swarm_size': ROBOT_DEFAULT_SIZE, 'obstacle': 4, 'obstacle_size': OB_DEFAULT_SIZE}
        for i in range(info['swarm']):
            x = 60 * (i / 2 + 1)
            if i % 2 == 0:
                y = self.height - info['swarm_size'] - 100
            else:
                y = 100
            angle = 60
            speed = 0.0
            robot = Robot((x, y), index=self._robot_count, angle=angle, speed=speed, vel_bound=self.vel_bound)
            self._robot_count += 1
            robot.drag = (robot.mass / (robot.mass + self.mass_of_air)) ** robot.size
            self.entities['swarm'].append(robot)

        for j in range(info['obstacle']):
            x = (j + 1) * self.width / (info['obstacle'] + 1)
            y = self.height / 2
            obstacle = Obstacle((x, y), self._ob_count, info['obstacle_size'], vel_bound=self. vel_bound)
            self._ob_count += 1
            self.entities['obstacles'].append(obstacle)
        self._initialized = True

        self.assign_goals()

        self.update_neighborhood_set()
        self.render()
        return self.get_state()

    def add_Random_Obstacles(self, n=1, **kargs):
        for i in range(n):
            size = kargs.get('size', OB_DEFAULT_SIZE)
            ob = None
            while True:
                x = kargs.get('x', random.uniform(size + 5, self.width - size - 5))
                y = kargs.get('y', random.uniform(size + 5, self.height - size - 5))
                ob = Obstacle((x, y), self.ob_count, size, vel_bound=self.vel_bound)
                ob.drag = (ob.mass/(ob.mass + self.mass_of_air)) ** ob.size
                has_collision = False
                entities_list = reduce(lambda p, q: p+q, [j for j in self.entities.itervalues()])
                for entity in entities_list:
                    if collide(entity, ob, only_return_if_collision=True):
                        has_collision = True
                        break
                if not has_collision:
                    self._ob_count += 1
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
                robot = Robot((x, y), self._robot_count, size=size, vel_bound=self.vel_bound)
                robot.speed = kargs.get('speed', 1.0)
                robot.angle = kargs.get('angle', random.uniform(0, math.pi*2))
                robot.drag = (robot.mass/(robot.mass + self.mass_of_air)) ** robot.size
                has_collision = False
                entities_list = reduce(lambda p, q: p+q, [j for j in self.entities.itervalues()])
                for entity in entities_list:
                    if collide(entity, robot, only_return_if_collision=True):
                        has_collision = True
                        break
                if not has_collision:
                    self._robot_count += 1
                    break
            self.entities['swarm'].append(robot)
        self._initialized = True    # only at least one robot has been initialized, the game is initialized

    # Render
    def render(self):
        self._screen.fill(self.colour)
        # Draw swarm
        for r in self.entities['swarm']:
            pg.draw.circle(self._screen, r.colour, (int(round(r.x)), int(round(r.y))), r.size, r.thickness)
            # render text
            myfont = pg.font.SysFont("monospace", 17, bold=True)
            label = myfont.render(str(r.id), False, (255, 0, 0))
            self._screen.blit(label, (r.x - float(r.size) / 2, r.y - r.size))
        # Draw obstalces
        for ob in self.entities['obstacles']:
            pg.draw.rect(self._screen, ob.colour, (
            ob.x - float(ob.size) / 1.414, ob.y - float(ob.size) / 1.414, ob.size * 1.414, ob.size * 1.414),
                         ob.thickness)
            # render text
            myfont = pg.font.SysFont("monospace", 17, bold=True)
            label = myfont.render(str(ob.id), False, (0, 255, 0))
            self._screen.blit(label, (ob.x - float(ob.size) / 4, ob.y - float(ob.size) / 4))

        # Draw communication, mainly for debug
        if self.comm_render:
            for ri_str, neighbor_dict in self._neighborhood_set.items():
                ri = self.entities['swarm'][int(ri_str)]
                for swarm_id in neighbor_dict['swarm']:
                    rj = self.entities['swarm'][swarm_id]
                    pg.draw.line(self._screen, (0, 255, 255), (ri.x, ri.y), (rj.x, rj.y))
                for ob_id in neighbor_dict['obstacles']:
                    obj = self.entities['obstacles'][ob_id]
                    pg.draw.line(self._screen, (255, 255, 0), (ri.x, ri.y), (obj.x, obj.y))

        # buff = pg.image.tostring(screen, "P")
        # buff = np.fromstring(buff, dtype=np.uint8)
        # buff = buff.reshape(width, height)
        # plt.imshow(buff)
        # plt.show()
        pg.display.flip()

    def get_state(self):
        # Todo: fix the state representation with the dict {'id':..., 'neighborhood':{'swarm': ...}}
        # Return robots state and obs data
        z = [(ent.info_flow, self.neighborhood_set[str(ent.id)]) for ent in self.entities['swarm']]
        for item in z:
            item[0].update({'neighborhood': item[1]})

        z = [item[0] for item in z]
        return z, [ent.info_flow for ent in self.entities['obstacles']]
