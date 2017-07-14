# -*- coding:utf-8 -*-
# __author__ = 'shawnlue'

import pygame as pg
import os
from simulator import Environment


class Experiment:

    def __init__(self, width, height, name='CIL-Swarm-Simulator', centered=False, gui=True, **kargs):
        if centered:
            os.environ["SDL_VIDEO_CENTERED"] = '1'
        self.gui = gui
        if not self.gui:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
        pg.init()
        pg.display.set_caption(name)
        self.width, self.height = width, height
        self.screen = pg.display.set_mode((width, height), 0, 8)
        self._running = True
        self.env = Environment.Environment((self.width, self.height))
        self.use_mouse = kargs.get('use_mouse', False)

    @property
    def running(self):
        return self._running

    def env_settings(self):
        # -------Env settings-------

        # add specified entities to env
        # env.addParticles(x=200, y=250, size=10, speed=0, angle=0)

        # add random entities to env
        self.env.add_Random_Obstacles(8)
        self.env.add_Random_Robots(20)

        # default map
        # env.get_map_1()

    def main_loop(self):
        selected_particle = None
        while self.running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self._running = False
                elif event.type == pg.MOUSEBUTTONDOWN:
                    (mouseX, mouseY) = pg.mouse.get_pos()
                    selected_particle = self.env.findParticle(mouseX, mouseY)
                elif event.type == pg.MOUSEBUTTONUP:
                    selected_particle = None

            if self.use_mouse and selected_particle:
                (mouseX, mouseY) = pg.mouse.get_pos()
                selected_particle.mouseMove(mouseX, mouseY)

            self.env.update()
            self.screen.fill(self.env.colour)
            self.draw()
            pg.display.flip()

    def draw(self):
        for r in self.env.entities['swarm']:
            pg.draw.circle(self.screen, r.colour, (int(r.x), int(r.y)), r.size, r.thickness)
        for ob in self.env.entities['obstacles']:
            pg.draw.rect(self.screen, ob.colour, (int(ob.x), int(ob.y), ob.size / 1.414, ob.size / 1.414), ob.thickness)
        # buff = pg.image.tostring(screen, "P")
        # buff = np.fromstring(buff, dtype=np.uint8)
        # buff = buff.reshape(width, height)
        # plt.imshow(buff)
        # plt.show()