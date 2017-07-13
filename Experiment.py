# -*- coding:utf-8 -*-
# __author__ = 'shawnlue'

import pygame as pg
import os
from simulator import Environment


class Experiment:

    def __init__(self, width, height, name='Swarm-MARL-Simulator', centered=False, gui=True, **kargs):
        os.environ["SDL_VIDEO_CENTERED"] = centered
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

    def loop(self):
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

