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
        self._width, self._height = width, height
        self._screen = pg.display.set_mode((width, height), 0, 8)
        self._running = True
        self._env = Environment.Environment((self._width, self._height))
        self._use_mouse = kargs.get('use_mouse', False)

    @property
    def running(self):
        return self._running

    def env_settings(self):
        # TODO: outside settings API
        # -------Env settings-------

        # add specified entities to env
        # self.env.addParticles(x=200, y=250, size=10, speed=0, angle=0)

        # add random entities to env
        # self.env.add_Random_Obstacles(8)
        # self.env.add_Random_Robots(20)

        # default map
        self._env.get_map_1()

    def main_loop(self):
        selected_particle = None
        while self.running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self._running = False
                elif event.type == pg.MOUSEBUTTONDOWN:
                    (mouseX, mouseY) = pg.mouse.get_pos()
                    selected_particle = self._env.findParticle(mouseX, mouseY)
                elif event.type == pg.MOUSEBUTTONUP:
                    selected_particle = None

            if self._use_mouse and selected_particle:
                (mouseX, mouseY) = pg.mouse.get_pos()
                selected_particle.mouseMove(mouseX, mouseY)

            self._env.update()
            self._screen.fill(self._env.colour)
            self.draw()
            pg.display.flip()

    def draw(self):
        # Draw swarm
        for r in self._env.entities['swarm']:
            pg.draw.circle(self._screen, r.colour, (int(r.x), int(r.y)), r.size, r.thickness)
            # render text
            myfont = pg.font.SysFont("monospace", 17, bold=True)
            label = myfont.render(str(r.id), False, (255, 0, 0))
            self._screen.blit(label, (r.x - r.size / 2, r.y - r.size))
        # Draw obstalces
        for ob in self._env.entities['obstacles']:
            pg.draw.rect(self._screen, ob.colour, (int(ob.x), int(ob.y), ob.size / 1.414, ob.size / 1.414), ob.thickness)
            # render text
            myfont = pg.font.SysFont("monospace", 17, bold=True)
            label = myfont.render(str(ob.id), False, (0, 255, 0))
            self._screen.blit(label, (ob.x + ob.size/4, ob.y + ob.size/4))
        # Draw communication
        #'''
        for ri_str, neighbor_dict in self._env._neighborhood_set.items():
            ri = self._env.entities['swarm'][int(ri_str)]
            for swarm_id in neighbor_dict['swarm']:
                rj = self._env.entities['swarm'][swarm_id]
                pg.draw.line(self._screen, (0, 255, 255), (round(ri.x), round(ri.y)), (round(rj.x), round(rj.y)))
            for ob_id in neighbor_dict['obstacles']:
                obj = self._env.entities['obstacles'][ob_id]
                pg.draw.line(self._screen, (255, 255, 0), (round(ri.x), round(ri.y)), (round(obj.x), round(obj.y)))
        #'''

        # buff = pg.image.tostring(screen, "P")
        # buff = np.fromstring(buff, dtype=np.uint8)
        # buff = buff.reshape(width, height)
        # plt.imshow(buff)
        # plt.show()