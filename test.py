import pygame as pg
import PyParticles
import numpy as np
import os
import matplotlib.pyplot as plt

from settings import *

# os.environ["SDL_VIDEO_CENTERED"] = "True"
# os.environ["SDL_VIDEODRIVER"] = "dummy"
pg.init()
pg.display.set_caption('Swarm RVO')
(width, height) = (WIDTH, HEIGHT)
screen = pg.display.set_mode((width, height), 0, 8)
env = PyParticles.Environment((width, height))

# env.addParticles(x=200, y=250, size=10, speed=0, angle=0)

env.add_Random_Obstacles(8)
env.add_Random_Robots(20)
# env.get_map_1()

selected_particle = None

running = True
while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
        elif event.type == pg.MOUSEBUTTONDOWN:
            (mouseX, mouseY) = pg.mouse.get_pos()
            selected_particle = env.findParticle(mouseX, mouseY)
        elif event.type == pg.MOUSEBUTTONUP:
            selected_particle = None

    if USE_MOUSE and selected_particle:
        (mouseX, mouseY) = pg.mouse.get_pos()
        selected_particle.mouseMove(mouseX, mouseY)

    env.update()
    screen.fill(env.colour)
    for r in env.entities['swarm']:
        pg.draw.circle(screen, r.colour, (int(r.x), int(r.y)), r.size, r.thickness)
    for ob in env.entities['obstacles']:
        pg.draw.rect(screen, ob.colour, (int(ob.x), int(ob.y), ob.size / 1.414, ob.size / 1.414), ob.thickness)

    pg.display.flip()

    #
    # buff = pg.image.tostring(screen, "P")
    # buff = np.fromstring(buff, dtype=np.uint8)
    # buff = buff.reshape(width, height)
    # plt.imshow(buff)
    # plt.show()
