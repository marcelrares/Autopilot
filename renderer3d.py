import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

def draw_car(x, z, color):
    glPushMatrix()
    glTranslatef(x, 0, z)

    glColor3f(*color)
    glBegin(GL_QUADS)

    # cub simplu (mașină)
    glVertex3f(-1,0,-2)
    glVertex3f(1,0,-2)
    glVertex3f(1,0,2)
    glVertex3f(-1,0,2)

    glEnd()
    glPopMatrix()

def run_3d(get_decision):
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, pygame.DOUBLEBUF | pygame.OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0, -1, -10)

    car_speed = 0.1

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        decision = get_decision()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # culoare în funcție de frână
        color = (0,1,0)
        if decision["brake"] == "soft":
            color = (1,1,0)
        if decision["brake"] == "hard":
            color = (1,0,0)

        # logică mișcare
        if decision["brake"] == "none":
            car_speed += 0.01
        else:
            car_speed *= 0.9

        draw_car(0, -car_speed, color)

        pygame.display.flip()
        pygame.time.wait(16)