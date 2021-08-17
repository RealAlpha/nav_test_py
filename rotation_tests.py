# NOTE: OpenGL code is based on the example/tutorial here:
# https://pythonprogramming.net/opengl-rotating-cube-example-pyopengl-tutorial/
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

quadric = None

vertices= (
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1)
    )

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )


def Cube(yaw, pitch, roll):
    #yaw, pitch, roll = -pitch, yaw, roll
    body_to_inertial_matrix = np.array([
        [np.cos(roll) * np.cos(pitch), np.cos(pitch) * np.sin(roll), -np.sin(pitch)],
        [np.cos(roll) * np.sin(yaw) * np.sin(pitch) - np.cos(yaw) * np.sin(roll),
         np.cos(yaw) * np.cos(roll) + np.sin(yaw) * np.sin(roll) * np.sin(pitch), np.cos(pitch) * np.sin(yaw)],
        [np.sin(yaw) * np.sin(roll) + np.cos(yaw) * np.cos(roll) * np.sin(pitch),
         np.cos(yaw) * np.sin(roll) * np.sin(pitch) - np.cos(roll) * np.sin(yaw), np.cos(yaw) * np.cos(pitch)]])

    # NED -> OpenGL's weird a$$ coordinate system
    """
    NED_to_opengl = np.array([
        [0, 0, 1],
        [1, 0, 0],
        [0, -1, 0]
    ])
    """
    NED_to_opengl = np.array([
        [1, 0, 0],
        [0, 0, 1],
        [0, -1, 0]
    ])

    #NED_to_opengl = np.identity(3)

    body_to_inertial_matrix = NED_to_opengl.T @  body_to_inertial_matrix @ NED_to_opengl

    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(body_to_inertial_matrix @ vertices[vertex])
    glEnd()


def drawVector(vec, start, color='white'):
    """
    Helper function for drawing vectors at locations
    :return:
    """
    glLineWidth(10)
    if color == 'red':
        glColor3f(1, 0, 0)
    elif color == 'green':
        glColor3f(0, 1, 0)
    elif color == 'blue':
        glColor3f(0, 0, 1)
    else:
        # Assume white is default / this also happens to be the default line color -> no need to change anything
        pass
    glBegin(GL_LINES)

    glVertex3fv(start)
    glVertex3fv(start + vec)
    glEnd()

    gluCylinder(quadric, 0.005, 0, 0.1, 20, 20)

    # Reset things
    glLineWidth(1)
    glColor3f(1, 1, 1)


def main():
    # Initialize screen for use with OpenGL
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)



    # Configure 45deg FoV
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    global quadric
    quadric = gluNewQuadric()
    gluQuadricNormals(quadric, GLU_SMOOTH)
    gluQuadricTexture(quadric, GL_TRUE)



    pitch = 0

    viewport_angular_rates = np.array([0, 0, 0])
    viewport_cur_angles = np.zeros(3)

    last_tick = 0

    while True:
        current_tick = pygame.time.get_ticks()
        dt = (current_tick - last_tick) / 1000
        last_tick = current_tick

        #glLoadIdentity()
        glPushMatrix()  # Pushign the matrix allows us to kind of reset the view changes we did using glRotatef without
        # worrying about re-intializing the perspective every time
        #gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
        #glTranslatef(0.0, 0.0, -5)

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        # Rotate the viewport to the desired angle - done first to ensure everything else is drawn according to this
        # "perspective"
        for i in range(3):
            # NOTE: The magic number 30 makes it easier/faster to move around
            glRotatef(30 * viewport_cur_angles[i], *np.identity(3)[i])

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    viewport_angular_rates += [1, 0, 0]
                if event.key == pygame.K_s:
                    viewport_angular_rates -= [1, 0, 0]
                if event.key == pygame.K_a:
                    viewport_angular_rates += [0, 1, 0]
                if event.key == pygame.K_d:
                    viewport_angular_rates -= [0, 1, 0]

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_w:
                    viewport_angular_rates -= [1, 0, 0]
                if event.key == pygame.K_s:
                    viewport_angular_rates += [1, 0, 0]
                if event.key == pygame.K_a:
                    viewport_angular_rates -= [0, 1, 0]
                if event.key == pygame.K_d:
                    viewport_angular_rates += [0, 1, 0]

        #glRotatef(100 * dt, viewport_angular_rates[0], viewport_angular_rates[1], viewport_angular_rates[2])


        #glRotatef(1, 3, 1, 1)

        # Draw coordinate system basis
        drawVector(np.array([1, 0, 0]), np.zeros(3), 'red')
        drawVector(np.array([0, 1, 0]), np.zeros(3), 'green')
        drawVector(np.array([0, 0, 1]), np.zeros(3), 'blue')

        Cube(0, pitch, 0)


        viewport_cur_angles += viewport_angular_rates * dt
        #print(viewport_angular_rates)
        #print(viewport_cur_angles)
        pygame.display.flip()
        glPopMatrix()
        #for i in range(3):
        #    glRotatef(-viewport_cur_angles[i], *np.identity(3)[i])
        pitch += 1 * dt
        pygame.time.wait(1)

main()