import numpy as np
from math import *
import time
import pygame
import tkinter
from tkinter import *

# % Link | d   | theta        |   a   | alpha
# % 1      d1    theta_1         0       90
# % 2      0     theta_2         a2      0
# % 3      0     theta_3         a3     0

def DH_Matrix(d, theta, a, alpha):
    c = cos(theta)
    s = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)
    return np.array([
        [c, -ca*s, sa*s, a*c],
        [s, c*ca, -sa*c, a*s],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])

d1 = 100
theta1 = theta2 = theta3 = 0
a2 = a3 = 100
T0 = np.eye(4)
T1 = np.dot(DH_Matrix(d1, theta1, 0, pi/2), T0)
T2 = np.dot(DH_Matrix(0, theta2, a2, 0), T1)
T3 = np.dot(DH_Matrix(0, theta3, a3, 0), T2)
T = [T0, T1, T2, T3]
isDrawCoor = 1
baseO = np.array([0, 0, 0])
baseX = np.array([50, 0, 0])
baseY = np.array([0, 50, 0])
baseZ = np.array([0, 0, 50])
scale = 1
angleStep = 0.02
angleX = 0
angleY = 0
angleZ = 0
WHITE = (255, 255, 255)
RED = (245, 71, 72)
GREEN = (121, 212, 94)
BLUE = (49, 191, 243)
YELLOW = (247, 253, 4)
ORANGE = (251, 147, 0)
BLACK = (0, 0, 0)
WIDTH, HEIGHT = 800, 500
circle_pos = [WIDTH/2, HEIGHT*3/4]  # x, y
pygame.display.set_caption("3D")
screen = pygame.display.set_mode((WIDTH, HEIGHT))

def DHtranspose(pos, Tm):
    pos = np.append(pos, 1)
    # pos = np.reshape(pos,(1,4))
    pos = np.dot(Tm, pos)
    return pos[:3]

def projective(pos):
    pos = pos.reshape((3, 1))
    rotation_z = np.array([
        [cos(angleZ), -sin(angleZ), 0],
        [sin(angleZ), cos(angleZ), 0],
        [0, 0, -1],
    ])
    rotation_y = np.array([
        [cos(angleY), 0, sin(angleY)],
        [0, 1, 0],
        [-sin(angleY), 0, cos(angleY)],
    ])
    rotation_x = np.array([
        [1, 0, 0],
        [0, cos(angleX), -sin(angleX)],
        [0, sin(angleX), cos(angleX)],
    ])
    pos = np.dot(rotation_z, pos)
    pos = np.dot(rotation_y, pos)
    pos = np.dot(rotation_x, pos)
    x = pos[0][0]
    y = pos[1][0]
    z = pos[2][0]
    xx = int(x * scale) + circle_pos[0]
    yy = int(y * scale) + circle_pos[1]
    return (xx, yy)
    
def drawCoor(poso, posx, posy, posz):
    o = projective(poso)
    x = projective(posx)
    y = projective(posy)
    z = projective(posz)
    pygame.draw.circle(screen, BLACK, o, 2)
    pygame.draw.line(screen, RED, o, x, width=1)
    pygame.draw.line(screen, BLUE, o, y, width=1)
    pygame.draw.line(screen, GREEN, o, z, width=1)

clock = pygame.time.Clock()
def pygameMain():
    global angleX,angleY,angleZ
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                pygame.quit()
                exit()

    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:
        angleX += angleStep
    if keys[pygame.K_DOWN]:
        angleX -= angleStep
    if keys[pygame.K_RIGHT]:
        angleZ += angleStep
    if keys[pygame.K_LEFT]:
        angleZ -= angleStep
    if keys[pygame.K_PERIOD]:
        angleY += angleStep
    if keys[pygame.K_COMMA]:
        angleY -= angleStep

    screen.fill(WHITE)
    if isDrawCoor:
        for t in T:
            drawCoor(DHtranspose(baseO, t), DHtranspose(baseX, t), DHtranspose(baseY, t), DHtranspose(baseZ, t))

    pygame.display.update()


root = Tk()
root.title("Robot")
frameControl = LabelFrame(root, text='Forward Kinematic', padx=10, pady=10)
frameControl.grid(row=0, column=1)
while True:
    pygameMain()
