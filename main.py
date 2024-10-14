import numpy as np
from math import *
import time
import pygame
from tkinter import *

# % Link | d   | theta        |   a   | alpha
# % 1      d1    theta_1         0       90
# % 2      0     theta_2         a2      0
# % 3      0     theta_3         a3     0
d = np.array([100, 0, 0])
a = np.array([0, 100, 100])
alpha = np.array([pi/2, 0, 0])

theta = np.array([0, 0, 0])
offsetTheta = np.array([pi/18, pi/9, pi/36])
limitTheta = np.array([[-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2]])
animateArray = np.array([])
travelArray = np.array([])

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

def updateTranposeMatrix():
    T0 = np.eye(4)
    T1 = np.dot(T0, DH_Matrix(d[0], theta[0]+offsetTheta[0], a[0], alpha[0]))
    T2 = np.dot(T1, DH_Matrix(d[1], theta[1]+offsetTheta[1], a[1], alpha[1]))
    T3 = np.dot(T2, DH_Matrix(d[2], theta[2]+offsetTheta[2], a[2], alpha[2]))
    return [T0, T1, T2, T3]

baseO = np.array([0, 0, 0])
baseX = np.array([50, 0, 0])
baseY = np.array([0, 50, 0])
baseZ = np.array([0, 0, 50])

scale = 2
angleX = -3*pi/4
angleY = 0
angleZ = -pi/4
angleStep = 0.02
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (247, 253, 4)
ORANGE = (251, 147, 0)
BLACK = (0, 0, 0)
WIDTH, HEIGHT = 800, 500
circle_pos = [WIDTH/2, HEIGHT*3/4]  # x, y
pygame.display.set_caption("3D")
screen = pygame.display.set_mode((WIDTH, HEIGHT))

def DHtranspose(pos, Tm):
    pos = np.append(pos, 1)
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
    
def drawCoor(screen, poso, posx, posy, posz):
    o = projective(poso)
    x = projective(posx)
    y = projective(posy)
    z = projective(posz)
    pygame.draw.circle(screen, BLACK, o, 3)
    pygame.draw.line(screen, RED, o, x, width=2)
    pygame.draw.line(screen, BLUE, o, y, width=2)
    pygame.draw.line(screen, GREEN, o, z, width=2)
def drawCircle(screen, pos, color=RED, radius=1, width=0):
    pos = projective(pos)
    pygame.draw.circle(screen, color, pos, radius, width)
def drawLine(screen, posx, posy, color=ORANGE, width=1):
    x = projective(posx)
    y = projective(posy)
    pygame.draw.line(screen, color, x, y, width)
def drawPolygon(screen, posArray, color):
    pygame.draw.polygon(screen, color, [projective(p) for p in posArray], 0)
    #pygame.draw.polygon(screen, BLACK, [projective(p) for p in posArray], 1)
def drawTravel(screen):
    if travelArray.size > 0:
        travel = np.reshape(travelArray, (-1,3))
        for i in range(len(travel)-1):
            drawLine(screen, travel[i], travel[i+1], width=2)
def drawMachine(opacity, T):
    s = pygame.Surface((WIDTH,HEIGHT), pygame.SRCALPHA)
    # s.fill((255,255,255,opacity)) 
    ground = np.array([[-100, -100, 0], [-100, 100, 0], [100, 100, 0], [100, -100, 0]])
    drawPolygon(s, ground ,(0,0,0,opacity))     
    drawCircle(s, DHtranspose(baseO, T[0]), color=(255,0,0,opacity), radius=5)
    drawCircle(s, DHtranspose(baseO, T[1]), color=(255,0,0,opacity), radius=5)
    drawCircle(s, DHtranspose(baseO, T[2]), color=(255,0,0,opacity), radius=5)
    drawCircle(s, DHtranspose(baseO, T[3]), color=(255,0,0,opacity), radius=5)
    drawLine(s, DHtranspose(baseO, T[0]), DHtranspose(baseO, T[1]), color=(230,0,180,opacity), width=5)
    drawLine(s, DHtranspose(baseO, T[1]), DHtranspose(baseO, T[2]), color=(230,0,180,opacity), width=5)
    drawLine(s, DHtranspose(baseO, T[2]), DHtranspose(baseO, T[3]), color=(230,0,180,opacity), width=5)
    screen.blit(s, (0,0))

def updateTheta():
    global theta, animateArray, travelArray
    if animateArray.size > 0:
        theta = animateArray[0]
        animateArray = animateArray[1:]
    T = updateTranposeMatrix()
    for i in range(2):
        guiCoor[i].set(coor2str(DHtranspose(baseO, T[i+2])))

    travelArray = np.append(travelArray, DHtranspose(baseO, T[3]))
    return T

def coor2str(coor):
    return str(round(coor[0], 3)) + " | " + str(round(coor[1], 3)) + " | " + str(round(coor[2], 3))

clock = pygame.time.Clock()
def pygameMain():
    global angleX,angleY,angleZ,scale

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
    if keys[pygame.K_EQUALS]:
        scale += 0.1
    if keys[pygame.K_MINUS]:
        scale -= 0.1

    screen.fill(WHITE)
    T = updateTheta()
    #T = updateTranposeMatrix()
    drawMachine(guiOpacity.get(), T)
    if isDrawCoor.get() == 1:
        for t in T:
            drawCoor(screen, DHtranspose(baseO, t), DHtranspose(baseX, t), DHtranspose(baseY, t), DHtranspose(baseZ, t))
    if isDrawTravel.get() == 1:
        drawTravel(screen)
    pygame.display.update()
    root.update()
    clock.tick(30)

root = Tk()
root.title("Control Panel")

guiTheta = [DoubleVar(), DoubleVar(), DoubleVar()]
guiOpacity = DoubleVar()
isDrawCoor = IntVar()
isDrawTravel = IntVar()
guiOpacity.set(125)
isDrawCoor.set(1)
isDrawTravel.set(1)
guiCoor = [StringVar(), StringVar()]

def updateGuiVariable():
    global guiTheta
    for i in range(len(guiTheta)):
        guiTheta[i].set(theta[i]*180/pi)
def forwardKine():
    global animateArray, travelArray
    deltaTime = 60
    nextTheta = np.array([guiTheta[0].get()*pi/180, guiTheta[1].get()*pi/180, guiTheta[2].get()*pi/180])
    stepAngle = np.array([(nextTheta[0]-theta[0])/deltaTime, (nextTheta[1]-theta[1])/deltaTime, (nextTheta[2]-theta[2])/deltaTime])
    temp = np.array([0, 0, 0])
    for j in range(deltaTime):
        temp = temp+stepAngle
        animateArray = np.append(animateArray, theta+temp)
    animateArray = np.append(animateArray, nextTheta)
    animateArray = np.reshape(animateArray, (-1,3))
    travelArray = np.array([])

updateGuiVariable()
frameView = LabelFrame(root, text='View', padx=10, pady=10)
frameView.grid(row=0, column=0)
Label(frameView, text="Opacity", font='Helvetica 10').grid(row=0, column=0)
scale1 = Scale(frameView, variable=guiOpacity, showvalue=0, resolution=1,  from_ = 0, to = 255,  orient = HORIZONTAL, length=200) 
scale1.grid(row=0, column=1)
entry1 = Entry(frameView, textvariable=guiOpacity, width=5)
entry1.grid(row=0, column=2)
c1 = Checkbutton(frameView, text='Draw Coordinate',variable=isDrawCoor, onvalue=1, offvalue=0)
c1.grid(row=1, column=1)
c2 = Checkbutton(frameView, text='Draw Travel',variable=isDrawTravel, onvalue=1, offvalue=0)
c2.grid(row=1, column=0)
Label(frameView, text="Coordinate X | Y | Z", font='Helvetica 10', fg="red").grid(row=2, columnspan = 3, column=0)
Label(frameView, text="Joint 2", font='Helvetica 10').grid(row=3, column=0)
Label(frameView, textvariable=guiCoor[0], font='Helvetica 10').grid(row=3, column=1)
Label(frameView, text="Tool", font='Helvetica 10').grid(row=4, column=0)
Label(frameView, textvariable=guiCoor[1], font='Helvetica 10').grid(row=4, column=1)

frameForw = LabelFrame(root, text='Forward Kinematic', padx=10, pady=10)
frameForw.grid(row=1, column=0)
Label(frameForw, text="Forward Kinematic", font='Helvetica 10', fg="red").grid(row=0, columnspan = 3, column=0)
for i in range(len(theta)):
    Label(frameForw, text="Theta "+str(i+1), font='Helvetica 10').grid(row=i+1, column=0)
    scale2 = Scale(frameForw, variable=guiTheta[i], showvalue=0, resolution=1,  from_ = -90, to = 90,  orient = HORIZONTAL, length=200) 
    scale2.grid(row=i+1, column=1)
    entry2 = Entry(frameForw, textvariable=guiTheta[i], width=5)
    entry2.grid(row=i+1, column=2)
buttonForw = Button(frameForw, text="Execute", command=forwardKine, height=2, width=5).grid(row=4, column=2)
while True:
    pygameMain()
