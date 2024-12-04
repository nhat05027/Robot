import numpy as np
import matplotlib.pyplot as plt

qmax = 200 #degree
vmax = 20000 #deg/s
amax = 400 #deg/s^2

def lspb(q0, qn, vmax, amax):
    qmax = qn-q0
    t1 = round((vmax/amax)/0.0333)*0.0333

    if qmax < amax*t1**2:
        t1 = round((np.sqrt(0.333*qmax*2/amax))/0.0333)*0.0333
        t2 = round(((qmax-amax*t1**2)/(amax*t1))/0.0333)*0.0333 + t1
        t3 = t2+t1

    else:
        t2 = round(((qmax-amax*t1**2)/vmax)/0.0333)*0.0333 + t1
        t3 = t2+t1


    t = np.arange(0, t3, 0.0333)

    qt = np.zeros(len(t), dtype=float)
    vt = np.zeros(len(t), dtype=float)
    at = np.zeros(len(t), dtype=float)

    for i, tt in enumerate(t):
        if tt <= t1:
            at[i] = amax
            vt[i] = at[i]*tt
            qt[i] = q0 + 0.5*at[i]*tt**2
        elif tt <= t2:
            at[i] = 0
            vt[i] = amax*t1
            qt[i] = q0 + 0.5*amax*t1**2 + vt[i]*(tt-t1)
        else:
            at[i] = -amax
            vt[i] = amax*t1 + at[i]*(tt-t2)
            qt[i] = q0 + 0.5*amax*t1**2 + amax*t1*(t2-t1) + amax*t1*(tt-t2) - 0.5*amax*(tt-t2)**2
    qt[-1] = qn
    vt[-1] = 0
    return t, qt, vt, at

def trajectory(nextTheta):
    global animateArray, travelArray
    temp = np.array([0, 0, 0])
    for j in range(deltaTime):
        temp = temp+stepAngle
        animateArray = np.append(animateArray, theta+temp)
    animateArray = np.append(animateArray, nextTheta)
    animateArray = np.reshape(animateArray, (-1,3))
    travelArray = np.array([])


