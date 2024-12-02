import numpy as np
import matplotlib.pyplot as plt

qmax = 200 #degree
vmax = 20000 #deg/s
amax = 400 #deg/s^2

t1 = vmax/amax

if qmax < vmax*t1:
    t1 = np.sqrt(0.2*qmax*2/amax)
    t2 = (qmax-amax*t1**2)/(amax*t1) + t1
    t3 = t2+t1

else:
    t2 = (qmax-amax*t1**2)/vmax + t1
    t3 = t2+t1

print(t1, t2, t3)

t = np.arange(0, t3, 0.05)

qt = np.zeros(len(t), dtype=float)
vt = np.zeros(len(t), dtype=float)
at = np.zeros(len(t), dtype=float)

for i, tt in enumerate(t):
    if i == 0:
        at[i] = 0
        vt[i] = 0
        qt[i] = 0
    elif tt <= t1:
        at[i] = amax
        vt[i] = at[i]*tt
        qt[i] = 0.5*at[i]*tt**2
    elif tt <= t2:
        at[i] = 0
        vt[i] = amax*t1
        qt[i] = 0.5*amax*t1**2 + vt[i]*(tt-t1)
    else:
        at[i] = -amax
        vt[i] = amax*t1 + at[i]*(tt-t2)
        qt[i] = 0.5*amax*t1**2 + amax*t1*(t2-t1) + 0.5*amax*(tt-t2)**2


figure, axis = plt.subplots(2, 2)

# q(t)
axis[0, 0].plot(t, qt)
axis[0, 0].set_title("q(t)")

# v(t)
axis[0, 1].plot(t, vt)
axis[0, 1].set_title("v(t)")

# a(t)
axis[1, 0].plot(t, at)
axis[1, 0].set_title("a(t)")


plt.show()