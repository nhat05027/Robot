import matplotlib.pyplot as plt
import matplotlib.animation as animation


figure, axis = plt.subplots(3, 3)

# axis[0, 0].set_title("q(t)")
# axis[1, 0].set_title("v(t)")
# axis[2, 0].set_title("a(t)")

def animate(j):
    graph_data = open('data.txt','r').read()
    lines = graph_data.split('\n')
    t = []
    qt = [[], [], []]
    vt = [[], [], []]
    at = [[], [], []]
    qts = [0, 0, 0]
    vts = [0, 0, 0]
    ats = [0, 0, 0]
    for line in lines:
        if len(line) > 1:
            ts, qts[0], vts[0], ats[0], qts[1], vts[1], ats[1], qts[2], vts[2], ats[2] = line.split(',')
            t.append(float(ts))
            for i in range(3):
                qt[i].append(float(qts[i]))
                vt[i].append(float(vts[i]))
                at[i].append(float(ats[i]))

    for i in range(3):
        axis[0, i].clear()
        axis[1, i].clear()
        axis[2, i].clear()

        axis[0, i].plot(t, qt[i])
        axis[1, i].plot(t, vt[i], 'g')
        axis[2, i].plot(t, at[i], 'r')

ani = animation.FuncAnimation(figure, animate, interval=30, cache_frame_data=False)
plt.show()