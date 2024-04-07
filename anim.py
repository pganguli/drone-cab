import matplotlib.animation as animation
import matplotlib.pyplot as plt
import networkx as nx

fig = plt.figure(figsize=(5, 4))
ax = fig.add_subplot(autoscale_on=False, xlim=(-100, 100), ylim=(-100, 100))
ax.set_aspect("equal")
ax.grid()

dt = 0.01  # seconds between frames
time_text = ax.text(x=0.01, y=0.95, s="", transform=ax.transAxes)

RESIDENCE_CENTERS = [
    (-20, -70),
    (-20, 70),
    (-30, -40),
    (-30, 40),
    (-60, -80),
    (-60, 80),
    (-80, -50),
    (-80, 50),
    (20, -40),
    (20, 40),
    (20, 40),
    (30, -80),
    (30, 80),
    (30, 80),
    (60, -50),
    (60, 50),
    (60, 50),
    (80, -70),
    (80, 70),
    (80, 70),
]
for x, y in RESIDENCE_CENTERS:
    ax.add_artist(plt.Circle(xy=(x, y), radius=1, color="blue"))
DRONE_CENTER = (0, 0)
(drone,) = ax.plot(DRONE_CENTER, marker="x", color="red", markersize=10)
(trace,) = ax.plot([], [], lw=1, color="black")

x, y = DRONE_CENTER
history_x, history_y = [x], [y]


def update(i):
    history_x.append(x)
    history_y.append(y)

    drone.set_data(history_x[i], history_y[i])
    trace.set_data(history_x[:i], history_y[:i])

    time_text.set_text(f"time = {i * dt:.1f}s")

    return drone, trace, time_text


anim = animation.FuncAnimation(fig, update, frames=10000, interval=dt * 1000, blit=True)

plt.show()
