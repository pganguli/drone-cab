from drone_cab.utils import euclidean_distance
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
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
DRONE_SPEED = 2
(drone,) = ax.plot(DRONE_CENTER, marker="x", color="red", markersize=10)
(trace,) = ax.plot([], [], lw=1, color="black")

x, y = DRONE_CENTER
history_x, history_y = [x], [y]

G = nx.Graph()
G.add_node(DRONE_CENTER)
for residence in RESIDENCE_CENTERS:
    G.add_node(residence)
G.add_weighted_edges_from(
    [
        (node_i, node_j, euclidean_distance(node_i, node_j))
        for node_i in G.nodes
        for node_j in G.nodes
        if node_i != node_j
    ]
)
path = nx.algorithms.approximation.christofides(G)
path_iter = iter(path)
target_x, target_y = next(path_iter)


def update(i):
    global x, y, target_x, target_y

    if x == target_x and y == target_y:
        try:
            target_x, target_y = next(path_iter)
        except StopIteration:
            return drone, trace, time_text

    dist_left = euclidean_distance((x, y), (target_x, target_y))
    theta = np.arctan(np.abs((target_y - y)) / np.abs((target_x - x)))

    if x < target_x:
        x += min(DRONE_SPEED, dist_left) * (np.cos(theta))
    else:
        x -= min(DRONE_SPEED, dist_left) * (np.cos(theta))

    if y < target_y:
        y += min(DRONE_SPEED, dist_left) * (np.sin(theta))
    else:
        y -= min(DRONE_SPEED, dist_left) * (np.sin(theta))

    history_x.append(x)
    history_y.append(y)

    drone.set_data([history_x[i]], [history_y[i]])
    trace.set_data(history_x[:i], history_y[:i])

    time_text.set_text(f"time = {i * dt:.1f}s")

    return drone, trace, time_text


anim = animation.FuncAnimation(
    fig, update, interval=dt * 1000, blit=True, cache_frame_data=False
)

plt.show()
