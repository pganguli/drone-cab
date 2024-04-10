import math
import random

from matplotlib.transforms import Affine2D

from drone_cab.utils import euclidean_distance
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Rectangle
import numpy as np
import networkx as nx

#
# Constants for experiment setup
#

DRONE_CENTER = (908.783925, 987.6503665714287)
DRONE_SPEED = 2
DRONE_RANGE = 200
RESIDENCE_CENTERS = [
    (910.3206866, 982.6598806),
    (897.7645155999999, 983.6238812000001),
    (922.8360150000001, 979.0601842),
    (885.4288334, 986.713737),
    (916.4433986, 1009.9812502000001),
    (903.5212868000001, 1012.4543188),
    (935.511288, 976.1618116),
    (929.5470677999999, 1008.9670884),
    (904.1988334, 955.896592),
    (891.9186599999999, 1016.1003201999999),
    (892.1095095999999, 958.160534),
    (874.1442128000001, 989.7869800000001),
    (917.0004632, 953.3382267999999),
    (940.6113167999999, 1004.4080469999999),
    (879.0596686000001, 959.5672158000001),
    (929.4756416, 951.8087621999999),
    (879.8345105999999, 1019.1261244),
    (949.6702131428572, 972.3344204285713),
    (954.6100411428572, 999.501606),
    (867.1061235999999, 962.9131648),
    (859.8642066, 990.7989104000001),
    (922.2138957999999, 1035.8376819999999),
    (909.2441746000001, 1038.7797787999998),
    (944.9507005714286, 949.1998157142858),
    (934.6688631999999, 1033.8296896),
    (866.4968436000001, 1022.0492722000001),
    (897.2342978, 1041.781912),
    (947.5591926000001, 1030.3686554),
    (854.0374629999999, 966.0913326),
    (899.3839294000002, 928.804644),
    (888.0411787999999, 931.4964256),
    (911.1884278, 926.650951),
    (883.9453622000001, 1043.6868098),
    (874.6398735999999, 934.1820111999999),
    (960.4853264, 1026.6023628),
    (924.1934728, 923.4633688),
    (862.8532508000001, 937.0556304000002),
    (970.3773014000001, 956.4991302000001),
    (871.5755000000001, 1045.8307352),
    (841.7862394, 969.6005736000001),
    (966.6869708, 942.0474138),
    (938.6003447142857, 919.1150458571428),
    (849.2660953999999, 938.5192589999999),
    (858.0868281999999, 1047.1114752),
    (963.5278920000001, 927.9594903999999),
    (924.7884723999999, 1067.5589888),
    (827.5495724, 970.8122552000001),
    (909.3331226, 1070.6436312),
    (940.8595832000001, 1064.3849696000002),
    (894.0721143999999, 904.8858782000001),
    (883.0663814, 907.1689496000001),
    (836.6484234000002, 942.5011658),
    (906.2434393999999, 902.2196631999999),
    (821.5524691999999, 999.2944396),
    (959.0346302, 1059.899954),
    (892.8427866, 1074.6552433999998),
    (868.5190344, 908.6057537999999),
    (961.4259366, 916.0363445999999),
    (844.0037944, 1049.2635616000002),
    (918.0183374000001, 897.1136936),
    (856.6796019999999, 911.8420657999999),
    (827.554931, 1031.296192),
    (932.008994, 896.6970000000001),
    (822.8893118, 945.4026488),
    (877.1089652000001, 1078.0473178),
    (996.7316752, 949.3832497999999),
    (813.6211413999999, 972.7023057999999),
    (844.6615038, 915.2693621999999),
    (959.8089497999999, 905.2383588),
]

# random.seed(1)
# random.shuffle(RESIDENCE_CENTERS)
RESIDENCE_CENTERS = RESIDENCE_CENTERS[:30]

radius = 0
for i in RESIDENCE_CENTERS:
    dist = euclidean_distance(i, DRONE_CENTER)
    if dist > radius:
        radius = dist
        farthest_residence = i

theta = ((DRONE_RANGE) / radius) - 2

#
# Matplotlib plot setup
#

fig = plt.figure(figsize=(100, 100))
ax = fig.add_subplot(
    autoscale_on=False,
    # xlim=(DRONE_CENTER[0] - DRONE_RANGE // 2, DRONE_CENTER[0] + DRONE_RANGE // 2),
    # ylim=(DRONE_CENTER[1] - DRONE_RANGE // 2, DRONE_CENTER[1] + DRONE_RANGE // 2),
    # xlim=(
    #     min(map(lambda x: x[0], RESIDENCE_CENTERS)) - 20,
    #     max(map(lambda x: x[0], RESIDENCE_CENTERS)) + 20,
    # ),
    # ylim=(
    #     min(map(lambda x: x[1], RESIDENCE_CENTERS)) - 20,
    #     max(map(lambda x: x[1], RESIDENCE_CENTERS)) + 20,
    # ),
)
ax.set_aspect("equal")
ax.grid()

dt = 0.01  # seconds between frames
time_text = ax.text(x=0.01, y=0.95, s="", transform=ax.transAxes)

# transform = Affine2D.from_values(1, 0, 0, 1, 0, 0)
# transform = Affine2D.from_values(
#     math.cos(math.pi / 2),
#     math.sin(math.pi / 2),
#     -math.sin(math.pi / 2),
#     math.cos(math.pi / 2),
#     ax.get_xlim()[0],
#     ax.get_ylim()[0],
# )

#
# Plot objects setup
#


ax.add_artist(plt.Circle(xy=DRONE_CENTER, radius=2, color="red"))
for x, y in RESIDENCE_CENTERS:
    c = "blue"
    if (x, y) == farthest_residence:
        c = "red"
    ax.add_artist(plt.Circle(xy=(x, y), radius=1, color=c))

(drone,) = ax.plot(DRONE_CENTER, marker="x", color="red", markersize=10)

farthest_distance = ax.plot(
    (DRONE_CENTER[0], farthest_residence[0]),
    (DRONE_CENTER[1], farthest_residence[1]),
    linestyle="--",
    color="orange",
)

sector = Wedge(DRONE_CENTER, radius, 0, math.degrees(theta), alpha=0.25, color="orange")
ax.add_patch(sector)

r1 = Rectangle(DRONE_CENTER, 20, 40, color="blue", alpha=0.50)
r2 = Rectangle(DRONE_CENTER, 2000, 4000, color="red", alpha=0.50)

t2 = Affine2D().rotate_deg(-45) + ax.transData
r2.set_transform(t2)

ax.add_patch(r1)
ax.add_patch(r2)

plt.xlim(
    min(map(lambda x: x[0], RESIDENCE_CENTERS)) - 20,
    max(map(lambda x: x[0], RESIDENCE_CENTERS)) + 20,
)
plt.ylim(
    min(map(lambda x: x[1], RESIDENCE_CENTERS)) - 20,
    max(map(lambda x: x[1], RESIDENCE_CENTERS)) + 20,
)


(trace,) = ax.plot([], [], lw=1, color="black")

#
# Drone path (visit order) setup
#

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

#
# Drone step function
#

x, y = next(path_iter)
assert (
    x,
    y,
) == DRONE_CENTER, (
    f"Drone attempted to take off from {(x, y)} instead of {DRONE_CENTER=}"
)
target_x, target_y = x, y
history_x, history_y = [], []


def update(i):
    global x, y, target_x, target_y

    #
    # Drone history trace record
    #

    history_x.append(x)
    history_y.append(y)

    drone.set_data([history_x[i]], [history_y[i]])
    trace.set_data(history_x[:i], history_y[:i])

    # Drone reached current target; set next target
    if (x, y) == (target_x, target_y):
        try:
            target_x, target_y = next(path_iter)
        except StopIteration:
            return drone, trace, time_text

    #
    # Drone step towards current target
    #

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

    time_text.set_text(f"time = {i * dt:.1f}s")

    return drone, trace, time_text


anim = animation.FuncAnimation(
    fig, update, interval=dt * 1000, blit=True, cache_frame_data=False
)

plt.show()
