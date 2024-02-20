import os
import sys
import numpy as np
import random
import math
import networkx as nx
from scipy.spatial import distance_matrix
import matplotlib.pyplot as plt

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

# import traci.constants as tc


def shape2center(shape):
    tuple_sum = tuple([sum(x) for x in zip(*shape)])
    num = len(shape)
    return (tuple_sum[0] / num, tuple_sum[1] / num)


def euclidean_distance(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


sumoCmd = ["sumo-gui", "-c", "test.sumocfg", "-d", "150"]

traci.start(sumoCmd)

map_boundary = (1734.66, 1581.42)

polygon_id_list = traci.polygon.getIDList()
polygon_type_list = [traci.polygon.getType(id) for id in polygon_id_list]

building_list = [
    polygon_id_list[index]
    for index, i in enumerate(polygon_type_list)
    if i == "building"
]
building_center = [shape2center(traci.polygon.getShape(id)) for id in building_list]
buildings = list(zip(building_list, building_center))

lane_list = traci.lane.getIDList()
lane_list = list(filter(lambda x: x[0] != ":", lane_list))
lane_center = [shape2center(traci.lane.getShape(id)) for id in lane_list]
lanes = list(zip(lane_list, lane_center))


warehouse = "239796134"
traci.polygon.setColor(warehouse, (0, 0, 255))
warehouse_center = shape2center(traci.polygon.getShape(warehouse))

warehouse_edge = None
min_dist = math.inf
for i in lanes:
    x = euclidean_distance(i[1], warehouse_center)
    if x < min_dist:
        min_dist = x
        warehouse_edge = traci.lane.getEdgeID(i[0])

pickups = []

radius = 200
radius_increment = 200
no_of_pickups_in_radius = 3
pickup_id = "pickup#0"
# angles = list(range(0, 360, 30))
# random.shuffle(angles)
angles = [0, 265, 90, 300, 180, 60, 330, 150, 120, 240, 210, 30]
flag = False

while True:
    for _ in range(no_of_pickups_in_radius):
        theta = math.radians(angles[int(pickup_id[-1])])
        x_coordinate = radius * math.cos(theta) + warehouse_center[0]
        y_coordinate = radius * math.sin(theta) + warehouse_center[1]
        if int(x_coordinate) in range(int(map_boundary[0])) and int(
            y_coordinate
        ) in range(int(map_boundary[1])):
            traci.polygon.add(
                polygonID=pickup_id,
                shape=[
                    (x_coordinate, y_coordinate),
                    (x_coordinate + 15, y_coordinate),
                    (x_coordinate + 15, y_coordinate + 15),
                    (x_coordinate, y_coordinate + 15),
                    (x_coordinate, y_coordinate),
                ],
                color=[255, 0, 0],
                polygonType="pickup",
                fill=1,
            )
            pickups.append((pickup_id, (x_coordinate + 7, y_coordinate + 7)))
            pickup_id = pickup_id[:-1] + str(int(pickup_id[-1]) + 1)
        else:
            flag = True
            break
    if flag:
        break
    radius += radius_increment

no_of_pickups = len(pickups)

pickup_capacity = {}
for i in range(no_of_pickups):
    # pickup_capacity[i] = random.randint(2, 5)
    pickup_capacity[i] = 5

pickup_edges = []
edge_id = "edge#0"
for i in pickups:
    pickup_edge = None
    min_dist = math.inf
    angle = 0
    for j in lanes:
        x = euclidean_distance(i[1], j[1])
        if x < min_dist:
            min_dist = x
            pickup_edge = traci.lane.getEdgeID(j[0])
            angle = traci.lane.getShape(j[0])
            x_coordinate, y_coordinate = j[1]
    traci.polygon.add(
        polygonID=edge_id,
        shape=[
            (x_coordinate, y_coordinate),
            (x_coordinate + 5, y_coordinate),
            (x_coordinate + 5, y_coordinate + 5),
            (x_coordinate, y_coordinate + 5),
            (x_coordinate, y_coordinate),
        ],
        color=[0, 255, 0],
        polygonType="edge",
        fill=1,
    )
    edge_id = edge_id[:-1] + str(int(edge_id[-1]) + 1)
    pickup_edges.append(pickup_edge)

step = 0
# no_of_packages = 15
no_of_packages = 3
package_pickup_matrix = np.zeros((no_of_packages, no_of_pickups))
package_vehicle_mapping = {}

vehicle_load = {}
vehicle_capacity = 2

while step < 130:

    traci.simulationStep()

    if step == 30:
        package_no = 0
        # packages=[]
        packages = ["234807099", "239713538", "359039090"]
        while package_no < no_of_packages:
            # new_package_request = random.sample(buildings, k=1)
            # packages.append(new_package_request[0])
            # traci.polygon.setColor(new_package_request[0][0], (222, 52, 235))
            # package_center = new_package_request[0][1]

            new_package_request = packages[package_no]
            traci.polygon.setColor(new_package_request, (222, 52, 235))
            package_center = shape2center(traci.polygon.getShape(new_package_request))

            package_pickup_distance = [
                euclidean_distance(package_center, center) for _, center in pickups
            ]

            while package_pickup_distance != [math.inf] * no_of_pickups:
                selected_pickup = package_pickup_distance.index(
                    min(package_pickup_distance)
                )
                if (
                    package_pickup_matrix[:, selected_pickup].sum()
                    < pickup_capacity[selected_pickup]
                ):
                    package_pickup_matrix[package_no, selected_pickup] = 1
                    break
                else:
                    print(package_no, selected_pickup)
                    package_pickup_distance[selected_pickup] = math.inf
            package_no += 1

    if step == 32:
        vehicle_list = list(traci.vehicle.getIDList())

        for i in vehicle_list:
            if int(i) not in vehicle_load:
                vehicle_load[int(i)] = 0

        for vehicle in vehicle_list:
            vehicle_route = traci.vehicle.getRouteID(vehicle)
            vehicle_edges = traci.route.getEdges(vehicle_route)
            if warehouse_edge not in vehicle_edges:
                vehicle_list.remove(vehicle)
                continue

        for num, package in enumerate(package_pickup_matrix):
            if ("package#" + str(num)) not in package_vehicle_mapping:
                selected_pickup = np.where(package == 1)[0][0]
                min_dist = math.inf
                selected_vehicle = None
                for vehicle in vehicle_list:
                    vehicle_route = traci.vehicle.getRouteID(vehicle)
                    vehicle_edges = traci.route.getEdges(vehicle_route)
                    if (
                        vehicle_load[int(vehicle)] < vehicle_capacity
                        and pickup_edges[selected_pickup] in vehicle_edges
                    ):
                        # x = euclidean_distance(
                        #     pickups[selected_pickup][1], traci.vehicle.getPosition(vehicle)
                        # )
                        # x = traci.vehicle.getDrivingDistance2D(
                        #     vehicle,
                        #     pickups[selected_pickup][1][0],
                        #     pickups[selected_pickup][1][1],
                        # )
                        x = traci.simulation.getDistance2D(
                            # vehicle,
                            traci.vehicle.getPosition(vehicle)[0],
                            traci.vehicle.getPosition(vehicle)[1],
                            warehouse_center[0],
                            warehouse_center[1],
                            isDriving=True
                        )
                        if x < min_dist:
                            min_dist = x
                            selected_vehicle = vehicle
                if selected_vehicle != None:
                    package_vehicle_mapping["package#" + str(num)] = int(
                        selected_vehicle
                    )
                    vehicle_load[int(selected_vehicle)] += 1
                    traci.vehicle.setColor(selected_vehicle, (0, 255, 0))
    step += 1

print(package_pickup_matrix)
print(package_vehicle_mapping)
# print(vehicle_load)

for num0, pickup_ in enumerate(package_pickup_matrix.T):

    pickup = "pickup#" + str(num0)
    pickup_center = traci.polygon.getShape(pickup)[0]
    pickup_cost = 0

    coordinates = [pickup_center]

    for num1, package_ in enumerate(np.where(pickup_ == 1)[0]):

        if ("package#" + str(num1)) in package_vehicle_mapping:
            # package = packages[num1][0]
            # package_center = packages[num1][1]

            package = packages[num1]
            package_center = traci.polygon.getShape(package)[0]

            coordinates.append(package_center)

    if len(coordinates) > 1:
        coordinates = np.array(coordinates)
        dist_matrix = distance_matrix(coordinates, coordinates)
        dist_matrix = np.round(dist_matrix, decimals=2)
        plt.figure(figsize=(10, 6))
        G = nx.from_numpy_array(dist_matrix)
        path = nx.approximation.christofides(G)
        pos = {}
        for node in G.nodes():
            pos[node] = [coordinates[node, 0], coordinates[node, 1]]
        nx.draw_networkx_nodes(G, pos=pos, nodelist=[0], node_color="#ff0000")
        nx.draw_networkx_nodes(
            G, pos=pos, nodelist=list(G.nodes)[1:], node_color="#df34eb"
        )
        edgelist = []
        for i in range(0, len(path) - 1):
            edgelist.append((path[i], path[i + 1]))
            pickup_cost += G.get_edge_data(path[i], path[i + 1])["weight"]
        print(f"Drone travel distance for {pickup}: {pickup_cost:.2f} m")
        nx.draw_networkx_edges(
            G,
            pos=pos,
            edgelist=G.edges() - (edgelist + [(j, i) for (i, j) in edgelist]),
        )
        nx.draw_networkx_edges(
            G,
            pos=pos,
            edgelist=edgelist,
            edge_color="darkorange",
            arrows=True,
            arrowstyle="-|>",
        )
        edge_labels = nx.get_edge_attributes(G, "weight")
        nx.draw_networkx_edge_labels(
            G, pos=pos, edge_labels=edge_labels, label_pos=0.25
        )
        labels = {}
        labels[0] = pickup
        for node in list(G.nodes)[1:]:
            labels[node] = "package#" + str(node - 1)
        for i, p in enumerate(pos):
            if i % 2 == 0:
                pos[p][1] -= 7
            else:
                pos[p][1] += 7
        nx.draw_networkx_labels(G, pos=pos, labels=labels, font_size=10)
        plt.show()

traci.close()
