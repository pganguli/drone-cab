import math
import os
import sys

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from scipy.spatial import distance_matrix

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

# import traci.constants as tc

sumoCmd: list[str] = ["sumo", "-c", "test.sumocfg", "-d", "150"]

traci.start(sumoCmd)

map_boundary: tuple[float, float] = (1734.66, 1581.42)


def shape2center(shape):
    tuple_sum = tuple([sum(x) for x in zip(*shape)])
    num = len(shape)
    return (tuple_sum[0] / num, tuple_sum[1] / num)


def euclidean_distance(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def edgeAssignment(id):
    global edge_id
    edge = None
    center = shape2center(traci.polygon.getShape(id))
    min_dist = math.inf
    for lane in lanes:
        x, y = shape2center(traci.lane.getShape(lane))
        k = euclidean_distance((x, y), center)
        if k < min_dist:
            min_dist = k
            edge = traci.lane.getEdgeID(lane)
            x_coordinate, y_coordinate = x, y
    traci.polygon.add(
        polygonID=edge_id,
        shape=[
            (x_coordinate, y_coordinate),
            (x_coordinate + 10, y_coordinate),
            (x_coordinate + 10, y_coordinate + 10),
            (x_coordinate, y_coordinate + 10),
            (x_coordinate, y_coordinate),
        ],
        color=[0, 255, 0],
        polygonType="edge",
        fill=1,
    )
    edge_id = edge_id[:-1] + str(int(edge_id[-1]) + 1)
    return edge


def warehouseSelection(id="239796134"):
    global warehouse
    warehouse = id
    traci.polygon.setColor(warehouse, (0, 0, 255))


def pickupGeneration():
    global pickups
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
                pickups.append(pickup_id)
                pickup_id = pickup_id[:-1] + str(int(pickup_id[-1]) + 1)
            else:
                flag = True
                break
        if flag:
            break
        radius += radius_increment


def newPackageRequest(id):
    new_package_request = id
    traci.polygon.setColor(new_package_request, (222, 52, 235))
    assignPickup(id)


def assignPickup(package):
    global package_pickup_matrix
    new_assignment = np.zeros(no_of_pickups)

    package_center = shape2center(traci.polygon.getShape(package))

    package_pickup_distance = [
        euclidean_distance(
            package_center,
            shape2center(traci.polygon.getShape(pickup)),
        )
        for pickup in pickups
    ]

    while package_pickup_distance != [math.inf] * no_of_pickups:
        selected_pickup = package_pickup_distance.index(min(package_pickup_distance))
        if np.any(package_pickup_matrix):
            if (
                package_pickup_matrix[:, selected_pickup].sum()
                < pickup_capacity[selected_pickup]
            ):
                new_assignment[selected_pickup] = 1
                new_assignment.reshape(1, -1)
                package_pickup_matrix = np.vstack(
                    (package_pickup_matrix, new_assignment)
                )
                break
            else:
                print(package_no, selected_pickup)
                package_pickup_distance[selected_pickup] = math.inf
        else:
            new_assignment[selected_pickup] = 1
            new_assignment = new_assignment.reshape(1, -1)
            package_pickup_matrix = np.copy(new_assignment)
            break

    assignVehicle(selected_pickup)


def assignVehicle(pickup):
    global vehicle_load, package_vehicle_mapping, pickup_vehicle_mapping

    pickup_id = "pickup#" + str(pickup)
    if pickup_id not in pickup_vehicle_mapping:
        pickup_vehicle_mapping[pickup_id] = set()

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

    min_dist = math.inf
    selected_vehicle = None
    for vehicle in vehicle_list:
        vehicle_route = traci.vehicle.getRouteID(vehicle)
        vehicle_edges = traci.route.getEdges(vehicle_route)
        if (
            vehicle_load[int(vehicle)] < vehicle_capacity
            and pickup_edges[pickup_id] in vehicle_edges
        ):
            x = traci.simulation.getDistance2D(
                traci.vehicle.getPosition(vehicle)[0],
                traci.vehicle.getPosition(vehicle)[1],
                warehouse_center[0],
                warehouse_center[1],
                isDriving=True,
            )
            if x < min_dist:
                min_dist = x
                selected_vehicle = vehicle
    if selected_vehicle != None:
        package_vehicle_mapping["package#" + str(package_no)] = int(selected_vehicle)
        pickup_vehicle_mapping["pickup#" + str(pickup)].add(selected_vehicle)
        vehicle_load[int(selected_vehicle)] += 1
        traci.vehicle.setColor(selected_vehicle, (0, 255, 0))


def isDropped():
    global pickup_vehicle_mapping
    for pickup in pickup_vehicle_mapping:
        if len(pickup_vehicle_mapping[pickup]) > 0:
            for vehicle in list(pickup_vehicle_mapping[pickup]):
                if traci.vehicle.getRoadID(vehicle) == pickup_edges[pickup]:
                    pickup_vehicle_mapping[pickup].remove(vehicle)

            if len(pickup_vehicle_mapping[pickup]) == 0:
                droneTSP(pickup)


def droneTSP(pickup):  # parameter should be of form "pickup#k"
    pickup_center = shape2center(traci.polygon.getShape(pickup))
    pickup_cost = 0

    coordinates = [pickup_center]

    for num in np.nonzero(package_pickup_matrix)[0]:
        if ("package#" + str(num)) in package_vehicle_mapping:
            package = packages[num]
            package_center = shape2center(traci.polygon.getShape(package))

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


# polygon_id_list = traci.polygon.getIDList()
# polygon_type_list = [traci.polygon.getType(id) for id in polygon_id_list]

# building_list = [
#     polygon_id_list[index]
#     for index, i in enumerate(polygon_type_list)
#     if i == "building"
# ]
# building_center = [shape2center(traci.polygon.getShape(id)) for id in building_list]
# buildings = list(zip(building_list, building_center))

lanes = traci.lane.getIDList()
lanes = list(filter(lambda x: x[0] != ":", lanes))

edge_id = "edge#0"

warehouse = None
warehouseSelection()

warehouse_center = shape2center(traci.polygon.getShape(warehouse))

warehouse_edge = edgeAssignment(warehouse)

pickups = []
pickupGeneration()

no_of_pickups = len(pickups)

pickup_capacity = {}
for i in range(no_of_pickups):
    # pickup_capacity[i] = random.randint(2, 5)
    pickup_capacity[i] = 5

pickup_edges = {}
for pickup in pickups:
    pickup_edge = edgeAssignment(pickup)
    pickup_edges[pickup] = pickup_edge

step = 0
package_pickup_matrix = np.array([])
package_vehicle_mapping = {}
pickup_vehicle_mapping = {}

vehicle_load = {}
vehicle_capacity = 2

while step < 200:
    traci.simulationStep()

    if step == 30:
        package_no = 0
        packages = ["234807099", "239713538", "359039090"]
        for package in packages:
            new_package_request = newPackageRequest(package)
            traci.polygon.setColor(package, (222, 52, 235))
            package_no += 1
        print(package_pickup_matrix)
        print(package_vehicle_mapping)

    isDropped()

    step += 1

traci.close()
