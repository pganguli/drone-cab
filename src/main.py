import random
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

sumoCmd: list[str] = [
    "sumo",
    "-c",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "config.sumocfg"),
    "-d",
    "150",
]

traci.start(sumoCmd)

MAP_BOUNDARY = (1734.66, 1581.42)
WAREHOUSE_ID = "239796134"
EDGE_COUNTER = 0
PICKUP_COUNTER = 0


def shape2centroid(shape: list[tuple[float, float]]) -> tuple[float, float]:
    return tuple(np.mean(np.asarray(shape), axis=0))


def euclidean_distance(
    point_a: tuple[float, float], point_b: tuple[float, float]
) -> float:
    return float(np.linalg.norm(np.asarray(point_a) - np.asarray(point_b)))


def get_nearest_edge(polygon_id: str, lane_id_list: list[str]) -> str:
    global EDGE_COUNTER

    nearest_lane_id = min(
        lane_id_list,
        key=lambda lane_id: euclidean_distance(
            shape2centroid(traci.polygon.getShape(polygon_id)),
            shape2centroid(traci.lane.getShape(lane_id)),
        ),
    )
    nearest_lane_center = shape2centroid(traci.lane.getShape(nearest_lane_id))

    traci.polygon.add(
        polygonID=f"edge#{EDGE_COUNTER}",
        shape=[
            (nearest_lane_center[0], nearest_lane_center[1]),
            (nearest_lane_center[0] + 10, nearest_lane_center[1]),
            (nearest_lane_center[0] + 10, nearest_lane_center[1] + 10),
            (nearest_lane_center[0], nearest_lane_center[1] + 10),
            (nearest_lane_center[0], nearest_lane_center[1]),
        ],
        color=[0, 255, 0],
        polygonType="edge",
        fill=True,
    )
    EDGE_COUNTER += 1

    nearest_edge_id = traci.lane.getEdgeID(nearest_lane_id)
    return nearest_edge_id


def select_warehouse() -> str:
    global WAREHOUSE_ID

    warehouse_id = WAREHOUSE_ID
    traci.polygon.setColor(warehouse_id, (0, 0, 255))
    return warehouse_id


def get_pickup_id_list(
    radius_increment: int = 200, pickups_per_circle: int = 3
) -> list[str]:
    global PICKUP_COUNTER

    angles = [0, 30, 60, 90, 120, 150, 180, 210, 240, 265, 300, 330]

    radius = radius_increment
    pickup_id_list: list[str] = []
    while True:
        for _ in range(pickups_per_circle):
            pickup_id = f"pickup#{PICKUP_COUNTER}"
            theta = math.radians(angles[int(pickup_id[-1])])
            pickup_center = (
                radius * math.cos(theta),
                radius * math.sin(theta),
            ) + warehouse_center

            if pickup_center < MAP_BOUNDARY:
                traci.polygon.add(
                    polygonID=pickup_id,
                    shape=[
                        (pickup_center[0], pickup_center[1]),
                        (pickup_center[0] + 15, pickup_center[1]),
                        (pickup_center[0] + 15, pickup_center[1] + 15),
                        (pickup_center[0], pickup_center[1] + 15),
                        (pickup_center[0], pickup_center[1]),
                    ],
                    color=[255, 0, 0],
                    polygonType="pickup",
                    fill=True,
                )
                pickup_id_list.append(pickup_id)
                PICKUP_COUNTER += 1
            else:
                return pickup_id_list
        radius += radius_increment


def get_pickup_capacity_dict(pickup_id_list: list[str]) -> dict[int, int]:
    return {
        pickup_id: random.randint(5, 15) for pickup_id in range(len(pickup_id_list))
    }


def get_pickup_edge_dict(
    pickup_id_list: list[str], lane_id_list: list[str]
) -> dict[str, str]:
    return {
        pickup_id: get_nearest_edge(pickup_id, lane_id_list)
        for pickup_id in pickup_id_list
    }


def handle_package_request(destination_id: str):
    traci.polygon.setColor(destination_id, (222, 52, 235))
    assignPickup(destination_id)


def assignPickup(package):
    global package_pickup_matrix
    new_assignment = np.zeros(PICKUP_COUNTER)

    package_center = shape2centroid(traci.polygon.getShape(package))

    package_pickup_distance = [
        euclidean_distance(
            package_center,
            shape2centroid(traci.polygon.getShape(pickup)),
        )
        for pickup in pickup_id_list
    ]

    selected_pickup = 0
    while package_pickup_distance != [math.inf] * PICKUP_COUNTER:
        selected_pickup = package_pickup_distance.index(min(package_pickup_distance))
        if np.any(package_pickup_matrix):
            if (
                package_pickup_matrix[:, selected_pickup].sum()
                < pickup_capacity_dict[selected_pickup]
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


def assignVehicle(pickup: int):
    global vehicle_load, package_vehicle_mapping, pickup_vehicle_mapping, warehouse_edge_id

    pickup_id = f"pickup#{pickup}"
    if pickup_id not in pickup_vehicle_mapping:
        pickup_vehicle_mapping[pickup_id] = set()

    vehicle_list = list(traci.vehicle.getIDList())

    for i in vehicle_list:
        if int(i) not in vehicle_load:
            vehicle_load[int(i)] = 0

    for vehicle in vehicle_list:
        vehicle_route = traci.vehicle.getRouteID(vehicle)
        vehicle_edges = traci.route.getEdges(vehicle_route)
        if warehouse_edge_id not in vehicle_edges:
            vehicle_list.remove(vehicle)
            continue

    min_dist = math.inf
    selected_vehicle = None
    for vehicle in vehicle_list:
        vehicle_route = traci.vehicle.getRouteID(vehicle)
        vehicle_edges = traci.route.getEdges(vehicle_route)
        if (
            vehicle_load[int(vehicle)] < vehicle_capacity
            and pickup_edge_dict[pickup_id] in vehicle_edges
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
    if selected_vehicle is not None:
        package_vehicle_mapping["package#" + str(package_no)] = int(selected_vehicle)
        pickup_vehicle_mapping["pickup#" + str(pickup)].add(selected_vehicle)
        vehicle_load[int(selected_vehicle)] += 1
        traci.vehicle.setColor(selected_vehicle, (0, 255, 0))


def isDropped():
    global pickup_vehicle_mapping
    for pickup in pickup_vehicle_mapping:
        if len(pickup_vehicle_mapping[pickup]) > 0:
            for vehicle in list(pickup_vehicle_mapping[pickup]):
                if traci.vehicle.getRoadID(vehicle) == pickup_edge_dict[pickup]:
                    pickup_vehicle_mapping[pickup].remove(vehicle)

            if len(pickup_vehicle_mapping[pickup]) == 0:
                droneTSP(pickup)


def droneTSP(pickup: str):  # parameter should be of form "pickup#k"
    pickup_center = shape2centroid(traci.polygon.getShape(pickup))
    pickup_cost = 0

    coordinates = [pickup_center]

    for num in np.nonzero(package_pickup_matrix)[0]:
        if ("package#" + str(num)) in package_vehicle_mapping:
            package = destination_id_list[num]
            package_center = shape2centroid(traci.polygon.getShape(package))

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


def get_lane_id_list() -> list[str]:
    return traci.lane.getIDList()


def filter_internal_list_id(id_list: list[str]) -> list[str]:
    return list(filter(lambda id: id.startswith(":"), id_list))


def get_polygon_id_list() -> list[str]:
    return traci.polygon.getIDList()


def get_building_id_list(polygon_id_list: list[str]) -> list[str]:
    return list(
        filter(
            lambda polygon_id: traci.polygon.getType(polygon_id) != "building",
            polygon_id_list,
        )
    )


if __name__ == "__main__":
    lane_id_list: list[str] = filter_internal_list_id(get_lane_id_list())

    warehouse_id = select_warehouse()
    warehouse_center = shape2centroid(traci.polygon.getShape(warehouse_id))
    warehouse_edge_id = get_nearest_edge(warehouse_id, lane_id_list)

    pickup_id_list = get_pickup_id_list()
    pickup_capacity_dict = get_pickup_capacity_dict(pickup_id_list)
    pickup_edge_dict = get_pickup_edge_dict(pickup_id_list, lane_id_list)

    package_pickup_matrix = np.array([])
    package_vehicle_mapping = {}
    pickup_vehicle_mapping = {}

    vehicle_load = {}
    vehicle_capacity = 2

    for step in range(200):
        traci.simulationStep()

        if step == 30:
            package_no = 0
            destination_id_list = ["234807099", "239713538", "359039090"]
            for destination_id in destination_id_list:
                handle_package_request(destination_id)
                package_no += 1
            print(package_pickup_matrix)
            print(package_vehicle_mapping)

        isDropped()

    traci.close()
