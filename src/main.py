import math
import os
# import random
import sys

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from scipy.spatial import distance_matrix

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

from .utils import (
    euclidean_distance,
    filter_internal_list_id,
    get_building_id_list,
    get_lane_id_list,
    get_nearest_edge,
    get_pickup_edge_dict,
    get_polygon_id_list,
    shape2centroid,
)

sumoCmd: list[str] = [
    "sumo",
    "-c",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "data", "config.sumocfg"
    ),
    "-d",
    "150",
]

traci.start(sumoCmd)

MAP_BOUNDARY = (1734.66, 1581.42)
WAREHOUSE_ID = "239796134"


def select_warehouse() -> str:
    global WAREHOUSE_ID

    warehouse_id = WAREHOUSE_ID
    traci.polygon.setColor(warehouse_id, (0, 0, 255))
    return warehouse_id


def get_pickup_id_list() -> list[str]:
    pickup_center_list: list[tuple[float, float]] = [
        (1108.783925, 787.6503665714287),
        (891.3527764504684, 588.4114269530795),
        (908.783925, 987.6503665714287),
        (1108.783925, 441.2402050576532),
        (508.78392499999995, 787.6503665714287),
        (1108.783925, 1134.0605280852042),
        (1428.399167270663, 487.6503665714284),
        (389.16868272933675, 1087.6503665714285),
        (608.7839250000001, 1307.2656088420918),
        (508.7839249999996, 94.83004354387799),
        (1708.783925, 787.6503665714287),
    ]

    pickup_id_list: list[str] = []
    pickup_counter = 0
    for pickup_center in pickup_center_list:
        pickup_id = f"pickup#{pickup_counter}"

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
        pickup_counter += 1

    return pickup_id_list


def get_pickup_capacity_dict(pickup_id_list: list[str]) -> dict[int, int]:
    return {
        # pickup_id: random.randint(5, 15) for pickup_id in range(len(pickup_id_list))
        pickup_id: 2
        for pickup_id in range(len(pickup_id_list))
    }


def assign_pickup(
    destination_id: str, pickup_id_list: list[str], pickup_capacity_dict: dict[int, int]
):
    global package_pickup_matrix
    new_assignment = np.zeros(len(pickup_id_list))

    package_center = shape2centroid(traci.polygon.getShape(destination_id))

    package_pickup_distance = [
        euclidean_distance(
            package_center,
            shape2centroid(traci.polygon.getShape(pickup)),
        )
        for pickup in pickup_id_list
    ]

    selected_pickup = 0
    while package_pickup_distance != [math.inf] * len(pickup_id_list):
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
                package_pickup_distance[selected_pickup] = math.inf
        else:
            new_assignment[selected_pickup] = 1
            new_assignment = new_assignment.reshape(1, -1)
            package_pickup_matrix = np.copy(new_assignment)
            break

    assignVehicle(selected_pickup)


def assignVehicle(pickup: int):
    global \
        vehicle_load, \
        package_vehicle_mapping, \
        pickup_vehicle_mapping, \
        warehouse_edge_id

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


def droneTSP(pickup: str):
    pickup_center = shape2centroid(traci.polygon.getShape(pickup))
    pickup_cost = 0

    coordinates = [pickup_center]
    pickup_no = int(pickup.rpartition("#")[2])

    for num in np.nonzero(package_pickup_matrix.T[pickup_no])[0]:
        package = destination_id_list[num]
        package_center = shape2centroid(traci.polygon.getShape(package))
        coordinates.append(package_center)

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
    nx.draw_networkx_nodes(G, pos=pos, nodelist=list(G.nodes)[1:], node_color="#df34eb")
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
    nx.draw_networkx_edge_labels(G, pos=pos, edge_labels=edge_labels, label_pos=0.25)
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


def main():
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

    for step in range(1200):
        traci.simulationStep()

        if step == 30:
            package_no = 0
            destination_id_list = ["234807099", "239713538", "359039090"]
            for destination_id in destination_id_list:
                traci.polygon.setColor(destination_id, (222, 52, 235))
                assign_pickup(destination_id, pickup_id_list, pickup_capacity_dict)
                package_no += 1
            print(package_pickup_matrix)
            print(package_vehicle_mapping)

        isDropped()

    traci.close()


if __name__ == "__main__":
    main()
