import os
import sys

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from scipy.spatial import distance_matrix

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

from drone_cab import Package, Pickup, Vehicle, Warehouse
from drone_cab.package import assign_package_pickup, assign_package_vehicle
from drone_cab.pickup import get_pickup_list
from drone_cab.vehicle import get_vehicle_list
from drone_cab.warehouse import get_warehouse_id


def poll_packages(pickup_list: list[Pickup], vehicle_list: list[Vehicle]):
    vehicle_list = list(
        filter(lambda vehicle: len(vehicle.carrying_package_set) > 0, vehicle_list)
    )

    for vehicle in vehicle_list:
        for package in vehicle.carrying_package_set:
            assert (
                package.assigned_pickup is not None
            ), f"Attempted to query vehicle on {package=} with no assigned pickup"

            if vehicle.get_road_id() == package.assigned_pickup.nearest_edge_id:
                vehicle.drop_package(package)

    pickup_list = list(
        filter(lambda pickup: len(pickup.assigned_package_set) == 0, pickup_list)
    )

    for pickup in pickup_list:
        droneTSP(pickup)


def droneTSP(pickup: Pickup):
    pickup_cost = 0

    nodes_to_visit = [pickup.center]

    for package in pickup.dropped_package_set:
        nodes_to_visit.append(package.center)

    nodes_to_visit = np.array(nodes_to_visit)
    dist_matrix = distance_matrix(nodes_to_visit, nodes_to_visit)
    dist_matrix = np.round(dist_matrix, decimals=2)
    plt.figure(figsize=(10, 6))
    G = nx.from_numpy_array(dist_matrix)
    path = nx.approximation.christofides(G)
    pos = {}
    for node in G.nodes():
        pos[node] = [nodes_to_visit[node, 0], nodes_to_visit[node, 1]]
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


if __name__ == "__main__":
    traci.start(
        [
            "sumo",
            "-c",
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "data",
                "config.sumocfg",
            ),
            "-d",
            "150",
        ]
    )

    warehouse = Warehouse(get_warehouse_id())
    pickup_list = get_pickup_list()
    vehicle_list = get_vehicle_list()

    for step in range(1200):
        traci.simulationStep()

        if step == 30:
            destination_id_list = ["234807099", "239713538", "359039090"]
            for destination_id in destination_id_list:
                package = Package(destination_id)
                pickup = assign_package_pickup(package, pickup_list)
                assert pickup is not None, f"Failed to assign {package=} to any pickup"
                vehicle = assign_package_vehicle(package, vehicle_list, warehouse)

        poll_packages(pickup_list, vehicle_list)

    traci.close()
