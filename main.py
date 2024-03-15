import logging
import os
import sys
from collections import deque

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from scipy.spatial import distance_matrix

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

from drone_cab import Package, Pickup, Vehicle, Warehouse

logger = logging.getLogger(__name__)


def poll_packages(pickup_list: list[Pickup]):
    Vehicle.create_vehicle_list()

    vehicle_list: list[Vehicle] = list(
        filter(lambda vehicle: vehicle.carrying_package_set, Vehicle.vehicle_list)
    )

    for vehicle in vehicle_list:
        for package in vehicle.carrying_package_set.copy():
            try:
                assert (
                    package.assigned_pickup is not None
                ), f"Attempted to query vehicle on {package} with no assigned pickup"
            except AssertionError as e:
                logger.error("AssertionError", exc_info=True)
                raise e

            if vehicle.get_road_id() == package.assigned_pickup.nearest_edge_id:
                vehicle.drop_package(package)

    pickup_list = list(filter(lambda pickup: pickup.dropped_package_set, pickup_list))

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
    logging.basicConfig(
        filename="drone_cab.log",
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        level=logging.DEBUG,
        filemode="w",
    )

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
    logger.info("traci.start()")

    warehouse = Warehouse(Warehouse.create_warehouse_id())
    pickup_list = Pickup.create_pickup_list()
    Vehicle.create_vehicle_list()

    package_queue: deque['Package'] = deque()
    for step in range(200):
        logger.info(f"traci.simulationStep() at {step=}")

        poll_packages(pickup_list)

        if step == 30:
            package_queue.append(Package("234807099"))
            package_queue.append(Package("239713538"))
            package_queue.append(Package("359039090"))

        while package_queue:
            package = package_queue.popleft()
            try:
                pickup = package.assign_package_pickup(pickup_list)
                assert pickup is not None, f"Failed to assign {package} to any pickup"
                vehicle = package.assign_package_vehicle(Vehicle.vehicle_list, warehouse)
                assert vehicle is not None, f"Failed to assign {package} to any vehicle"
            except AssertionError:
                logger.debug("AssertionError", exc_info=True)

        traci.simulationStep()

    traci.close()
    logger.info("traci.close()")
