"""Drone class.

This class implements the drones that transport packages
from the corresponding pickup point to the destination residences.

"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import traci
from scipy.spatial import distance_matrix

from drone_cab.tunables import DRONE_CAPACITY, DRONE_RANGE, DRONE_SPEED
from drone_cab.utils import euclidean_distance, shape2centroid

if TYPE_CHECKING:
    from drone_cab.package import Package

logger = logging.getLogger(__name__)


class Drone:
    """Drones that carry packages from their pickup points to the destination residences.

    Args:
        pickup_id: SUMO ID of the pickup point on which this drone sits.
        drone_capacity (optional): Maximum number of packages that this drone can carry. Defaults to tunable constant.
        drone_speed (optional): Maximum flying speed of this drone. Defaults to tunable constant.
        drone_range (optional): Maximum flying range of this drone. Defaults to tunable constant.

    Attributes:
        pickup_id: SUMO ID of the pickup point on which this drone sits.
        center: 2-D coordinates of the center of the pickup point polyon on which this drone sits.
        capacity: Maximum number of packages that this drone can carry.
        speed: Maximum flying speed of this drone.
        range: Maximum flying range of this drone.
        parked: True if drone is currently sitting idle at the pickup point.
        distance_travelled: Total flying distance covered by the drone so far.
        idle_steps: Number of idle time steps spent by this drone sitting parked at the pickup point.
        carrying_package_set: Set of packages currently being carried by this drone.
    """

    def __init__(
        self,
        pickup_id: str,
        drone_capacity: int = DRONE_CAPACITY(),
        drone_speed: float = DRONE_SPEED(),
        drone_range: float = DRONE_RANGE(),
    ) -> None:
        self.pickup_id = pickup_id
        self.center = shape2centroid(traci.polygon.getShape(self.pickup_id))
        self.capacity = drone_capacity
        self.speed = drone_speed
        self.range = drone_range
        self.parked = True
        self.distance_travelled = 0.0
        self.idle_steps = 0
        self.carrying_package_set: set[Package] = set()

        logger.debug(f"Created {self}")

    def __repr__(self) -> str:
        return f"Drone({self.center}, {self.capacity})"

    def assign_package(self, package: Package) -> None:
        """Assign a package to this drone for being transported to its destination residence.

        Args:
            package: Package object to assign to this drone.
        """
        try:
            assert (
                package not in self.carrying_package_set
            ), f"Attempted to assign already-assigned {package} to {self}"
        except AssertionError:
            logger.warning("AssertionError", exc_info=True)
            return

        self.carrying_package_set.add(package)
        logger.debug(f"Assigned drone of {package}: {self}")

    def define_route(self) -> list:
        G = nx.Graph()
        G.add_node(self.center)
        for package in self.carrying_package_set:
            G.add_node(package.destination_center)
        G.add_weighted_edges_from(
            [
                (node_i, node_j, euclidean_distance(node_i, node_j))
                for node_i in G.nodes
                for node_j in G.nodes
                if node_i != node_j
            ]
        )

        route = nx.algorithms.approximation.christofides(G)
        return list(route)

    def start_route(self) -> None:
        route = self.define_route()
        route_iter = iter(route)
        x, y = next(route_iter)
        assert (
            (x, y) == self.center
        ), f"Drone attempted to take off from {(x, y)} instead of {self.center=}"

    def tsp(self):
        cost = 0
        self.parked = False
        logger.debug(f"{self} started TSP")

        nodes_to_visit = [self.center] + list(
            map(lambda x: x.destination_center, self.carrying_package_set)
        )
        nodes_to_visit = np.array(nodes_to_visit)

        dist_matrix = distance_matrix(nodes_to_visit, nodes_to_visit)
        dist_matrix = np.round(dist_matrix, decimals=2)

        G = nx.from_numpy_array(dist_matrix)
        path = nx.approximation.christofides(G)

        edgelist = []
        for i in range(0, len(path) - 1):
            edgelist.append((path[i], path[i + 1]))
            cost += G.get_edge_data(path[i], path[i + 1])["weight"]
        self.distance_travelled += cost

        logger.debug(f"{self} completed TSP with cost {cost:.2f} m")

        carrying_package_set_copy = self.carrying_package_set.copy()
        for package in carrying_package_set_copy:
            self.drop_package(package)
        carrying_package_set_copy.clear()

        self.parked = True

        # self.display_tsp(G, nodes_to_visit, edgelist)

    def display_tsp(
        self,
        G: nx.Graph,
        nodes_to_visit: list[tuple[float, float]],
        edgelist: list[tuple[int, int]],
    ):
        plt.figure(figsize=(10, 6))
        pos = {}
        for node in G.nodes():
            pos[node] = [nodes_to_visit[node, 0], nodes_to_visit[node, 1]]
        nx.draw_networkx_nodes(G, pos=pos, nodelist=[0], node_color="#ff0000")
        nx.draw_networkx_nodes(
            G, pos=pos, nodelist=list(G.nodes)[1:], node_color="#df34eb"
        )

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
        labels = dict.fromkeys(G.nodes, "package")
        labels[0] = "pickup"
        for i, p in enumerate(pos):
            if i % 2 == 0:
                pos[p][1] -= 7
            else:
                pos[p][1] += 7
        nx.draw_networkx_labels(G, pos=pos, labels=labels, font_size=10)

        plt.show()

    def drop_package(self, package: Package) -> None:
        """Drop off a package being cuurently carried by this drone at its destination residence.

        Args:
            package: Package object to drop off at its destination residence from this drone.
        """
        try:
            assert (
                package not in self.carrying_package_set
            ), f"Attempted to drop {package} not being carried by {self}"
        except AssertionError:
            logger.warning("AssertionError", exc_info=True)
            return

        self.carrying_package_set.remove(package)
        package.mark_delivered()
        logger.debug(f"Delivered {package} by {self}")
