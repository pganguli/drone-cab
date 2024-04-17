from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

from matplotlib.patches import Wedge
from matplotlib.pylab import ArrayLike

from drone_cab.utils import euclidean_distance

if TYPE_CHECKING:
    from drone_cab.package import Package

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from scipy.spatial import distance_matrix

logger = logging.getLogger(__name__)


def DRONE_CAPACITY() -> int:
    return 2  # random.randint(5, 15)


def DRONE_TIMEOUT() -> int:
    return 30


class Drone:
    def __init__(self, drone_center: tuple[float, float], drone_capacity: int) -> None:
        self.center = drone_center
        self.capacity = drone_capacity
        self.speed = 2
        self.range = 200
        self.id = f"drone#{hash(self.center)}"
        self.parked = True
        self.cost = 0
        self.timer = 0
        self.carrying_package_set: set[Package] = set()
        # self.delivered_package_set: set[Package] = set()
        self.sector = None

        logger.debug(f"Created {self}")

    def __repr__(self) -> str:
        return f"Drone({self.center}, {self.capacity})"

    def assign_package(self, package: Package) -> None:
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
            G.add_node(package.center)
        G.add_weighted_edges_from(
            [
                (node_i, node_j, euclidean_distance(node_i, node_j))
                for node_i in G.nodes
                for node_j in G.nodes
                if node_i != node_j
            ]
        )

        route = nx.algorithms.approximation.christofides(G)
        return(list(route))
        # self.start_route()
        path_iter = iter(path)

    def start_route(self) -> None:
        route = self.define_route()
        route_iter = iter(route)
        x, y = next(route_iter)
        assert (
            (x, y) == self.center
        ), f"Drone attempted to take off from {(x, y)} instead of {self.center=}"
        target_x, target_y = x, y
        history_x, history_y = [], []
        distance_travelled = 0



    def tsp(self):
        cost = 0
        self.parked = False
        logger.debug(f"{self} started TSP")

        nodes_to_visit = [self.center] + list(
            map(lambda x: x.center, self.carrying_package_set)
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
        self.cost += cost

        logger.debug(f"{self} completed TSP with cost {cost:.2f} m")

        carrying_package_set_copy = self.carrying_package_set.copy()
        for package in carrying_package_set_copy:
            self.deliver_package(package)
        carrying_package_set_copy.clear()

        self.parked = True

        # self.display_tsp(G, nodes_to_visit, edgelist)

    def display_tsp(
        self,
        G: nx.Graph,
        nodes_to_visit: list[(float, float)],
        edgelist: list[(int, int)],
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

    def deliver_package(self, package: Package) -> None:
        try:
            assert (
                package not in self.delivered_package_set
            ), f"Attempted to drop existing {package} at {self}"
        except AssertionError:
            logger.warning("AssertionError", exc_info=True)
            return

        self.carrying_package_set.remove(package)
        self.delivered_package_set.add(package)
        logger.debug(f"Delivered {package} by {self}")

        package.package_delivered()

    @staticmethod
    def create_drone(drone_center) -> Drone:
        return Drone(drone_center, DRONE_CAPACITY())
