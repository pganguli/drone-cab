"""Drone class.

This class implements the drones that transport packages
from the corresponding pickup point to the destination residences.

"""

from __future__ import annotations

import logging
import math
from typing import Iterator

import networkx as nx
import traci

from drone_cab.package import Package
from drone_cab.tunables import DRONE_CAPACITY, DRONE_RANGE, DRONE_SPEED
from drone_cab.utils import euclidean_distance, shape2centroid

logger = logging.getLogger(__name__)


class Drone(traci.StepListener):
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
        self.route: Iterator[Drone | Package] | None = None
        self.current_target: Drone | Package = self
        self.current_position: tuple[float, float] = self.center
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

    def christofides_route(self) -> list[Drone | Package]:
        """Find the drone route to follow as per Christofides approximation of TSP.

        Note:
            Start and end coordinates of the TSP route will be the center of the drone's pickup point.
        """
        G = nx.Graph()
        G.add_node(self)
        for package in self.carrying_package_set:
            G.add_node(package)
        G.add_weighted_edges_from(
            [
                (node_i, node_j, euclidean_distance(node_i.center, node_j.center))
                for node_i in G.nodes
                for node_j in G.nodes
                if node_i != node_j
            ]
        )

        return nx.algorithms.approximation.christofides(G)

    def start_tsp(self) -> None:
        """Start the TSP route of the drone to start delivery of carrying_package_set."""
        self.route = iter(self.christofides_route())
        self.current_position = next(self.route).center
        self.current_target = next(self.route)
        self.parked = False

    def end_tsp(self) -> None:
        """End the TSP route of the drone."""
        self.route = None
        self.current_position = self.center
        self.current_target = self
        self.parked = True
        self.idle_steps = 0

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

    def step(self, t: int = 0) -> bool:
        if not self.parked:
            if self.current_position == self.current_target.center:
                logger.debug(
                    f"{self} reached target {self.current_target} at step = {t}"
                )
                if isinstance(self.current_target, Package):
                    self.drop_package(self.current_target)
                try:
                    self.current_target = next(self.route)
                except StopIteration:
                    self.end_tsp()

            dist_left = euclidean_distance(
                self.current_position, self.current_target.center
            )
            distance_step = min(self.speed, dist_left)
            x, y = self.current_position
            target_x, target_y = self.current_target.center
            theta = math.atan2(
                target_y - y,
                target_x - x,
            )
            self.current_position = (
                x + distance_step * math.cos(theta),
                y + distance_step * math.sin(theta),
            )
            self.distance_travelled += distance_step
            logger.debug(
                f"{self} travelled by {distance_step} at step = {t} towards {self.current_target}"
            )

        return True
