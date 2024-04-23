"""Pickup class.

This class implements the pickup points that store packages
temporarily after being dropped off by a vehicle, until a drone
picks them up for completion of their delivery.

"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

import traci
from matplotlib.patches import Wedge

from drone_cab.drone import Drone
from drone_cab.tunables import PICKUP_CAPACITY
from drone_cab.utils import euclidean_distance, get_nearest_edge_id

if TYPE_CHECKING:
    from drone_cab.package import Package

logger = logging.getLogger(__name__)


class Pickup:
    """Pickup points stroe packages after being dropped off by vehicles, until a drone picks them up.

    Args:
        pickup_center: 2-D coordinates of the center of the pickup point polyon.
        pickup_capacity (optional): Maximum number of packages that this pikcup point can store. Defaults to tunable constant.

    Attributes:
        center: 2-D coordinates of the center of the pickup point polyon.
        capacity: Maximum number of packages that this pikcup point can store.
        id: SUMO ID of the pickup point.
        capacity: Maximum number of packages that this drone can carry.
        drone: Drone object that services (sits at) this pickup point.
        assigned_package_set: Packages expected to be delivered to this pickup point by vehicles.
        received_package_set: Packages currently being stored at this pickup point.
        nearest_edge_id: SUMO ID of the raod edge closest to this pickup point polygon.
    """

    def __init__(
        self,
        pickup_center: tuple[float, float],
        pickup_capacity: int = PICKUP_CAPACITY(),
    ) -> None:
        self.center = pickup_center
        self.capacity = pickup_capacity
        self.id = f"pickup#{hash(self.center)}"
        self.assigned_package_set: set[Package] = set()
        self.received_package_set: set[Package] = set()

        traci.polygon.add(
            polygonID=self.id,
            shape=[
                (self.center[0], self.center[1]),
                (self.center[0] + 15, self.center[1]),
                (self.center[0] + 15, self.center[1] + 15),
                (self.center[0], self.center[1] + 15),
                (self.center[0], self.center[1]),
            ],
            color=(255, 0, 0),
            polygonType="pickup",
            fill=True,
        )

        self.drone = Drone(self.id)
        self.nearest_edge_id = get_nearest_edge_id(self.id)

        logger.debug(f"Created {self}")

    def __repr__(self) -> str:
        return f"Pickup({self.center}, {self.capacity})"

    def assign_package(self, package: Package) -> None:
        """Assign a package to this pickup point for storage after being dropped off by a vehicle.

        Args:
            package: Package object to assign to this pickup point.
        """
        try:
            assert (
                package not in self.assigned_package_set
            ), f"Attempted to assign already-assigned {package} to {self}"
        except AssertionError:
            logger.warning("AssertionError", exc_info=True)
            return

        self.assigned_package_set.add(package)
        logger.debug(f"Assigned pickup of {package} to {self}")

    def add_package(self, package: Package) -> None:
        """Add a package to this pickup point's storage.

        Args:
            package: Package object to add to this pickup point's storage.

        Raises:
            AssertionError: If addition of package would exceed capacity of this pickup point.
        """
        try:
            assert (
                package not in self.received_package_set
            ), f"Attempted to drop existing {package} at {self}"
        except AssertionError:
            logger.warning("AssertionError", exc_info=True)
            return

        try:
            assert (
                len(self.received_package_set) < self.capacity
            ), f"Adding of {package} to {self} exceeds capacity={self.capacity}"
        except AssertionError as e:
            logger.error("AssertionError", exc_info=True)
            raise e

        self.assigned_package_set.remove(package)
        self.received_package_set.add(package)
        logger.debug(f"Dropped {package} at {self}")

    def init_tsp(self):
        logger.debug(f"Starting TSP of {self.drone}")

        farthest_package = max(
            self.received_package_set,
            key=lambda package: euclidean_distance(self.center, package.center),
        )
        radius = euclidean_distance(self.center, farthest_package.center)
        theta = self.drone.range / radius - 2

        farthest_residence_angle = math.atan2(
            farthest_package.center[1] - self.center[1],
            farthest_package.center[0] - self.center[0],
        )

        self.sector = Wedge(
            center=self.center,
            r=radius,
            theta1=math.degrees(farthest_residence_angle - theta / 2),
            theta2=math.degrees(farthest_residence_angle + theta / 2),
        )

        delivery_packages = set(
            [
                package
                for package in self.received_package_set
                if self.sector.contains_point(package.center)
            ]
        )

        while len(delivery_packages) > self.drone.capacity:
            package_to_remove = min(
                delivery_packages,
                key=lambda package: euclidean_distance(self.center, package.center)
            )
            delivery_packages.remove(
                package_to_remove
            )
            logger.debug(f"Removed {package_to_remove} from {self.drone} due to capacity being at {len(delivery_packages)}")

        for package in delivery_packages:
            self.received_package_set.remove(package)
            logger.debug(f"Added {package} to {self.drone} from {self}")
            self.drone.assign_package(package)

        try:
            assert (
                self.drone.carrying_package_set
            ), f"{self.drone} is carrying no packages, yet TSP initiated."
        except AssertionError as e:
            logger.error("AssertionError", exc_info=True)
            raise e

        self.drone.start_tsp()
