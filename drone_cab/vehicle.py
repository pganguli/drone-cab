"""Vehicle (or cab) class.

This class implements the vehicles / cabs that transport
packages from the central warehouse to the drone pickup points.

"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from drone_cab.tunables import VEHICLE_CAPACITY

if TYPE_CHECKING:
    from drone_cab.package import Package
    from drone_cab.warehouse import Warehouse

import traci

logger = logging.getLogger(__name__)


class Vehicle(traci.StepListener):
    """Vehicles that carry packages from the warehouse to pickup points.

    Args:
        vehicle_id: SUMO ID of vehicle.
        vehicle_capacity (optional): Maximum number of packages that this vehicle can carry. Defaults to tunable constant.
    """

    vehicle_list: list[Vehicle] = []  #: List of all vehicle objects.

    def __init__(
        self, vehicle_id: str, vehicle_capacity: int = VEHICLE_CAPACITY()
    ) -> None:
        self.id: str = vehicle_id  #: SUMO IF of vehicle
        self.capacity: int = (
            vehicle_capacity  #: Maximum number of packages that this vehicle can carry
        )
        self.carrying_package_set: set[Package] = (
            set()
        )  #: Set of packages being carried by this vehicle
        if self not in Vehicle.vehicle_list:
            Vehicle.vehicle_list.append(self)
            logger.debug(f"Created {self}")

    def __repr__(self) -> str:
        return f"Vehicle({self.id}, {self.capacity})"

    def __eq__(self, other) -> bool:
        return self.id == other.id

    def get_road_id(self) -> str:
        """Get SUMO ID of road that vehicle is currently on.

        Returns:
            SUMO ID of road that vehicle is currently on.
        """
        return traci.vehicle.getRoadID(self.id)

    def get_route_edge_id_list(self) -> list[str]:
        """Get list of SUMO IDs of edges that comprise vehicle's route.

        Returns:
            List of SUMO IDs of edges that comrpise vehcile's route.
        """
        return traci.route.getEdges(traci.vehicle.getRouteID(self.id))

    def is_visiting_warehouse(self, warehouse: Warehouse) -> bool:
        """Whether vehicle is yet to reach warehouse.

        Args:
            warehouse: SUMO ID of warehouse to visit for picking up packages.

        Returns:
            True if vehicle has not yet reached edge closest to warehouse.
        """
        return warehouse.nearest_edge_id in self.get_route_edge_id_list()

    def get_distance_along_road(self, target: tuple[float, float]) -> float:
        """Get distance along the road network to the specified target.

        Args:
            target: 2-D coordinates of target to measure distance to.

        Returns:
            Distance along the road network to the specified target.
        """
        x1, y1 = traci.vehicle.getPosition(self.id)
        x2, y2 = target
        return traci.simulation.getDistance2D(
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
            isDriving=True,
        )

    def add_package(self, package: Package) -> None:
        """Add a package to the vehicle for transporting to a pickup point.

        Args:
            package: Package object to be added to vehicle.

        Raises:
            AssertionError: If addition of package would exceed capacity of vehicle.
        """
        try:
            assert (
                package not in self.carrying_package_set
            ), f"Attempted to assign already-assigned {package} to {self}"
        except AssertionError:
            logger.warning("AssertionError", exc_info=True)
            return

        try:
            assert (
                len(self.carrying_package_set) < self.capacity
            ), f"Adding of {package} to {self} exceeds capacity={self.capacity}"
        except AssertionError as e:
            logger.error("AssertionError", exc_info=True)
            raise e

        self.carrying_package_set.add(package)
        traci.vehicle.setColor(self.id, (0, 255, 0))
        logger.debug(f"Assigned vehicle of {package} to {self}")

    def drop_package(self, package: Package) -> None:
        """Drop a package off onto its assigned pickup point.

        Args:
            package: Package object to drop off.

        Raises:
            AssertionError: If package has no assigned pickup point.
        """
        try:
            assert (
                package.assigned_pickup is not None
            ), f"Attempted to drop {package} at unassigned pickup"
        except AssertionError as e:
            logger.error("AssertionError", exc_info=True)
            raise e

        self.carrying_package_set.remove(package)
        traci.vehicle.setColor(self.id, (255, 255, 0))
        logger.debug(f"Dropped {package} by {self}")

        package.assigned_pickup.add_package(package)
        package.reached_pickup = True

    def check_reached_pickup(self):
        packages_to_drop: list[Package] = []

        for package in self.carrying_package_set:
            try:
                assert (
                    package.assigned_pickup is not None
                ), f"Attempted to query vehicle on {package} with no assigned pickup"
            except AssertionError as e:
                logger.error("AssertionError", exc_info=True)
                raise e

            if self.get_road_id() == package.assigned_pickup.nearest_edge_id:
                packages_to_drop.append(package)

        for package in packages_to_drop:
            self.drop_package(package)

    def step(self, t: int = 0):
        t += 0

        if self.carrying_package_set:
            self.check_reached_pickup()

        return True

    @staticmethod
    def create_vehicle_list() -> None:
        """Populate vehicle object list with SUMO IDs from current simulation."""
        for vehicle_id in traci.vehicle.getIDList():
            Vehicle(vehicle_id)

    @staticmethod
    def get_vehicle_list() -> list[Vehicle]:
        """Return vehicle object list with SUMO IDs from current simulation.

        Returns:
            List of vehicle objects.
        """
        Vehicle.create_vehicle_list()
        return Vehicle.vehicle_list
