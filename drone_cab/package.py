from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drone_cab.pickup import Pickup
    from drone_cab.vehicle import Vehicle
    from drone_cab.warehouse import Warehouse

import traci

from drone_cab.utils import (
    euclidean_distance,
    get_lane_list,
    get_nearest_edge_id,
    shape2centroid,
)

logger = logging.getLogger(__name__)


class Package:
    def __init__(self, destination_id: str) -> None:
        self.id = destination_id
        traci.polygon.setColor(self.id, (222, 52, 235))

        self.center = shape2centroid(traci.polygon.getShape(self.id))
        self.nearest_edge_id = get_nearest_edge_id(self.id, get_lane_list())
        self.assigned_pickup: Pickup | None = None
        self.reached_pickup: bool = False
        self.reached_destination: bool = False
        logger.debug(f"Created {self}")

    def __repr__(self) -> str:
        return f"Package({self.id})"

    def assign_pickup(self, pickup: Pickup) -> None:
        self.assigned_pickup = pickup
        # logger.debug(f"Assigned pickup of {self}: {pickup}")

    def assign_package_pickup(self, pickup_list: list[Pickup]) -> Pickup | None:
        pickup_list = sorted(
            pickup_list,
            key=lambda pickup: euclidean_distance(self.center, pickup.center),
        )

        for pickup in pickup_list:
            if len(pickup.assigned_package_set) < pickup.capacity:
                pickup.assign_package(self)
                self.assign_pickup(pickup)
                logger.debug(f"Assigned pickup of {self}: {pickup}")
                return pickup

        logger.debug(f"Failed to assign pickup of {self} to any pickup")
        return None

    def assign_package_vehicle(
        self, vehicle_list: list[Vehicle], warehouse: Warehouse
    ) -> Vehicle | None:
        try:
            assert (
                self.assigned_pickup is not None
            ), f"Attempted to assign vehicle to {self} with no assigned pickup"
        except AssertionError as e:
            logger.error("AssertionError", exc_info=True)
            raise e

        vehicle_list = sorted(
            list(
                filter(
                    lambda vehicle: vehicle.is_visiting_warehouse(warehouse), vehicle_list
                )
            ),
            key=lambda vehicle: vehicle.get_distance_along_road(warehouse.center),
        )

        for vehicle in vehicle_list:
            if (
                len(vehicle.carrying_package_set) < vehicle.capacity
                and self.assigned_pickup.nearest_edge_id
                in vehicle.get_route_edge_id_list()
            ):
                vehicle.assign_package(self)
                logger.debug(f"Assigned vehicle of {self}: {vehicle}")
                return vehicle

        logger.debug(f"Failed to assign pickup of {self} to any vehicle")
        return None
