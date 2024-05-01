"""Assignment utilities.

Collection of functions that assign various entities to other entities.

"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from drone_cab.utils import euclidean_distance

if TYPE_CHECKING:
    from drone_cab.package import Package
    from drone_cab.pickup import Pickup
    from drone_cab.vehicle import Vehicle
    from drone_cab.warehouse import Warehouse

logger = logging.getLogger(__name__)


def assign_package_pickup(package: Package, pickup_list: list[Pickup]) -> Pickup | None:
    """Attempt to assign a pickup point to the given package.

    Args:
        package: Package obejct to attenpt assignment of pickup point to.
        pickup_list: List of pickup point objects to choose the pickup point from.

    Returns:
        Assigned pickup object if successful, else None.
    """
    try:
        assert (
            package.assigned_pickup is None
        ), f"Attempted to assign pickup to {package} with already assigned pickup"
    except AssertionError:
        logger.error("AssertionError", exc_info=True)
        return

    pickup_list = sorted(
        pickup_list,
        key=lambda pickup: euclidean_distance(package.center, pickup.center),
    )

    for pickup in pickup_list:
        if len(pickup.assigned_package_set) < pickup.capacity:
            pickup.assign_package(package)
            package.set_pickup(pickup)
            return pickup

    logger.debug(f"Failed to assign pickup of {package} to any pickup")
    return None


def assign_package_vehicle(
    package: Package, vehicle_list: list[Vehicle], warehouse: Warehouse
) -> Vehicle | None:
    """Attempt to assign a vehicle to the given package.

    Args:
        package: Package obejct to attenpt assignment of vehicle to.
        vehicle_list: List of vehicle objects to choose the vehicle from.
        warehouse: Warehouse object from where the vehicle will pick up the package.

    Returns:
        Assigned vehicle object if successful, else None.

    Raises:
        AssertionError: If given package does not have an assigned pickup point.
    """
    try:
        assert (
            package.assigned_pickup is not None
        ), f"Attempted to assign vehicle to {package} with no assigned pickup"
    except AssertionError as e:
        logger.error("AssertionError", exc_info=True)
        raise e

    vehicle_distance = sorted(
        [
            (vehicle, vehicle.get_distance_along_road(warehouse.center))
            for vehicle in filter(
                lambda vehicle: vehicle.is_visiting_warehouse(warehouse), vehicle_list
            )
        ],
        key=lambda vehicle_distance: vehicle_distance[1],
    )

    for vehicle, distance in vehicle_distance:
        if (
            len(vehicle.carrying_package_set) < vehicle.capacity
            and package.assigned_pickup.nearest_edge_id
            in vehicle.get_route_edge_id_list()
        ):
            vehicle.add_package(package)
            package.set_vehicle(vehicle)
            distance_to_pickup = vehicle.get_distance_along_road(
                package.assigned_pickup.center
            ) - distance
            package.distance_vehicle += distance_to_pickup
            logger.debug(
                f"Assigned vehicle of {package} to {vehicle} with {distance=} from warehouse and {distance_to_pickup=}"
            )
            return vehicle

    logger.debug(f"Failed to assign {package} to any vehicle")
    return None
