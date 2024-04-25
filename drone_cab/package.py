"""Package (or parcel) class.

This class implements the packages / parcels that need to be
delivered to the residences that have requested them from the warehouse.

"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drone_cab.pickup import Pickup

import traci

from drone_cab.utils import shape2centroid

logger = logging.getLogger(__name__)


class Package:
    """Packages (or parcels) that get transported by cabs (vehicles) and drones.

    Args:
        destination_id: SUMO ID of residence where this package needs to be delivered.

    Note:
        self.center has to be named this way to be compatiple with drone.center for christofides_route()
    """

    def __init__(self, destination_id: str) -> None:
        self.destination_id: str = destination_id  #: SUMO ID of destination residence.
        traci.polygon.setColor(self.destination_id, (222, 52, 235))
        self.center: tuple[float, float] = shape2centroid(
            traci.polygon.getShape(self.destination_id)
        )  #: 2-D coordinates of the centroid of destination residence's polygon.
        self.assigned_pickup: Pickup | None = (
            None  #: Pickup object that this package has been assigned to.
        )
        self.reached_pickup: bool = (
            False  #: True if package has reached its assigned pickup point.
        )
        self.reached_destination: bool = (
            False  #: True if package has reached its destination residence.
        )
        self.distance_drone: float = 0.0
        self.distance_vehicle: float = 0.0
        logger.debug(f"Created {self} with center {self.center}")

    def __repr__(self) -> str:
        return f"Package({self.destination_id})"

    def set_pickup(self, pickup: Pickup) -> None:
        """Set assigned pickup point for this package.

        Args:
            pickup: Pickup object that this package has been assigned to.
        """
        self.assigned_pickup = pickup
        logger.debug(f"Assigned pickup of {self} to {pickup}")

    def mark_delivered(self, distance_drone: float):
        """Mark package as delivered to destination residence."""
        self.reached_destination = True
        self.distance_drone = distance_drone
        logger.debug(f"{self} reached destination")
        print(f"{self} delivered with {self.distance_vehicle=} and {self.distance_drone=}")
