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

    Attributes:
        destination_id: SUMO ID of destination residence.
        center: 2-D coordinates of the centroid of destination residence's polygon.
        assigned_pickup: Pikcup object that this package has been assigned to.
        reached_pickup: True if package has reached its assigned pickup point.
        reached_destination: True if package has reached its destination residence.
    """

    def __init__(self, destination_id: str) -> None:
        self.destination_id = destination_id
        traci.polygon.setColor(self.destination_id, (222, 52, 235))
        self.center = shape2centroid(traci.polygon.getShape(self.destination_id))
        self.assigned_pickup: Pickup | None = None
        self.reached_pickup: bool = False
        self.reached_destination: bool = False
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

    def mark_delivered(self):
        """Mark package as delivered to destination residence."""
        self.reached_destination = True
        logger.debug(f"{self} reached destination")
