"""Warehouse class.

This class implements the central warehouse from where
all central packages are distributed for delivery.

"""

import logging

import traci

from drone_cab.tunables import WAREHOUSE_ID
from drone_cab.utils import get_nearest_edge_id, shape2centroid

logger = logging.getLogger(__name__)


class Warehouse:
    """Central warehouse from where all packages are distributed for delivery.

    Args:
        warehouse_id (optional): SUMO ID of warehouse. Defaults to tunable constant.
    """

    def __init__(self, warehouse_id: str = WAREHOUSE_ID()) -> None:
        self.id: str = warehouse_id  #: SUMO ID of this warehouse.
        traci.polygon.setColor(self.id, (0, 0, 255))
        self.center: tuple[float, float] = shape2centroid(
            traci.polygon.getShape(self.id)
        )  #: 2-D coordinates of the centroid of this warehouse's polygon.
        self.nearest_edge_id = get_nearest_edge_id(
            self.id
        )  #: SUMO ID of road edge that is closest to this warehouse.
        logger.debug(f"Created {self}")

    def __repr__(self) -> str:
        return f"Warehouse({self.id})"
