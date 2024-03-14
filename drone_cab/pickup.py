from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drone_cab.package import Package

import traci

from drone_cab.utils import get_lane_list, get_nearest_edge_id

logger = logging.getLogger(__name__)


def PICKUP_CAPACITY():
    return 2  # random.randint(5, 15)


PICKUP_CENTER_LIST = [
    (1108.783925, 787.6503665714287),
    (891.3527764504684, 588.4114269530795),
    (908.783925, 987.6503665714287),
    (1108.783925, 441.2402050576532),
    (508.78392499999995, 787.6503665714287),
    (1108.783925, 1134.0605280852042),
    (1428.399167270663, 487.6503665714284),
    (389.16868272933675, 1087.6503665714285),
    (608.7839250000001, 1307.2656088420918),
    (508.7839249999996, 94.83004354387799),
    (1708.783925, 787.6503665714287),
]


class Pickup:
    def __init__(
        self, pickup_center: tuple[float, float], pickup_capacity: int
    ) -> None:
        self.center = pickup_center
        self.capacity = pickup_capacity
        self.id = f"pickup#{hash(self.center)}"
        self.assigned_package_set: set[Package] = set()
        self.dropped_package_set: set[Package] = set()

        traci.polygon.add(
            polygonID=self.id,
            shape=[
                (self.center[0], self.center[1]),
                (self.center[0] + 15, self.center[1]),
                (self.center[0] + 15, self.center[1] + 15),
                (self.center[0], self.center[1] + 15),
                (self.center[0], self.center[1]),
            ],
            color=[255, 0, 0],
            polygonType="pickup",
            fill=True,
        )

        self.nearest_edge_id = get_nearest_edge_id(self.id, get_lane_list())
        logger.debug(f"Created {self}")

    def __repr__(self) -> str:
        return f"Pickup({self.center}, {self.capacity})"

    def assign_package(self, package: Package) -> None:
        try:
            assert (
                package not in self.assigned_package_set
            ), f"Attempted to assign already-assigned {package} to {self}"
        except AssertionError:
            logger.warning("AssertionError", exc_info=True)
            return

        self.assigned_package_set.add(package)
        logger.debug(f"Assigned pickup of {package}: {self}")

    def drop_package(self, package: Package) -> None:
        try:
            assert (
                package not in self.dropped_package_set
            ), f"Attempted to drop existing {package} at {self}"
        except AssertionError:
            logger.warning("AssertionError", exc_info=True)
            return

        try:
            assert (
                len(self.dropped_package_set) < self.capacity
            ), f"Adding of {package} to {self} exceeds capacity={self.capacity}"
        except AssertionError as e:
            logger.error("AssertionError", exc_info=True)
            raise e

        self.assigned_package_set.remove(package)
        self.dropped_package_set.add(package)
        # print(self, self.dropped_package_set)
        logger.debug(f"Dropped {package} at {self}")

    def pickup_package(self, package: Package) -> None:
        self.dropped_package_set.remove(package)
        logger.debug(f"Picked up {package} from {self}")

    @staticmethod
    def create_pickup_list() -> list[Pickup]:
        return [
            Pickup(pickup_center, PICKUP_CAPACITY())
            for pickup_center in PICKUP_CENTER_LIST
        ]
