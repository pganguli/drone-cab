from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drone_cab.package import Package
    from drone_cab.warehouse import Warehouse

import traci


def VEHICLE_CAPACITY():
    return 2  # random.randint(5, 15)


class Vehicle:
    def __init__(self, vehicle_id: str, vehicle_capacity: int) -> None:
        self.id = vehicle_id
        self.capacity = vehicle_capacity
        self.carrying_package_set: set[Package] = set()

    def __str__(self) -> str:
        return self.id

    def __repr__(self) -> str:
        return f"Vehicle({self.id}, {self.capacity})"

    def get_road_id(self) -> str:
        return traci.vehicle.getRoadID(self.id)

    def get_route_edge_id_list(self) -> list[str]:
        return traci.route.getEdges(traci.vehicle.getRouteID(self.id))

    def is_visiting_warehouse(self, warehouse: Warehouse) -> bool:
        return warehouse.nearest_edge_id in self.get_route_edge_id_list()

    def get_distance_along_road(self, point: tuple[float, float]) -> float:
        return traci.simulation.getDistance2D(
            *traci.vehicle.getPosition(self.id),
            *point,
            isDriving=True,
        )

    def assign_package(self, package: Package) -> None:
        assert (
            package not in self.carrying_package_set
        ), f"Attempted to assign already-assigned {package=} to vehicle={self}"
        assert (
            len(self.carrying_package_set) < self.capacity
        ), f"Adding of {package=} to pickup={self} exceeds capacity={self.capacity}"
        self.carrying_package_set.add(package)
        traci.vehicle.setColor(self.id, (0, 255, 0))

    def drop_package(self, package: Package) -> None:
        assert (
            package.assigned_pickup is not None
        ), f"Attempted to drop {package=} at unassigned pickup"
        package.assigned_pickup.drop_package(package)
        package.reached_pickup = True

        self.carrying_package_set.remove(package)
        traci.vehicle.setColor(self.id, (255, 255, 0))


def get_vehicle_list() -> list[Vehicle]:
    return [
        Vehicle(vehicle_id, VEHICLE_CAPACITY())
        for vehicle_id in traci.vehicle.getIDList()
    ]
