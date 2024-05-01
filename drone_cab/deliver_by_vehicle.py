from drone_cab.package import Package
from drone_cab.warehouse import Warehouse

import traci


def deliver_by_vehicle(
    packages: list[Package], warehouse: Warehouse
) -> None:  # works only if all packages have same vehicle
    total_dist = 0
    current_center = warehouse.center
    print(current_center)
    while packages:
        package = min(
            packages,
            key=lambda package: traci.simulation.getDistance2D(
                x1=current_center[0],
                y1=current_center[1],
                x2=package.center[0],
                y2=package.center[1],
                isDriving=True,
            ),
        )
        total_dist += traci.simulation.getDistance2D(
            x1=current_center[0],
            y1=current_center[1],
            x2=package.center[0],
            y2=package.center[1],
            isDriving=True,
        )
        print(f"{package} delivered by {package.assigned_vehicle}")
        current_center = package.center
        packages.remove(package)
    print(f"Total distance traveled by cab: {total_dist}")
