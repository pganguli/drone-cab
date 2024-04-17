import logging
import os
import sys
from collections import deque

from drone_cab.drone import DRONE_TIMEOUT


if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

from drone_cab import Package, Pickup, Vehicle, Warehouse

logger = logging.getLogger(__name__)


def poll_packages(pickup_list: list[Pickup]):
    Vehicle.create_vehicle_list()

    vehicle_list: list[Vehicle] = list(
        filter(lambda vehicle: vehicle.carrying_package_set, Vehicle.vehicle_list)
    )

    for vehicle in vehicle_list:
        for package in vehicle.carrying_package_set.copy():
            try:
                assert (
                    package.assigned_pickup is not None
                ), f"Attempted to query vehicle on {package} with no assigned pickup"
            except AssertionError as e:
                logger.error("AssertionError", exc_info=True)
                raise e

            if vehicle.get_road_id() == package.assigned_pickup.nearest_edge_id:
                vehicle.drop_package(package)

    pickup_list = list(filter(lambda pickup: pickup.received_package_set, pickup_list))
    for pickup in pickup_list:
        if pickup.drone.timer > DRONE_TIMEOUT() and pickup.drone.parked:
            pickup.prepare_drone()
        else:
            pickup.drone.timer += 1
    #     idle_time = max(
    #         map(
    #             lambda package: traci.simulation.getTime()
    #             - package.reached_pickup_time,
    #             pickup.received_package_set,
    #         )
    #     )
    #     if idle_time > PACKAGE_TIMEOUT():
    #         logger.warning(f"Package idle time exceeded at {pickup}")
    #         pickup.prepare_drone()


if __name__ == "__main__":
    logging.basicConfig(
        filename="drone_cab.log",
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        level=logging.DEBUG,
        filemode="w",
    )

    traci.start(
        [
            "sumo",
            "-c",
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "data",
                "config.sumocfg",
            ),
            "-d",
            "150",
        ]
    )
    logger.info("traci.start()")

    warehouse = Warehouse(Warehouse.create_warehouse_id())
    pickup_list = Pickup.create_pickup_list()
    Vehicle.create_vehicle_list()

    package_queue: deque["Package"] = deque()
    for step in range(200):
        logger.info(f"traci.simulationStep() at {step=}")

        poll_packages(pickup_list)

        if step == 30:
            package_queue.append(Package("234807099"))
            package_queue.append(Package("239713538"))
            package_queue.append(Package("359039090"))

        while package_queue:
            package = package_queue.popleft()
            try:
                pickup = package.assign_package_pickup(pickup_list)
                assert pickup is not None, f"Failed to assign {package} to any pickup"
                vehicle = package.assign_package_vehicle(
                    Vehicle.vehicle_list, warehouse
                )
                assert vehicle is not None, f"Failed to assign {package} to any vehicle"
            except AssertionError:
                package_queue.appendleft(package)
                logger.debug("AssertionError", exc_info=True)

        traci.simulationStep()

    traci.close()
    logger.info("traci.close()")
