import logging
import os
import sys
from collections import deque

from drone_cab import Package, Pickup, Vehicle, Warehouse
from drone_cab.assign import assign_package_pickup, assign_package_vehicle
from drone_cab.tunables import DRONE_MAX_IDLE_STEPS, PICKUP_CENTER_LIST

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

logger = logging.getLogger(__name__)


def poll_packages(non_empty_pickup_list: list[Pickup]):
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

    non_empty_pickup_list = list(filter(lambda pickup: pickup.received_package_set, non_empty_pickup_list))
    for pickup in non_empty_pickup_list:
        if pickup.drone.idle_steps > DRONE_MAX_IDLE_STEPS() and pickup.drone.parked:
            pickup.init_tsp()
        else:
            pickup.drone.idle_steps += 1


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

    warehouse = Warehouse()
    pickup_list = [Pickup(pickup_center) for pickup_center in PICKUP_CENTER_LIST()]
    for pickup in pickup_list:
        traci.addStepListener(pickup.drone)
    Vehicle.create_vehicle_list()

    package_queue: deque["Package"] = deque()

    for step in range(200):
        logger.info(f"Simulation {step=}")

        poll_packages(pickup_list)

        if step == 30:
            for destination_id in ["234807099", "239713538"]:#, "359039090"]:
                package_queue.append(
                    Package(destination_id)
                )

        while package_queue:
            package = package_queue.popleft()
            try:
                pickup = assign_package_pickup(package, pickup_list)
                assert pickup is not None, f"Failed to assign {package} to any pickup"
                vehicle = assign_package_vehicle(
                    package, Vehicle.vehicle_list, warehouse
                )
                assert vehicle is not None, f"Failed to assign {package} to any vehicle"
            except AssertionError:
                package_queue.appendleft(package)
                logger.debug("AssertionError", exc_info=True)

        traci.simulationStep()
        logger.info("traci.simulationStep()")

    traci.close()
    logger.info("traci.close()")
