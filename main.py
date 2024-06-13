#!/usr/bin/env python3

import logging
import os
import sys
from collections import deque
from logging.handlers import RotatingFileHandler

import numpy as np

from drone_cab import Package, Pickup, Vehicle, Warehouse
from drone_cab.assign import assign_package_pickup, assign_package_vehicle
from drone_cab.tunables import AVG_STEPS_PER_PACKAGE_REQUEST
from drone_cab.utils import get_building_id_list

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

logger = logging.getLogger(__name__)
rng = np.random.default_rng(seed=42)


def next_package_request_step(current_step: int = 0) -> int:
    return current_step + rng.poisson(AVG_STEPS_PER_PACKAGE_REQUEST())


def main():
    logging.basicConfig(
        handlers=[
            RotatingFileHandler(
                "drone_cab.log", mode="w", maxBytes=1024 * 1024 * 256, backupCount=1
            )
        ],
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        level=logging.DEBUG,
    )

    traci.start(
        [
            "sumo-gui",
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

    building_ids = get_building_id_list()
    warehouse = Warehouse()
    pickup_list = Pickup.create_pickup_list()
    for pickup in pickup_list:
        traci.addStepListener(pickup.drone)
        traci.addStepListener(pickup)

    package_queue: deque[Package] = deque()

    step = 0
    package_counter = 0
    package_request_step = next_package_request_step(step)
    while package_counter < 1000:
        logger.info(f"Simulation {step=}")

        for vehicle in Vehicle.get_vehicle_list():
            vehicle.step()

        if step == package_request_step:
            destination_id = rng.choice(building_ids, replace=False)
            package = Package(destination_id)
            package_queue.append(package)
            package_request_step = next_package_request_step(step)
            package_counter += 1
            logger.debug(f"Generated {package=} request at {step=}")

        unassigned_packages: deque[Package] = deque()
        while package_queue:
            package = package_queue.popleft()
            try:
                pickup = assign_package_pickup(package, pickup_list)
                assert pickup is not None, f"Failed to assign {package} to any pickup"
                vehicle = assign_package_vehicle(
                    package, Vehicle.get_vehicle_list(), warehouse
                )
                assert vehicle is not None, f"Failed to assign {package} to any vehicle"
            except AssertionError:
                unassigned_packages.append(package)
                logger.debug("AssertionError", exc_info=True)

        while unassigned_packages:
            package_queue.append(unassigned_packages.popleft())

        traci.simulationStep()
        logger.info("traci.simulationStep()")
        step += 1

    traci.close()
    logger.info("traci.close()")


if __name__ == "__main__":
    main()
