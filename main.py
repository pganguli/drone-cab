import logging
import os
import sys
from collections import deque
from logging.handlers import RotatingFileHandler

from drone_cab import Package, Pickup, Vehicle, Warehouse
from drone_cab.assign import assign_package_pickup, assign_package_vehicle

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

logger = logging.getLogger(__name__)


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

    warehouse = Warehouse()
    pickup_list = Pickup.create_pickup_list()
    for pickup in pickup_list:
        traci.addStepListener(pickup.drone)
        traci.addStepListener(pickup)

    package_queue: deque[Package] = deque()

    for step in range(400):
        logger.info(f"Simulation {step=}")

        for vehicle in Vehicle.get_vehicle_list():
            vehicle.step()

        if step == 30:
            for destination_id in ["234807099", "239713538", "359039090"]:
                package_queue.append(Package(destination_id))

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
                package_queue.appendleft(package)
                logger.debug("AssertionError", exc_info=True)

        traci.simulationStep()
        logger.info("traci.simulationStep()")

    traci.close()
    logger.info("traci.close()")


if __name__ == "__main__":
    main()
