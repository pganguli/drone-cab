"""Tunable parameters.

Collection of various tunable constants and functions.

"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


def WAREHOUSE_ID() -> str:
    """Get hard-coded SUMO ID of chosen warehouse.

    Returns:
        SUMO ID of warehouse.
    """
    return "239796134"


def PICKUP_CENTER_LIST() -> list[tuple[float, float]]:
    """Get hard-coded list of 2-D coordinates of centers of chosen pickup points.

    Returns:
        List of 2-D coordinates of pikcup points.
    """
    return [
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


def VEHICLE_CAPACITY():
    """Generate capacities for vehicles.

    Returns:
        A (possibly random) capacity for a vehicle.
    """
    return 2  # random.randint(5, 15)


def DRONE_CAPACITY() -> int:
    """Generate capacities for drones.

    Returns:
        A (possibly random) capacity for a drone.
    """
    return 2  # random.randint(5, 15)


def DRONE_MAX_IDLE_STEPS() -> int:
    """Get hard-coded maximum number of allowed idle steps for drones.

    Returns:
        Maximum number of allowed idle steps for drones.
    """
    return 30


def DRONE_RANGE() -> float:
    """Generate flying range for drones.

    Returns:
        A (possibly random) flying range for a drone.
    """
    return 500.0


def DRONE_SPEED() -> float:
    """Generate flying speed for drones.

    Returns:
        A (possibly random) flying speed for a drone.
    """
    return 2.0


def PICKUP_CAPACITY() -> int:
    """Generate capacities for pickups.

    Returns:
        A (possibly random) capacity for a pickup.
    """
    return 2  # random.randint(5, 15)


def AVG_STEPS_PER_PACKAGE_REQUEST() -> int:
    """Generate capacities for pickups.

    Returns:
        A (possibly random) capacity for a pickup.
    """
    return 2  # random.randint(5, 15)
