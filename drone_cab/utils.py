"""Helper utilities.

Collection of various helper functions and utilities.

"""

import logging
import math
from operator import sub

import traci

logger = logging.getLogger(__name__)


def shape2centroid(shape: list[tuple[float, float]]) -> tuple[float, float]:
    """Calculate the centroid of given polygon shape.

    Args:
        shape: List of 2-D coordinates of vertices of given polygon.

    Returns:
        2-D coordinates of centroid of given polygon shape.

    Raises:
        AssertionError: If given coordinates are not 2-D.
    """
    centroid = tuple(
        map(lambda dim_sum: dim_sum / len(shape), [sum(x) for x in zip(*shape)])
    )

    try:
        assert (
            len(centroid) == 2
        ), f"Expected 2-D centroid for {shape=}, got {len(centroid)}-D"
    except AssertionError as e:
        logger.error("AssertionError", exc_info=True)
        raise e

    return centroid


def euclidean_distance(
    point_a: tuple[float, float], point_b: tuple[float, float]
) -> float:
    """Calculate the Euclidean distance between two given points.

    Args:
        point_a: 2-D coordinates of first point.
        point_b: 2-D coordinates of second point.

    Returns:
        Euclidean distance between given two points.

    Raises:
        AssertionError: If given coordinates are not 2-D.
    """
    try:
        assert len(point_a) == 2, f"Expected 2-D point={point_a}, got {len(point_a)}-D"
    except AssertionError as e:
        logger.error("AssertionError", exc_info=True)
        raise e
    return math.hypot(*map(sub, point_a, point_b))


def get_lane_list() -> list[str]:
    """Get list of all SUMO IDs of lanes in current simulation.

    Returns:
        List of SUMO IDs of all lanes in current simulation.
    """
    return list(filter(lambda id: not id.startswith(":"), traci.lane.getIDList()))


def get_nearest_edge_id(polygon_id: str) -> str:
    """Finds the nearest road edge to a given polygon.

    Args:
        polygon_id: SUMO ID of given polygon.

    Returns:
        SUMO ID of road edge that is closest to given polygon.
    """
    nearest_lane_id = min(
        get_lane_list(),
        key=lambda lane_id: euclidean_distance(
            shape2centroid(traci.polygon.getShape(polygon_id)),
            shape2centroid(traci.lane.getShape(lane_id)),
        ),
    )

    nearest_edge_id = traci.lane.getEdgeID(nearest_lane_id)

    logger.debug(f"Found {nearest_edge_id=} of {polygon_id=}")
    return nearest_edge_id


def get_building_id_list() -> list[str]:
    """Get list of all SUMO IDs of polygons of type 'building' in current simulation.

    Returns:
        List of SUMO IDs of all polygons of type 'building' in current simulation.
    """
    return list(
        filter(
            lambda polygon_id: traci.polygon.getType(polygon_id) == "building",
            traci.polygon.getIDList(),
        )
    )
