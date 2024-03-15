import logging
import math
from operator import sub

import traci

logger = logging.getLogger(__name__)


def shape2centroid(shape: list[tuple[float, float]]) -> tuple[float, float]:
    centroid = tuple(
        map(lambda dim_sum: dim_sum / len(shape), [sum(x) for x in zip(*shape)])
    )
    assert (
        len(centroid) == 2
    ), f"Expected 2-D centroid for {shape=}, got {len(centroid)}-D"
    return centroid


def euclidean_distance(
    point_a: tuple[float, float], point_b: tuple[float, float]
) -> float:
    assert len(point_a) == 2, f"Expected 2-D point={point_a}, got {len(point_a)}-D"
    return math.hypot(*map(sub, point_a, point_b))


def get_nearest_edge_id(polygon_id: str, lane_id_list: list[str]) -> str:
    nearest_lane_id = min(
        lane_id_list,
        key=lambda lane_id: euclidean_distance(
            shape2centroid(traci.polygon.getShape(polygon_id)),
            shape2centroid(traci.lane.getShape(lane_id)),
        ),
    )
    nearest_lane_center = shape2centroid(traci.lane.getShape(nearest_lane_id))

    traci.polygon.add(
        polygonID=f"edge#{nearest_lane_id}",
        shape=[
            (nearest_lane_center[0], nearest_lane_center[1]),
            (nearest_lane_center[0] + 10, nearest_lane_center[1]),
            (nearest_lane_center[0] + 10, nearest_lane_center[1] + 10),
            (nearest_lane_center[0], nearest_lane_center[1] + 10),
            (nearest_lane_center[0], nearest_lane_center[1]),
        ],
        color=[0, 255, 0],
        polygonType="edge",
        fill=True,
    )

    nearest_edge_id = traci.lane.getEdgeID(nearest_lane_id)

    logger.debug(f"Found {nearest_edge_id=} of {polygon_id=}")
    return nearest_edge_id


def get_lane_list() -> list[str]:
    return list(filter(lambda id: not id.startswith(":"), traci.lane.getIDList()))


def get_building_id_list() -> list[str]:
    return list(
        filter(
            lambda polygon_id: traci.polygon.getType(polygon_id) == "building",
            traci.polygon.getIDList(),
        )
    )
