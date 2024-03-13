import math
import os
import sys
from operator import sub

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci


def shape2centroid(shape: list[tuple[float, float]]) -> tuple[float, float]:
    return tuple(np.mean(np.asarray(shape), axis=0))


def euclidean_distance(
    point_a: tuple[float, float], point_b: tuple[float, float]
) -> float:
    return math.hypot(*map(sub, point_a, point_b))


def get_nearest_edge(polygon_id: str, lane_id_list: list[str]) -> str:
    global EDGE_COUNTER

    nearest_lane_id = min(
        lane_id_list,
        key=lambda lane_id: euclidean_distance(
            shape2centroid(traci.polygon.getShape(polygon_id)),
            shape2centroid(traci.lane.getShape(lane_id)),
        ),
    )
    nearest_lane_center = shape2centroid(traci.lane.getShape(nearest_lane_id))

    traci.polygon.add(
        polygonID=f"edge#{EDGE_COUNTER}",
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
    EDGE_COUNTER += 1

    nearest_edge_id = traci.lane.getEdgeID(nearest_lane_id)
    return nearest_edge_id


def get_lane_id_list() -> list[str]:
    return traci.lane.getIDList()


def filter_internal_list_id(id_list: list[str]) -> list[str]:
    return list(filter(lambda id: not id.startswith(":"), id_list))


def get_polygon_id_list() -> list[str]:
    return traci.polygon.getIDList()


def get_building_id_list(polygon_id_list: list[str]) -> list[str]:
    return list(
        filter(
            lambda polygon_id: traci.polygon.getType(polygon_id) == "building",
            polygon_id_list,
        )
    )


def get_pickup_edge_dict(
    pickup_id_list: list[str], lane_id_list: list[str]
) -> dict[str, str]:
    return {
        pickup_id: get_nearest_edge(pickup_id, lane_id_list)
        for pickup_id in pickup_id_list
    }
