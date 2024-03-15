import gzip
import json
import os
import pickle
import sys

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

sys.path.append("..")


def test_euclidean_distance() -> None:
    from drone_cab.utils import euclidean_distance

    assert euclidean_distance((0.0, 0.0), (3.0, 4.0)) == 5.0
    assert euclidean_distance((3.0, 4.0), (3.0, 4.0)) == 0.0
    assert euclidean_distance((3.0, 4.0), (0.0, 0.0)) == 5.0
    assert euclidean_distance((-1.0, 1.0), (1.0, -1.0)) == (8**0.5)


def test_shape2centroid() -> None:
    from drone_cab.utils import shape2centroid

    assert shape2centroid([(1.0, 1.0)]) == (1.0, 1.0)
    assert shape2centroid([(1.0, 2.0), (3.0, 4.0)]) == (2.0, 3.0)
    assert shape2centroid([(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]) == (3.0, 4.0)


def test_get_lane_list() -> None:
    from drone_cab.utils import get_lane_list

    traci.start(
        [
            "sumo",
            "-c",
            os.path.join("data", "config.sumocfg"),
            "-d",
            "150",
        ]
    )

    try:
        with gzip.open(os.path.join("data", "lane_list.json.gz"), "rt") as lane_list_zipfile:
            expected_lane_list = json.load(lane_list_zipfile)
            assert not set(get_lane_list()) ^ set(expected_lane_list)
    finally:
        traci.close()


def test_get_building_id_list() -> None:
    from drone_cab.utils import get_building_id_list

    traci.start(
        [
            "sumo",
            "-c",
            os.path.join("data", "config.sumocfg"),
            "-d",
            "150",
        ]
    )

    try:
        with gzip.open(os.path.join("data", "building_id_list.json.gz"), "rt") as building_id_list_zipfile:
            expected_building_id_list = json.load(building_id_list_zipfile)
            assert not set(get_building_id_list()) ^ set(expected_building_id_list)
    finally:
        traci.close()


def test_get_nearest_edge_id() -> None:
    from drone_cab.utils import get_lane_list, get_nearest_edge_id

    traci.start(
        [
            "sumo",
            "-c",
            os.path.join("data", "config.sumocfg"),
            "-d",
            "150",
        ]
    )

    try:
        assert get_nearest_edge_id("239796134", get_lane_list()) == "158320863#6"
    finally:
        traci.close()
