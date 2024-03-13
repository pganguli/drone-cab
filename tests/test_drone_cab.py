import sys

sys.path.append("..")

from drone_cab.utils import euclidean_distance, shape2centroid


def test_euclidean_distance() -> None:
    assert euclidean_distance((0.0, 0.0), (3.0, 4.0)) == 5.0
    assert euclidean_distance((3.0, 4.0), (3.0, 4.0)) == 0.0
    assert euclidean_distance((3.0, 4.0), (0.0, 0.0)) == 5.0
    assert euclidean_distance((-1.0, 1.0), (1.0, -1.0)) == (8**0.5)


def test_shape2centroid() -> None:
    assert shape2centroid([(1.0, 1.0)]) == (1.0, 1.0)
    assert shape2centroid([(1.0, 2.0), (3.0, 4.0)]) == (2.0, 3.0)
    assert shape2centroid([(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]) == (3.0, 4.0)
