import sys

sys.path.append("..")

from drone_cab.utils import euclidean_distance, shape2centroid


def test_euclidean_distance():
    # 0-D
    assert euclidean_distance(tuple(), tuple()) == 0.0

    # 1-D
    assert euclidean_distance((0.0,), (3.0,)) == 3.0

    # 2-D
    assert euclidean_distance((0.0, 0.0), (3.0, 4.0)) == 5.0
    assert euclidean_distance((3.0, 4.0), (3.0, 4.0)) == 0.0
    assert euclidean_distance((3.0, 4.0), (0.0, 0.0)) == 5.0
    assert euclidean_distance((-1.0, 1.0), (1.0, -1.0)) == (8**0.5)

    # 3-D
    assert euclidean_distance((0.0, 0.0, 0.0), (1.0, 2.0, 2.0)) == 3.0


def test_shape2centroid():
    # 0-D
    assert shape2centroid([]) == tuple()

    # 1-D
    assert shape2centroid([(1.0,)]) == (1.0,)
    assert shape2centroid([(1.0,), (2.0,)]) == (1.5,)

    # 2-D
    assert shape2centroid([(1.0, 1.0)]) == (1.0, 1.0)
    assert shape2centroid([(1.0, 2.0), (3.0, 4.0)]) == (2.0, 3.0)
    assert shape2centroid([(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]) == (3.0, 4.0)

    # 3-D
    assert shape2centroid([(1.0, 1.0, 1.0)]) == (1.0, 1.0, 1.0)
    assert shape2centroid([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]) == (2.5, 3.5, 4.5)
    assert shape2centroid([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]) == (4.0, 5.0, 6.0)
