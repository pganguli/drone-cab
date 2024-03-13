import traci

from drone_cab.utils import get_lane_list, get_nearest_edge_id, shape2centroid

WAREHOUSE_ID = "239796134"


class Warehouse:
    def __init__(self, warehouse_id: str) -> None:
        self.id = warehouse_id
        traci.polygon.setColor(self.id, (0, 0, 255))

        self.center = shape2centroid(traci.polygon.getShape(self.id))
        self.nearest_edge_id = get_nearest_edge_id(self.id, get_lane_list())

    def __str__(self) -> str:
        return self.id

    def __repr__(self) -> str:
        return f"Warehouse({self.id})"


def get_warehouse_id() -> str:
    return WAREHOUSE_ID
