import os
import sys
import numpy as np
import random
import math

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci


def shape2center(shape):
    tuple_sum = tuple([sum(x) for x in zip(*shape)])
    num = len(shape)
    return (tuple_sum[0] / num, tuple_sum[1] / num)


def euclidean_distance(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def distance_matrix(center_list):
    distances = []
    for i in center_list:
        row = []
        for j in center_list:
            row.append(euclidean_distance(i, j))
        distances.append(row)
    return distances


sumoCmd = ["sumo", "-c", "test.sumocfg"]

traci.start(sumoCmd)

polygon_id_list = traci.polygon.getIDList()
polygon_type_list = [traci.polygon.getType(id) for id in polygon_id_list]

polygon_id_type_list = list(zip(polygon_id_list, polygon_type_list))
building_list = list(filter(lambda x: x[1] == "building", polygon_id_type_list))

junction_id_list = traci.junction.getIDList()
junction_position_list = [traci.junction.getPosition(id) for id in junction_id_list]

junction_list = list(zip(junction_id_list, junction_position_list))

no_of_warehouses = 4
warehouse_capacity = {}
for i in range(no_of_warehouses):
    warehouse_capacity[i] = random.randint(2, 5)
print(warehouse_capacity)

warehouses = []

idx = np.round(np.linspace(0, len(junction_list) - 1, no_of_warehouses)).astype(int)
junctions = [junction_list[i] for i in idx]
for i in junctions:
    dist = math.inf
    selected = None
    for j in building_list:
        x = euclidean_distance(i[1], shape2center(traci.polygon.getShape(j[0])))
        if x < dist:
            dist = x
            selected = j[0]
    warehouses.append(selected)

warehouse_center = [shape2center(traci.polygon.getShape(id)) for id in warehouses]

step = 0
while step < 100:
    traci.simulationStep()
    step += 1

vehicle_list = traci.vehicle.getIDList()

vehicle_capacity = {id: 2 for id in vehicle_list}

no_of_packages = 10
package_warehouse_matrix = np.zeros((no_of_packages, no_of_warehouses))
package_vehicle_mapping = {}

package_no = 0
while package_no < no_of_packages:
    new_package_request = random.sample(building_list, k=1)[0]
    package_center = shape2center(traci.polygon.getShape(new_package_request[0]))
    package_warehouse_distance = [
        euclidean_distance(package_center, center) for center in warehouse_center
    ]

    while package_warehouse_distance != [math.inf] * no_of_warehouses:
        selected_warehouse = package_warehouse_distance.index(
            min(package_warehouse_distance)
        )
        if (
            package_warehouse_matrix[:, selected_warehouse].sum()
            < warehouse_capacity[selected_warehouse]
        ):
            package_warehouse_matrix[package_no, selected_warehouse] = 1

            vehicle_distance = [
                (
                    id,
                    euclidean_distance(
                        warehouse_center[selected_warehouse],
                        traci.vehicle.getPosition(id),
                    ),
                    vehicle_capacity[id]
                )
                for id in vehicle_list
                if vehicle_capacity[id] != 0
            ]
            vehicle_distance = sorted(vehicle_distance, key=lambda x: (x[1], -x[2], x[0]))
            selected_vehicle, _, _ = vehicle_distance[0]
            vehicle_capacity[selected_vehicle] -= 1
            package_vehicle_mapping[package_no] = selected_vehicle
            break
        else:
            print(package_no, selected_warehouse)
            package_warehouse_distance[selected_warehouse] = math.inf
    package_no += 1

print(package_warehouse_matrix)
print(package_vehicle_mapping)

traci.close()
