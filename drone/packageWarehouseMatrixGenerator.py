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

id_list = traci.polygon.getIDList()
type_list = [traci.polygon.getType(id) for id in id_list]

id_type_list = list(zip(id_list, type_list))
id_type_list = list(filter(lambda x: x[1] == "building", id_type_list))

# id_type_list = random.sample(id_type_list, k=4)

no_of_warehouses = 4
warehouse_capacity = {}
for i in range(no_of_warehouses):
    warehouse_capacity[i] = random.randint(2,5)
print(warehouse_capacity)

idx = np.round(np.linspace(0, len(id_type_list) - 1, no_of_warehouses)).astype(int)
warehouses = [id_type_list[i] for i in idx]
warehouse_center = [shape2center(traci.polygon.getShape(id)) for id, _ in warehouses]

no_of_packages = 10
package_warehouse_matrix = np.zeros((no_of_packages, no_of_warehouses))

package_no = 0
while package_no < no_of_packages:
    new_package_request = random.sample(id_type_list, k=1)[0]
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
            break
        else:
            print(package_no, selected_warehouse)
            package_warehouse_distance[selected_warehouse] = math.inf
    package_no += 1

print(package_warehouse_matrix)

traci.close()
