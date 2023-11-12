import os
import sys
import random

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci


def shape2center(shape):
    tuple_sum = tuple([sum(x) for x in zip(*shape)])
    num = len(shape)
    return (tuple_sum[0] / num, tuple_sum[1] / num)

def euclidean_distance(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

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

step = 0
while step < 1000:
    id_list = traci.polygon.getIDList()
    type_list = [traci.polygon.getType(id) for id in id_list]

    id_type_list = list(zip(id_list, type_list))
    id_type_list = list(filter(lambda x: x[1] == "building", id_type_list))
    id_type_list = random.sample(id_type_list, k=2)

    center_list = [shape2center(traci.polygon.getShape(id)) for id, _ in id_type_list]
    #print(center_list)
    print(distance_matrix(center_list))

    step += 1
    break

traci.close()
