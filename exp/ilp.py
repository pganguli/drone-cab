import pulp
import random
from itertools import chain, combinations

# function to get power set
def getSubsetsForSubtourConstraint(n):
    s = list([i for i in range(n)])
    return chain.from_iterable(combinations(s, r) for r in range(2, len(s)))

# n = Number of delivery points
n = 10

# r = range of the drones
r = 200

# c = capacity of the drones
c = 5

# farthest = index of the farthest point in the set of nodes
farthest = n-1      # can be anything from 0 to n-1

# source = index of the source (pick up point) in the set of nodes
source = 0          # can be anything from 0 to n-1

# d = distance matrix
d = [[random.randrange(10, 200) for i in range(n)] for j in range(n)]
print("Distance Matrix")
for i in d:
    print(i)

# As our model is a maximization model
print("Creating Model..")
model = pulp.LpProblem('thorn-algo', pulp.LpMaximize)
print("Model created!")

# Decision variables x[i][j]
print("Adding Decision Variables..")
x = pulp.LpVariable.dicts("x", ((i, j) for i in range(n) for j in range(n)), cat = 'Binary')
print("Decision Variables added!")

# Objective function
print("Adding Objective Function..")
model += pulp.lpSum(d[i][j] * x[i, j] for i in range(n) for j in range(n))
print("Objective Function added!")

# Constraint 1 : Source and destination cannot be same
print("Adding Constraint 1 : Source and destination cannot be same ..")
for i in range(n):
    model += x[i, i] == 0
print("Constraint 1 added!")

# Constraint 2 : Each node can be source for at most once
print("Adding Constraint 2 : Each node can be source for at most once ..")
for i in range(n):
    model += pulp.lpSum(x[i, j] for j in range(n)) in [0, 1]
print("Constraint 2 added!")

# Constraint 3 : Each node can be destination for at most once
print("Adding Constraint 3 : Each node can be destination for at most once ..")
for j in range(n):
    model += pulp.lpSum(x[i, j] for i in range(n)) in [0, 1]
print("Constraint 3 added!")

# Constraint 4 : A node has an outgoing edge iff it has an incoming edge.
print("Adding Constraint 4 : A node has an outgoing edge iff it has an incoming edge. ..")
for k in range(n):
    model += pulp.lpSum(x[k, j] for j in range(n)) == pulp.lpSum(x[i, k] for i in range(n))
print("Constraint 4 added!")

# Constraint 5 : Total distance travelled must be within the range of drone
print("Adding Constraint 5 : Total distance travelled must be within the range of drone ..")
model += pulp.lpSum(d[i][j] * x[i, j] for i in range(n) for j in range(n)) <= r
print("Constraint 5 added!")

# Constraint 6 : Total number of points travelled must be within the capacity of the drone
print("Adding Constraint 6 : Total number of points travelled must be within the capacity of the drone ..")
model += pulp.lpSum(x[i, j] for i in range(n) for j in range(n)) <= c
print("Constraint 6 added!")

# Constraint 7 : The farthest point must be included in the tour
print("Adding Constraint 7 : The farthest point must be included in the tour ..")
model += pulp.lpSum(x[i, farthest] for i in range(n)) == 1     # farthest must be a destination
model += pulp.lpSum(x[farthest, j] for j in range(n)) == 1     # farthest must be a source
print("Constraint 7 added!")

# Constraint 8 : The source(pick up point) must be included in the tour
print("Adding Constraint 8 : The source(pick up point) must be included in the tour ..")
model += pulp.lpSum(x[i, source] for i in range(n)) == 1     # source must be a destination
model += pulp.lpSum(x[source, j] for j in range(n)) == 1     # source must be a source
print("Constraint 8 added!")

# Constraint 9 : Subtour elimination
print("Adding Constraint 9 : Subtour elimination ..")
model += pulp.lpSum(
    x[i, j] for i in s for j in s for s in pulp.allcombinations(
        [k for k in range(n) if pulp.lpSum(x[k, j] for j in range(n)) == 1],
        len([k for k in range(n) if pulp.lpSum(x[k, j] for j in range(n)) == 1]))) <= len(
            [k for k in range(n) if pulp.lpSum(x[k, j] for j in range(n)) == 1]) - 1
print("\nConstraint 9 added!")

# Solve model with subtour constraint
status = model.solve()

# Get the selected edges from the solution
route = [(i, j) for i in range(n) for j in range(n) if pulp.value(x[i, j]) == 1]

print("Selected Edges")
print(route)

