import gurobipy as gp
from itertools import combinations, permutations
from math import hypot

##################################
#     AGATZ MODEL IN GUROBI      #
##################################

# ---------------------------------------------------------
# INSTANCE
# ---------------------------------------------------------

truckSpeed = 1.0
droneSpeed = 2.0

# Example:
# 0 = depot
# 1,2,3,4 = customers

coordinates = {
    0: (23.538033776916443, 19.71269830635222),  # depot
    1: (6.688034320166812, -59.30148092144562),  # v1
    2: (247.10237029541915, 16.582726892301235), # u2
    3: (-20.02811932620873, 31.940292850278276), # v3
    4: (-6.761722936087144, -23.718941163504528),  # v4j
    5: (189.94135833817387, -25.928848808968002)   # u5
}

depot = 0

V = list(coordinates.keys())

numVertices = len(V)

# Customers that can be served by drone
droneEligible = set(V)
droneEligible.remove(depot)

customers = [v for v in V if v != depot]


# ---------------------------------------------------------
# DISTANCE
# ---------------------------------------------------------

def distance(i, j):

    xi, yi = coordinates[i]
    xj, yj = coordinates[j]

    return hypot(xj - xi, yj - yi)


# ---------------------------------------------------------
# ROUTE TIME
# ---------------------------------------------------------

def routeTime(route, speed):

    if len(route) < 2:
        return 0.0

    total = 0.0

    for i in range(len(route) - 1):

        total += distance(
            route[i],
            route[i + 1]
        )

    return total / speed


# ---------------------------------------------------------
# GENERATE ALL ORDERED SUBSETS
#
# For {1,2,3}, generates:
#
# ()
# (1,)
# (2,)
# (3,)
# (1,2)
# (2,1)
# ...
# (1,2,3)
# (1,3,2)
# ...
# ---------------------------------------------------------

def orderedSubsets(items):

    yield tuple()

    for size in range(1, len(items) + 1):

        for p in permutations(items, size):

            yield p


# ---------------------------------------------------------
# GENERATE ALL OPERATIONS
# ---------------------------------------------------------

def generateOperations():

    operations = []

    # =====================================================
    # TYPE 0
    #
    # Trivial operation at the depot
    #
    # This appears in the output of the Agatz code:
    #
    # 0  0  -1  0
    #
    # cost = 0
    # =====================================================

    operations.append({

        "start": depot,
        "end": depot,

        "truck_internal": tuple(),

        "drone_node": None,

        "truck_route": (depot, depot),

        "drone_route": None,

        "truck_time": 0.0,
        "drone_time": 0.0,

        "cost": 0.0,

        "nodes": {depot}
    })


    # =====================================================
    # TYPE 1
    #
    # No drone flight
    #
    # Truck and drone travel together:
    #
    #      i --------> j
    #
    # =====================================================

    for start in V:

        for end in V:

            if start == end:
                continue

            truck_route = (
                start,
                end
            )

            truck_time = routeTime(
                truck_route,
                truckSpeed
            )

            operations.append({

                "start": start,
                "end": end,

                "truck_internal": tuple(),

                "drone_node": None,

                "truck_route": truck_route,

                "drone_route": None,

                "truck_time": truck_time,
                "drone_time": 0.0,

                "cost": truck_time,

                "nodes": {
                    start,
                    end
                }
            })


    # =====================================================
    # TYPE 2
    #
    # One drone flight
    #
    # Truck:
    #
    # start -> t1 -> t2 -> ... -> end
    #
    # Drone:
    #
    # start -> drone_node -> end
    #
    # =====================================================

    for start in V:

        for end in V:

            for drone_node in droneEligible:

                # Drone customer cannot also be the
                # launch/rendezvous node

                if drone_node == start:
                    continue

                if drone_node == end:
                    continue


                # -----------------------------------------
                # Possible internal truck customers
                # -----------------------------------------

                possibleTruckNodes = [

                    v for v in customers

                    if v != start
                    and v != end
                    and v != drone_node
                ]


                # -----------------------------------------
                # Generate every ordered subset
                # -----------------------------------------

                for internal in orderedSubsets(
                        possibleTruckNodes):

                    truck_route = (
                        start,
                        *internal,
                        end
                    )

                    drone_route = (
                        start,
                        drone_node,
                        end
                    )


                    # -------------------------------------
                    # Times
                    # -------------------------------------

                    truck_time = routeTime(
                        truck_route,
                        truckSpeed
                    )

                    drone_time = routeTime(
                        drone_route,
                        droneSpeed
                    )


                    # Synchronization time

                    cost = max(
                        truck_time,
                        drone_time
                    )


                    # -------------------------------------
                    # Nodes contained in operation
                    # -------------------------------------

                    containedNodes = set(
                        truck_route
                    )

                    containedNodes.add(
                        drone_node
                    )


                    # -------------------------------------
                    # Store operation
                    # -------------------------------------

                    operations.append({

                        "start": start,
                        "end": end,

                        "truck_internal": internal,

                        "drone_node": drone_node,

                        "truck_route": truck_route,

                        "drone_route": drone_route,

                        "truck_time": truck_time,

                        "drone_time": drone_time,

                        "cost": cost,

                        "nodes": containedNodes
                    })


    return operations


# =========================================================
# GENERATE OPERATIONS
# =========================================================

operations = generateOperations()


# ---------------------------------------------------------
# O
# ---------------------------------------------------------

O = list(range(len(operations)))

numOperations = len(O)


# ---------------------------------------------------------
# COST c[o]
# ---------------------------------------------------------

c = [

    operations[o]["cost"]

    for o in O
]


# ---------------------------------------------------------
# O(v)
#
# All operations containing v
# ---------------------------------------------------------

O_v = {

    v: [

        o for o in O

        if v in operations[o]["nodes"]

    ]

    for v in V
}


# ---------------------------------------------------------
# O-(v)
#
# Operations STARTING at v
# ---------------------------------------------------------

O_minus_v = {

    v: [

        o for o in O

        if operations[o]["start"] == v

    ]

    for v in V
}


# ---------------------------------------------------------
# O+(v)
#
# Operations ENDING at v
# ---------------------------------------------------------

O_plus_v = {

    v: [

        o for o in O

        if operations[o]["end"] == v

    ]

    for v in V
}


# =========================================================
# SHOW GENERATED OPERATIONS
# =========================================================

print()
print("NUMBER OF OPERATIONS =", numOperations)
print()


for o in O:

    op = operations[o]

    print(
        f"{o:4d}: "
        f"start={op['start']} "
        f"end={op['end']} "
        f"drone={op['drone_node']} "
        f"internal={op['truck_internal']} "
        f"truck={op['truck_route']} "
        f"droneRoute={op['drone_route']} "
        f"cost={op['cost']:.4f}"
    )


# =========================================================
# GUROBI MODEL
# =========================================================

model = gp.Model("Agatz_TSPD")


# ---------------------------------------------------------
# VARIABLES
# ---------------------------------------------------------

x = model.addVars(
    O,
    vtype=gp.GRB.BINARY,
    name="x"
)

y = model.addVars(
    V,
    vtype=gp.GRB.BINARY,
    name="y"
)


# ---------------------------------------------------------
# OBJECTIVE
# ---------------------------------------------------------

model.setObjective(

    gp.quicksum(
        c[o] * x[o]
        for o in O
    ),

    sense=gp.GRB.MINIMIZE
)


# =========================================================
# (14)
#
# Every node must belong to at least one selected operation
# =========================================================

model.addConstrs(

    (
        gp.quicksum(
            x[o]
            for o in O_v[v]
        )
        >= 1

        for v in V
    ),

    name="cover"
)


# =========================================================
# (15)
# =========================================================

model.addConstrs(

    (
        gp.quicksum(
            x[o]
            for o in O_plus_v[v]
        )
        <= numVertices * y[v]

        for v in V
    ),

    name="active"
)


# =========================================================
# (16)
#
# Eulerian flow balance
# =========================================================

model.addConstrs(

    (
        gp.quicksum(
            x[o]
            for o in O_plus_v[v]
        )

        ==

        gp.quicksum(
            x[o]
            for o in O_minus_v[v]
        )

        for v in V
    ),

    name="balance"
)


# =========================================================
# GENERATE SUBSETS S
# =========================================================

non_depot = [

    v for v in V

    if v != depot
]

subsets = []


for size in range(
        1,
        len(non_depot) + 1):

    for comb in combinations(
            non_depot,
            size):

        subsets.append(
            set(comb)
        )


# =========================================================
# (17)
#
# Connectivity
# =========================================================

for S in subsets:


    # O+(S):
    #
    # start OUTSIDE S
    # end INSIDE S

    O_plus_S = [

        o for o in O

        if operations[o]["end"] in S
        and operations[o]["start"] not in S
    ]


    for v in S:

        model.addConstr(

            gp.quicksum(
                x[o]
                for o in O_plus_S
            )

            >= y[v],

            name=
            "connect_"
            + "_".join(
                map(str, sorted(S))
            )
            + f"_v{v}"
        )


# =========================================================
# (18)
# =========================================================

model.addConstr(

    gp.quicksum(
        x[o]
        for o in O_plus_v[depot]
    )

    >= 1,

    name="depot_return"
)


# =========================================================
# (19)
# =========================================================

model.addConstr(

    y[depot] == 1,

    name="depot_active"
)


# =========================================================
# SOLVE
# =========================================================

model.optimize()


# =========================================================
# PRINT SELECTED OPERATIONS
# =========================================================

if model.status == gp.GRB.OPTIMAL:

    print()
    print("==============================")
    print("OPTIMAL SOLUTION")
    print("==============================")
    print()

    print(
        "Total cost:",
        model.objVal
    )

    print()

    for o in O:

        if x[o].X > 0.5:

            op = operations[o]

            print(
                f"Operation {o}"
            )

            print(
                "  start:",
                op["start"]
            )

            print(
                "  end:",
                op["end"]
            )

            print(
                "  truck:",
                op["truck_route"]
            )

            print(
                "  drone:",
                op["drone_route"]
            )

            print(
                "  cost:",
                op["cost"]
            )

            print()