README - MILP Master Branch-and-Cut + CP Subproblem in C++
=========================================================

1. What this code implements
----------------------------

This package implements the framework described in the LaTeX document:

    MILP master branch-and-cut + CP subproblem

Master problem:
    - Uses Gurobi C++.
    - Decides customer assignment:
          y_i = 1 if customer i is assigned to drone,
          t_i = 1 if customer i is assigned to truck.
    - Builds a truck path relaxation/route using x_ij variables.
    - Adds truck subtour elimination cuts lazily in a Gurobi callback.
    - Adds aggregate drone workload and energy lower bounds.
    - Calls a CP subproblem at integer master solutions.

CP subproblem:
    - Uses OR-Tools CP-SAT C++.
    - Receives the drone customer set fixed by the master solution.
    - Solves anonymous drone chains with:
          h_i: i is chain head,
          u_ij: j follows i,
          s_i: swap before i,
          start_i/end_i,
          battery_i.
    - Returns the minimum drone makespan.

Callback:
    At each integer master solution:
        1. Check truck subtours.
           If subtour exists, add lazy subtour cut.

        2. If no subtour, call CP subproblem on the drone set.

        3. If CP infeasible, add drone no-good cut:
               sum_{i in Dbar} y_i <= |Dbar| - 1

        4. If CP feasible and phi(ybar) > z_master, add optimality cut:
               z + M * HammingDistance(y, ybar) >= phi(ybar)


2. File structure
-----------------

include/Types.h
    Common data structures.

include/DataReader.h
src/DataReader.cpp
    Read the instance format:
        header lines with commas,
        node lines: id x y demand.

include/CPSubproblem.h
src/CPSubproblem.cpp
    OR-Tools CP-SAT drone subproblem.

include/MasterProblem.h
src/MasterProblem.cpp
    Gurobi MILP master and callback.

src/main.cpp
    Command-line entry point.

CMakeLists.txt
    Build configuration.


3. Dependencies
---------------

You need:

    - C++17 compiler
    - CMake >= 3.18
    - Gurobi with C++ API
    - OR-Tools C++ package

Set GUROBI_HOME, for example:

    export GUROBI_HOME=/opt/gurobi1103/linux64

For OR-Tools, configure CMAKE_PREFIX_PATH if needed:

    cmake .. -DCMAKE_PREFIX_PATH=/path/to/or-tools


4. Build
--------

mkdir -p build
cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/or-tools
cmake --build . -j


5. Run
------

Basic:

    ./milp_cp_solver ../data/30-c-0-c-swap-0.txt

With overrides:

    ./milp_cp_solver ../data/30-c-0-c-swap-0.txt 4 3 3600 5

Arguments:

    argv[1] = data file
    argv[2] = number of drones override
    argv[3] = swap time in minutes override
    argv[4] = MILP master time limit in seconds
    argv[5] = CP subproblem time limit in seconds


6. Important notes
------------------

This is a research skeleton intended to be extended.

The no-good cut for CP infeasibility is safe but weak:
    sum_{i in Dbar} y_i <= |Dbar| - 1

A stronger implementation should derive a smaller infeasible explanation set S:
    sum_{i in S} y_i <= |S| - 1

The optimality cut is also a safe assignment-specific cut:
    z + M * HammingDistance(y, ybar) >= phi(ybar)

A stronger implementation should derive subset-based LBBD cuts:
    z >= L_D(S) - M * sum_{i in S}(1 - y_i)

The master currently includes truck routing, but the CP subproblem only solves
the drone scheduling part. This follows the recommended decomposition:
    Master = assignment + truck path branch-and-cut + drone aggregate LB
    CP     = drone chain/swap/energy scheduling


7. Relation to the LaTeX formulation
------------------------------------

Master variables:
    y_i, t_i, x_ij, q, zT, zD, z

CP variables:
    h_i, u_ij, s_i, start_i, end_i, battery_i, z_drone

Cuts:
    - truck subtour cuts
    - drone no-good feasibility cuts
    - assignment-specific optimality cuts


8. Added CP symmetry-breaking / tightening constraints
------------------------------------------------------

The CP drone subproblem in src/CPSubproblem.cpp now includes the following
extra constraints.

Let:
    m = number of drone customers fixed by the MILP master,
    K = number of available drones,
    H = min(K, m).

1. Exact number of anonymous drone chains:

    sum_i h_i = H

This is safe because the drone subproblem has no fixed cost for using an
additional drone. More chains cannot increase the makespan.

2. Exact number of successor arcs:

    sum_{i != j} u_ij = m - H

If H chains cover m customers, then there must be exactly m-H successor arcs.

3. Non-head order tightening:

    h_i = 1  => order_i = 1
    h_i = 0  => order_i >= 2

This tightens the order variables used to eliminate cycles.

4. Explicit predecessor-head implication:

    u_ij = 1 => h_j = 0

This is logically implied by h_j + incoming_j = 1, but adding it explicitly
improves propagation.
