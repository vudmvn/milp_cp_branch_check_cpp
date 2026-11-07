cmake_minimum_required(VERSION 3.18)
project(milp_master_cp_subproblem_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------
# This project uses:
#   - Gurobi C++ API for MILP master branch-and-cut callbacks.
#   - OR-Tools C++ CP-SAT for the CP subproblem.
#
# You need to provide:
#   - GUROBI_HOME, for example /opt/gurobi1103/linux64
#   - OR-Tools C++ package discoverable by find_package(ortools CONFIG REQUIRED)
# ---------------------------------------------------------------------

if(NOT DEFINED ENV{GUROBI_HOME})
    message(FATAL_ERROR "Please set GUROBI_HOME, e.g. export GUROBI_HOME=/opt/gurobi1103/linux64")
endif()

set(GUROBI_HOME $ENV{GUROBI_HOME})
set(GUROBI_INCLUDE_DIR "${GUROBI_HOME}/include")
set(GUROBI_LIB_DIR "${GUROBI_HOME}/lib")

find_package(ortools CONFIG REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GUROBI_INCLUDE_DIR}
)

add_executable(milp_cp_solver
    src/main.cpp
    src/DataReader.cpp
    src/CPSubproblem.cpp
    src/MasterProblem.cpp
)

# Adjust gurobi library name if your installed version differs.
# Common examples: gurobi110, gurobi1103, gurobi120
find_library(GUROBI_LIB
    NAMES gurobi130 gurobi120 gurobi110 gurobi1103 gurobi100
    PATHS ${GUROBI_LIB_DIR}
    REQUIRED
)

find_library(GUROBI_CXX_LIB
    NAMES gurobi_c++
    PATHS ${GUROBI_LIB_DIR}
    REQUIRED
)

target_link_libraries(milp_cp_solver
    PRIVATE
    ${GUROBI_CXX_LIB}
    ${GUROBI_LIB}
    ortools::ortools
)

target_compile_options(milp_cp_solver PRIVATE -Wall -Wextra -O2)
