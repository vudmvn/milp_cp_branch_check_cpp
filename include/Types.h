#ifndef BBCP_TYPES_H
#define BBCP_TYPES_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace bbcp {

using Node = int;

constexpr int64_t INF64 = std::numeric_limits<int64_t>::max() / 4;
constexpr int64_t INF_ENERGY = 1000000000LL;

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Instance {
    // Number of real customers. Customers are 1..n.
    // Depot is 0. Dummy end depot is n + 1.
    int n = 0;

    int n_drones = 1;
    int start_depot = 0;
    int end_depot = 0;

    double drone_cap = 0.0;
    double truck_speed = 0.0;
    double drone_speed = 0.0;

    int64_t battery_capacity = 928;
    int64_t swap_time = 0;  // seconds

    // Indexed by node id. Size should be n + 2: 0..n+1.
    std::vector<Point> coords;
    std::vector<double> demand;

    // Integer travel times, in seconds.
    std::vector<std::vector<int64_t>> truck_time;
    std::vector<std::vector<int64_t>> drone_time;

    // energy[i] for customer i. energy[0] and energy[end_depot] are 0.
    std::vector<int64_t> energy;

    // drone_feasible[i] indicates customer i can be served by drone.
    std::vector<bool> drone_feasible;

    int NumNodes() const {
        return n + 2;
    }

    std::vector<Node> Customers() const {
        std::vector<Node> customers;
        for (int i = 1; i <= n; ++i) {
            customers.push_back(i);
        }
        return customers;
    }

    std::vector<Node> DroneCustomers() const {
        std::vector<Node> customers;
        for (int i = 1; i <= n; ++i) {
            if (drone_feasible[i]) {
                customers.push_back(i);
            }
        }
        return customers;
    }

    int64_t DroneRoundTripTime(Node i) const {
        return drone_time[start_depot][i] + drone_time[i][start_depot];
    }
};

struct CPResult {
    bool feasible = false;
    bool optimal = false;
    int64_t objective = INF64;
    double wall_time = 0.0;

    // Decoded anonymous drone chains.
    // chains[k][p] is the p-th customer served by anonymous drone chain k.
    //
    // Example:
    //   chains[0] = {5, 30, 21, 7}
    // means anonymous drone chain 0 performs:
    //   0 -> 5 -> 0, then 0 -> 30 -> 0, then ...
    std::vector<std::vector<Node>> chains;

    // Battery swap decisions decoded from CP variable s_i.
    //
    // swap_before[k][p] = 1 if a battery swap is performed immediately
    // before serving chains[k][p].
    //
    // Interpretation:
    //   if chains[k][p] = i and swap_before[k][p] = 1,
    //   then the printed schedule should show:
    //
    //       [SWAP] 0 -> i -> 0
    //
    // The first customer of a chain should normally have swap_before = 0
    // because the drone starts the chain with a full battery.
    std::vector<std::vector<int>> swap_before;
};

struct MasterSolution {
    bool feasible = false;
    double z = 0.0;
    double z_truck = 0.0;
    double z_drone = 0.0;

    // y[i] = 1 if customer i is assigned to drone, 0 otherwise.
    std::vector<int> y;

    // x_truck[i][j] = 1 if truck arc i -> j is selected.
    std::vector<std::vector<int>> x_truck;
};

}  // namespace bbcp

#endif
