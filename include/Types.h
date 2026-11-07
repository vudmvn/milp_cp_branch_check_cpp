#ifndef BBCP_TYPES_H
#define BBCP_TYPES_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace bbcp
{

    using Node = int;

    constexpr int64_t INF64 = std::numeric_limits<int64_t>::max() / 4;
    constexpr int64_t INF_ENERGY = 1000000000LL;

    struct Point
    {
        double x = 0.0;
        double y = 0.0;
    };

    struct Instance
    {
        // Real customers: 1..n. Start depot: 0. Dummy end depot: n+1.
        int n = 0;
        int n_drones = 1;
        int start_depot = 0;
        int end_depot = 0;

        double drone_cap = 0.0;
        double truck_speed = 0.0;
        double drone_speed = 0.0;

        int64_t battery_capacity = 928;
        int64_t swap_time = 0; // seconds

        std::vector<Point> coords;
        std::vector<double> demand;
        std::vector<std::vector<int64_t>> truck_time;
        std::vector<std::vector<int64_t>> drone_time;
        std::vector<int64_t> energy;
        std::vector<bool> drone_feasible;

        int NumNodes() const { return n + 2; }

        std::vector<Node> Customers() const
        {
            std::vector<Node> c;
            for (int i = 1; i <= n; ++i)
                c.push_back(i);
            return c;
        }

        std::vector<Node> DroneCustomers() const
        {
            std::vector<Node> c;
            for (int i = 1; i <= n; ++i)
                if (drone_feasible[i])
                    c.push_back(i);
            return c;
        }

        int64_t DroneRoundTripTime(Node i) const
        {
            return drone_time[start_depot][i] + drone_time[i][start_depot];
        }
    };

    enum class CPStatus
    {
        Optimal,
        Feasible,
        Infeasible,
        Unknown,
        ModelInvalid
    };

    inline const char *CPStatusName(CPStatus s)
    {
        switch (s)
        {
        case CPStatus::Optimal:
            return "OPTIMAL";
        case CPStatus::Feasible:
            return "FEASIBLE";
        case CPStatus::Infeasible:
            return "INFEASIBLE";
        case CPStatus::Unknown:
            return "UNKNOWN";
        case CPStatus::ModelInvalid:
            return "MODEL_INVALID";
        }
        return "UNKNOWN";
    }

    struct CPResult
    {
        CPStatus status = CPStatus::Unknown;
        bool feasible = false;     // A concrete feasible schedule exists.
        bool optimal = false;      // An optimization objective is proved optimal.
        bool is_decision = false;  // Deadline decision rather than optimization.
        int64_t objective = INF64; // Makespan of a feasible schedule, NOT an LB.
        int64_t lower_bound = 0;   // Certified optimization LB (integer seconds).
        double wall_time = 0.0;

        // Each chain is one drone's ordered depot-to-customer roundtrips.
        std::vector<std::vector<Node>> chains;
        // swap_before[k][p] = 1 if swap occurs before chains[k][p].
        std::vector<std::vector<int>> swap_before;
    };

    struct MasterSolution
    {
        bool feasible = false;
        double z = 0.0;
        double z_truck = 0.0;
        double z_drone = 0.0;
        std::vector<int> y;
        std::vector<std::vector<int>> x_truck;
    };

} // namespace bbcp
#endif // BBCP_TYPES_H
