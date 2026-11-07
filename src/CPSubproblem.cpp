#include "CPSubproblem.h"

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "ortools/sat/cp_model.h"

namespace bbcp {

using operations_research::Domain;
using operations_research::sat::BoolVar;
using operations_research::sat::CpModelBuilder;
using operations_research::sat::CpSolverResponse;
using operations_research::sat::CpSolverStatus;
using operations_research::sat::IntVar;
using operations_research::sat::LinearExpr;
using operations_research::sat::Model;
using operations_research::sat::NewSatParameters;
using operations_research::sat::SatParameters;
using operations_research::sat::SolutionBooleanValue;
using operations_research::sat::SolutionIntegerValue;
using operations_research::sat::SolveCpModel;

namespace {

LinearExpr SumBool(const std::vector<BoolVar>& vars) {
    LinearExpr expr;
    for (const BoolVar& v : vars) {
        expr += v;
    }
    return expr;
}

}  // namespace

CPSubproblem::CPSubproblem(const Instance& inst) : inst_(inst) {}

CPResult CPSubproblem::SolveDroneSubproblem(
    const std::vector<Node>& drone_customers,
    const CPOptions& options
) const {
    CPResult result;

    const int m = static_cast<int>(drone_customers.size());

    // Empty drone set: feasible with zero drone makespan.
    if (m == 0) {
        result.feasible = true;
        result.optimal = true;
        result.objective = 0;
        return result;
    }

    // Validate all customers assigned to the CP drone subproblem.
    for (Node i : drone_customers) {
        if (i <= 0 || i > inst_.n || !inst_.drone_feasible[i]) {
            result.feasible = false;
            return result;
        }
    }

    CpModelBuilder model;

    // A safe but loose horizon:
    // all drone sorties plus a possible swap before each sortie.
    int64_t horizon = 0;
    for (Node i : drone_customers) {
        horizon += inst_.DroneRoundTripTime(i) + inst_.swap_time;
    }
    horizon += 10;

    std::vector<BoolVar> h(m);
    std::vector<BoolVar> s(m);
    std::vector<IntVar> start(m);
    std::vector<IntVar> end(m);
    std::vector<IntVar> size(m);
    std::vector<IntVar> battery(m);
    std::vector<IntVar> order(m);

    std::vector<std::vector<BoolVar>> u(m, std::vector<BoolVar>(m));

    for (int a = 0; a < m; ++a) {
        Node i = drone_customers[a];

        h[a] = model.NewBoolVar().WithName("h_" + std::to_string(i));
        s[a] = model.NewBoolVar().WithName("s_" + std::to_string(i));

        start[a] = model.NewIntVar(Domain(0, horizon)).WithName(
            "start_" + std::to_string(i)
        );

        end[a] = model.NewIntVar(Domain(0, horizon)).WithName(
            "end_" + std::to_string(i)
        );

        size[a] = model.NewIntVar(Domain(0, horizon)).WithName(
            "size_" + std::to_string(i)
        );

        battery[a] = model.NewIntVar(Domain(0, inst_.battery_capacity)).WithName(
            "battery_" + std::to_string(i)
        );

        order[a] = model.NewIntVar(Domain(1, m)).WithName(
            "order_" + std::to_string(i)
        );
    }

    for (int a = 0; a < m; ++a) {
        for (int b = 0; b < m; ++b) {
            if (a == b) continue;

            u[a][b] = model.NewBoolVar().WithName(
                "u_" + std::to_string(drone_customers[a]) +
                "_" + std::to_string(drone_customers[b])
            );
        }
    }

    IntVar z = model.NewIntVar(Domain(0, horizon)).WithName("z_drone");

    // ------------------------------------------------------------------
    // Symmetry-breaking / tightening 1:
    // Use exactly min(K, m) anonymous drone chains.
    //
    // This is safe here because this CP subproblem has no fixed activation
    // cost for using an additional drone. More chains cannot make the
    // makespan worse.
    // ------------------------------------------------------------------
    const int used_chains = std::min(inst_.n_drones, m);

    {
        std::vector<BoolVar> heads;
        for (int a = 0; a < m; ++a) {
            heads.push_back(h[a]);
        }

        model.AddEquality(SumBool(heads), used_chains);
    }

    // ------------------------------------------------------------------
    // Symmetry-breaking / tightening 2:
    // If H chains cover m customers, then exactly m-H successor arcs are used.
    // ------------------------------------------------------------------
    {
        std::vector<BoolVar> arcs;
        for (int a = 0; a < m; ++a) {
            for (int b = 0; b < m; ++b) {
                if (a == b) continue;
                arcs.push_back(u[a][b]);
            }
        }

        model.AddEquality(SumBool(arcs), m - used_chains);
    }

    // ------------------------------------------------------------------
    // Chain and timing constraints.
    // ------------------------------------------------------------------
    for (int a = 0; a < m; ++a) {
        Node i = drone_customers[a];

        // h_i + incoming_i = 1 because every customer in this subproblem
        // must be served by a drone.
        LinearExpr incoming = h[a];
        for (int b = 0; b < m; ++b) {
            if (a == b) continue;
            incoming += u[b][a];
        }
        model.AddEquality(incoming, 1);

        // At most one successor.
        LinearExpr outgoing;
        for (int b = 0; b < m; ++b) {
            if (a == b) continue;
            outgoing += u[a][b];
        }
        model.AddLessOrEqual(outgoing, 1);

        // If a is a head, it starts at time 0, no swap before it,
        // and its battery is full.
        model.AddEquality(start[a], 0).OnlyEnforceIf(h[a]);
        model.AddEquality(s[a], 0).OnlyEnforceIf(h[a]);
        model.AddEquality(battery[a], inst_.battery_capacity).OnlyEnforceIf(h[a]);

        // Tighten the order variable:
        //   head     => order = 1
        //   non-head => order >= 2
        model.AddEquality(order[a], 1).OnlyEnforceIf(h[a]);
        model.AddGreaterOrEqual(order[a], 2).OnlyEnforceIf(h[a].Not());

        // Duration = round-trip flight time + swap_time * s_i.
        //
        // If s_i = 1, it means the drone swaps battery immediately before
        // launching the sortie 0 -> i -> 0.
        model.AddEquality(
            size[a],
            LinearExpr(inst_.DroneRoundTripTime(i)) + inst_.swap_time * s[a]
        );

        model.AddEquality(end[a], start[a] + size[a]);

        // Must have enough battery before serving i.
        model.AddGreaterOrEqual(battery[a], inst_.energy[i]);

        // Drone makespan.
        model.AddGreaterOrEqual(z, end[a]);
    }

    // ------------------------------------------------------------------
    // Successor constraints.
    // ------------------------------------------------------------------
    for (int a = 0; a < m; ++a) {
        Node i = drone_customers[a];

        for (int b = 0; b < m; ++b) {
            if (a == b) continue;

            // Customer-order symmetry breaking:
            //
            //     u_ij <= s_j, for i > j.
            //
            // If i > j and j immediately follows i, then there must be
            // a battery swap before j.
            Node j = drone_customers[b];

            if (i > j) {
                model.AddImplication(u[a][b], s[b]);
            }


            // If b has a predecessor a, then b cannot be a chain head.
            model.AddImplication(u[a][b], h[b].Not());

            // If u_ij = 1, then start_j = end_i.
            model.AddEquality(start[b], end[a]).OnlyEnforceIf(u[a][b]);

            // Order eliminates directed cycles.
            model.AddGreaterOrEqual(order[b], order[a] + 1).OnlyEnforceIf(u[a][b]);

            // If no swap before b, battery_b = battery_a - energy_i.
            model.AddEquality(
                battery[b],
                battery[a] - inst_.energy[i]
            ).OnlyEnforceIf({u[a][b], s[b].Not()});

            // If swap before b, battery_b is reset to full capacity.
            model.AddEquality(
                battery[b],
                inst_.battery_capacity
            ).OnlyEnforceIf({u[a][b], s[b]});
        }
    }

    // ------------------------------------------------------------------
    // Aggregate energy cut:
    //
    //     E * (#heads + #swaps) >= sum_i energy_i
    //
    // This is redundant but helps propagation.
    // ------------------------------------------------------------------
    {
        LinearExpr available;

        for (int a = 0; a < m; ++a) {
            available += inst_.battery_capacity * h[a];
            available += inst_.battery_capacity * s[a];
        }

        int64_t required = 0;
        for (Node i : drone_customers) {
            required += inst_.energy[i];
        }

        model.AddGreaterOrEqual(available, required);
    }

    model.Minimize(z);

    // ------------------------------------------------------------------
    // Solve CP-SAT model.
    // ------------------------------------------------------------------
    Model cp_model;

    SatParameters params;
    params.set_max_time_in_seconds(options.time_limit_seconds);
    params.set_num_search_workers(1);
    params.set_log_search_progress(options.log_search);

    if (options.stop_after_first_solution) {
        params.set_stop_after_first_solution(true);
    }

    cp_model.Add(NewSatParameters(params));

    const CpSolverResponse response = SolveCpModel(model.Build(), &cp_model);

    result.wall_time = response.wall_time();

    const CpSolverStatus status = response.status();
    result.feasible = (
        status == CpSolverStatus::OPTIMAL ||
        status == CpSolverStatus::FEASIBLE
    );
    result.optimal = (status == CpSolverStatus::OPTIMAL);

    if (!result.feasible) {
        return result;
    }

    result.objective = SolutionIntegerValue(response, z);

    // ------------------------------------------------------------------
    // Decode chains from heads and successor arcs.
    //
    // Also decode swap decisions:
    //   s_i = 1 means a battery swap is performed immediately before i.
    //
    // The decoded arrays satisfy:
    //   result.chains[k][p]      = customer at position p in chain k
    //   result.swap_before[k][p] = 1 iff swap before that customer
    // ------------------------------------------------------------------
    std::vector<bool> reached(m, false);

    for (int a = 0; a < m; ++a) {
        if (!SolutionBooleanValue(response, h[a])) continue;

        std::vector<Node> chain;
        std::vector<int> chain_swaps;

        int cur = a;

        while (true) {
            if (reached[cur]) break;

            reached[cur] = true;

            chain.push_back(drone_customers[cur]);
            chain_swaps.push_back(
                SolutionBooleanValue(response, s[cur]) ? 1 : 0
            );

            int nxt = -1;

            for (int b = 0; b < m; ++b) {
                if (b == cur) continue;

                if (SolutionBooleanValue(response, u[cur][b])) {
                    nxt = b;
                    break;
                }
            }

            if (nxt == -1) break;

            cur = nxt;
        }

        result.chains.push_back(chain);
        result.swap_before.push_back(chain_swaps);
    }

    // Safety fallback: if something was not reached, report it as a
    // singleton chain. This should not happen for a valid CP solution,
    // but it prevents silent loss during debugging.
    for (int a = 0; a < m; ++a) {
        if (reached[a]) continue;

        result.chains.push_back({drone_customers[a]});
        result.swap_before.push_back({
            SolutionBooleanValue(response, s[a]) ? 1 : 0
        });
    }

    return result;
}

}  // namespace bbcp
