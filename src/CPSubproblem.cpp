#include "CPSubproblem.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "ortools/sat/cp_model.h"

namespace bbcp
{

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

    namespace
    {
        LinearExpr SumBool(const std::vector<BoolVar> &v)
        {
            LinearExpr expr;
            for (const BoolVar &b : v)
                expr += b;
            return expr;
        }

        // Construct a feasible hint: H nonempty chains, each sorted by customer id.
        // The hint is optional: feasibility and exactness never rely on its quality.
        void AddGreedyHint(CpModelBuilder &model,
                           const Instance &inst,
                           const std::vector<Node> &D,
                           const std::vector<BoolVar> &h,
                           const std::vector<BoolVar> &s,
                           const std::vector<IntVar> &start,
                           const std::vector<IntVar> &end,
                           const std::vector<IntVar> &size,
                           const std::vector<IntVar> &battery,
                           const std::vector<IntVar> &order,
                           const std::vector<std::vector<BoolVar>> &u,
                           const IntVar &z, int H)
        {
            const int m = static_cast<int>(D.size());
            std::vector<int> predecessor(m, -1), position(m, 1), swap(m, 0);
            std::vector<int64_t> finish(H, 0), remaining(H, inst.battery_capacity);
            std::vector<int> last(H, -1);
            std::vector<int64_t> st(m, 0), sz(m, 0), batt(m, 0);

            for (int a = 0; a < m; ++a)
            {
                int k = (a < H ? a : 0);
                if (a >= H)
                {
                    for (int t = 1; t < H; ++t)
                        if (finish[t] < finish[k])
                            k = t;
                }
                predecessor[a] = last[k];
                if (last[k] != -1)
                    position[a] = position[last[k]] + 1;
                const int64_t need = inst.energy[D[a]];
                if (last[k] != -1 && remaining[k] < need)
                {
                    swap[a] = 1;
                    remaining[k] = inst.battery_capacity;
                }
                st[a] = finish[k];
                batt[a] = remaining[k];
                sz[a] = inst.DroneRoundTripTime(D[a]) + inst.swap_time * swap[a];
                finish[k] += sz[a];
                remaining[k] -= need;
                last[k] = a;
            }

            for (int a = 0; a < m; ++a)
            {
                model.AddHint(h[a], predecessor[a] == -1);
                model.AddHint(s[a], swap[a] == 1);
                model.AddHint(start[a], st[a]);
                model.AddHint(end[a], st[a] + sz[a]);
                model.AddHint(size[a], sz[a]);
                model.AddHint(battery[a], batt[a]);
                model.AddHint(order[a], position[a]);
            }
            for (int a = 0; a < m; ++a)
                for (int b = 0; b < m; ++b)
                    if (a != b)
                        model.AddHint(u[a][b], predecessor[b] == a);
            model.AddHint(z, *std::max_element(finish.begin(), finish.end()));
        }
    } // namespace

    CPSubproblem::CPSubproblem(const Instance &inst) : inst_(inst) {}

    CPResult CPSubproblem::SolveDroneSubproblem(
        const std::vector<Node> &drone_customers,
        const CPOptions &options,
        int64_t deadline) const
    {
        CPResult result;
        result.is_decision = deadline >= 0;

        // Sorting guarantees consistency of the symmetry-breaking rule:
        // a backward link i>j must have a battery swap before j.
        std::vector<Node> D = drone_customers;
        std::sort(D.begin(), D.end());
        const int m = static_cast<int>(D.size());

        if (inst_.n_drones < 1 || inst_.battery_capacity <= 0 ||
            inst_.swap_time < 0 || options.time_limit_seconds < 0 ||
            std::adjacent_find(D.begin(), D.end()) != D.end())
        {
            result.status = CPStatus::ModelInvalid;
            return result;
        }

        if (m == 0)
        {
            result.status = CPStatus::Optimal;
            result.feasible = true;
            result.optimal = !result.is_decision;
            result.objective = 0;
            result.lower_bound = 0;
            return result;
        }

        int64_t horizon = 0;
        for (Node i : D)
        {
            if (i < 1 || i > inst_.n || !inst_.drone_feasible[i] ||
                inst_.energy[i] < 0 || inst_.energy[i] > inst_.battery_capacity ||
                inst_.DroneRoundTripTime(i) < 0)
            {
                result.status = CPStatus::ModelInvalid;
                return result;
            }
            const int64_t r = inst_.DroneRoundTripTime(i);
            if (horizon > INF64 - r - inst_.swap_time)
            {
                result.status = CPStatus::ModelInvalid;
                return result;
            }
            horizon += r + inst_.swap_time;
        }
        // All sorties on one drone with a swap between successive sorties.
        // This constructs a safe finite horizon, including for m=1.
        const int H = std::min(inst_.n_drones, m);

        CpModelBuilder model;
        std::vector<BoolVar> h(m), s(m);
        std::vector<IntVar> start(m), end(m), size(m), battery(m), order(m);
        std::vector<std::vector<BoolVar>> u(m, std::vector<BoolVar>(m));

        for (int a = 0; a < m; ++a)
        {
            const Node i = D[a];
            h[a] = model.NewBoolVar().WithName("h_" + std::to_string(i));
            s[a] = model.NewBoolVar().WithName("swap_" + std::to_string(i));
            start[a] = model.NewIntVar(Domain(0, horizon)).WithName("start_" + std::to_string(i));
            end[a] = model.NewIntVar(Domain(0, horizon)).WithName("end_" + std::to_string(i));
            size[a] = model.NewIntVar(Domain(0, horizon)).WithName("size_" + std::to_string(i));
            battery[a] = model.NewIntVar(Domain(0, inst_.battery_capacity)).WithName("battery_" + std::to_string(i));
            order[a] = model.NewIntVar(Domain(1, m)).WithName("order_" + std::to_string(i));
        }
        for (int a = 0; a < m; ++a)
            for (int b = 0; b < m; ++b)
                if (a != b)
                    u[a][b] = model.NewBoolVar().WithName(
                        "u_" + std::to_string(D[a]) + "_" + std::to_string(D[b]));

        IntVar z = model.NewIntVar(Domain(0, horizon)).WithName("z_drone");

        // Exactly H non-empty drone chains: safe because splitting chains
        // cannot increase makespan with independent drone batteries.
        model.AddEquality(SumBool(h), H);
        std::vector<BoolVar> arcs;
        for (int a = 0; a < m; ++a)
            for (int b = 0; b < m; ++b)
                if (a != b)
                    arcs.push_back(u[a][b]);
        model.AddEquality(SumBool(arcs), m - H);

        for (int a = 0; a < m; ++a)
        {
            LinearExpr incoming = h[a];
            LinearExpr outgoing;
            for (int b = 0; b < m; ++b)
            {
                if (a == b)
                    continue;
                incoming += u[b][a];
                outgoing += u[a][b];
            }
            model.AddEquality(incoming, 1);
            model.AddLessOrEqual(outgoing, 1);

            model.AddEquality(start[a], 0).OnlyEnforceIf(h[a]);
            model.AddEquality(s[a], 0).OnlyEnforceIf(h[a]);
            model.AddEquality(battery[a], inst_.battery_capacity).OnlyEnforceIf(h[a]);
            model.AddEquality(order[a], 1).OnlyEnforceIf(h[a]);
            model.AddGreaterOrEqual(order[a], 2).OnlyEnforceIf(h[a].Not());

            model.AddEquality(size[a], LinearExpr(inst_.DroneRoundTripTime(D[a])) + inst_.swap_time * s[a]);
            model.AddEquality(end[a], start[a] + size[a]);
            model.AddGreaterOrEqual(battery[a], inst_.energy[D[a]]);
            model.AddGreaterOrEqual(z, end[a]);
        }

        for (int a = 0; a < m; ++a)
        {
            const Node i = D[a];
            for (int b = 0; b < m; ++b)
            {
                if (a == b)
                    continue;
                const Node j = D[b];
                // Symmetry: within a battery cycle sort customers by ID.
                if (i > j)
                    model.AddImplication(u[a][b], s[b]);
                // Redundant battery incompatibility propagates mandatory swaps.
                if (inst_.energy[i] + inst_.energy[j] > inst_.battery_capacity)
                    model.AddImplication(u[a][b], s[b]);

                model.AddImplication(u[a][b], h[b].Not());
                model.AddEquality(start[b], end[a]).OnlyEnforceIf(u[a][b]);
                model.AddGreaterOrEqual(order[b], order[a] + 1).OnlyEnforceIf(u[a][b]);
                model.AddEquality(battery[b], battery[a] - inst_.energy[i])
                    .OnlyEnforceIf({u[a][b], s[b].Not()});
                model.AddEquality(battery[b], inst_.battery_capacity)
                    .OnlyEnforceIf({u[a][b], s[b]});
            }
        }

        // H battery cycles initially, plus one new cycle for each swap.
        LinearExpr energy_available;
        int64_t total_energy = 0;
        for (int a = 0; a < m; ++a)
        {
            energy_available += inst_.battery_capacity * h[a];
            energy_available += inst_.battery_capacity * s[a];
            total_energy += inst_.energy[D[a]];
        }
        model.AddGreaterOrEqual(energy_available, total_energy);

        if (options.use_warm_start)
            AddGreedyHint(model, inst_, D, h, s, start, end, size, battery,
                          order, u, z, H);

        if (result.is_decision)
            model.AddLessOrEqual(z, deadline);
        else
            model.Minimize(z);

        Model cp_model;
        SatParameters params;
        if (options.time_limit_seconds > 0.0)
            params.set_max_time_in_seconds(options.time_limit_seconds);
        params.set_num_search_workers(8);
        params.set_log_search_progress(options.log_search);
        if (options.stop_after_first_solution)
            params.set_stop_after_first_solution(true);
        cp_model.Add(NewSatParameters(params));

        const CpSolverResponse response = SolveCpModel(model.Build(), &cp_model);
        result.wall_time = response.wall_time();

        switch (response.status())
        {
        case CpSolverStatus::OPTIMAL:
            result.status = CPStatus::Optimal;
            break;
        case CpSolverStatus::FEASIBLE:
            result.status = CPStatus::Feasible;
            break;
        case CpSolverStatus::INFEASIBLE:
            result.status = CPStatus::Infeasible;
            break;
        case CpSolverStatus::MODEL_INVALID:
            result.status = CPStatus::ModelInvalid;
            break;
        default:
            result.status = CPStatus::Unknown;
            break;
        }
        result.feasible = (result.status == CPStatus::Optimal ||
                           result.status == CPStatus::Feasible);
        result.optimal = (!result.is_decision && result.status == CPStatus::Optimal);

        if (!result.is_decision && result.status != CPStatus::ModelInvalid &&
            result.status != CPStatus::Infeasible)
        {
            // For an integral objective, ceil(certified_bound - tolerance)
            // remains a certified integer lower bound (within numerical tolerance).
            const double b = response.best_objective_bound();
            if (std::isfinite(b) && b >= 0.0 && b < static_cast<double>(INF64))
                result.lower_bound = static_cast<int64_t>(std::ceil(b - 1e-6));
        }
        if (!result.feasible)
            return result;

        result.objective = SolutionIntegerValue(response, z);
        if (result.optimal)
            result.lower_bound = result.objective;

        std::vector<bool> reached(m, false);
        for (int a = 0; a < m; ++a)
        {
            if (!SolutionBooleanValue(response, h[a]))
                continue;
            std::vector<Node> chain;
            std::vector<int> swaps;
            int cur = a;
            while (cur != -1 && !reached[cur])
            {
                reached[cur] = true;
                chain.push_back(D[cur]);
                swaps.push_back(SolutionBooleanValue(response, s[cur]) ? 1 : 0);
                int next = -1;
                for (int b = 0; b < m; ++b)
                {
                    if (b != cur && SolutionBooleanValue(response, u[cur][b]))
                    {
                        next = b;
                        break;
                    }
                }
                cur = next;
            }
            result.chains.push_back(std::move(chain));
            result.swap_before.push_back(std::move(swaps));
        }
        // Do not fabricate missing drone jobs during decoding.
        if (std::find(reached.begin(), reached.end(), false) != reached.end())
        {
            result.status = CPStatus::ModelInvalid;
            result.feasible = false;
            result.optimal = false;
            result.chains.clear();
            result.swap_before.clear();
        }
        return result;
    }

} // namespace bbcp
