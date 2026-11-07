#include "MasterProblem.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gurobi_c++.h"

namespace bbcp
{

    MasterSubtourMode ParseMasterSubtourMode(const std::string &raw)
    {
        std::string s = raw;
        for (char &c : s)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (s == "cut" || s == "lazy" || s == "lazycut" || s == "branch-and-cut")
            return MasterSubtourMode::LazyCut;
        if (s == "mtz")
            return MasterSubtourMode::MTZ;
        throw std::runtime_error("Unknown master mode: " + raw + ". Use cut or mtz.");
    }

    std::string MasterSubtourModeName(MasterSubtourMode m)
    {
        return m == MasterSubtourMode::LazyCut ? "cut" : "mtz";
    }

    namespace
    {

        bool TruckArcAllowed(const Instance &inst, int i, int j)
        {
            return i != j && i != inst.end_depot && j != inst.start_depot;
        }

        using XMatrix = std::vector<std::vector<int>>;

        std::vector<Node> DecodeTruckRoute(const Instance &inst, const XMatrix &x)
        {
            std::vector<Node> path;
            std::set<Node> seen;
            int cur = inst.start_depot;
            while (true)
            {
                if (!seen.insert(cur).second)
                    throw std::runtime_error("Cycle in decoded truck path");
                path.push_back(cur);
                if (cur == inst.end_depot)
                    break;
                int next = -1;
                for (int j = 0; j < inst.NumNodes(); ++j)
                    if (x[cur][j] == 1)
                    {
                        if (next != -1)
                            throw std::runtime_error("Multiple truck successors");
                        next = j;
                    }
                if (next == -1)
                    throw std::runtime_error("Missing truck successor");
                cur = next;
            }
            return path;
        }

        int64_t ComputeTruckTime(const Instance &inst, const XMatrix &x)
        {
            int64_t total = 0;
            for (int i = 0; i < inst.NumNodes(); ++i)
                for (int j = 0; j < inst.NumNodes(); ++j)
                    if (x[i][j])
                        total += inst.truck_time[i][j];
            return total;
        }

        std::string JoinRoute(const std::vector<Node> &nodes, const char *delim)
        {
            std::ostringstream oss;
            for (size_t k = 0; k < nodes.size(); ++k)
            {
                if (k)
                    oss << delim;
                oss << nodes[k];
            }
            return oss.str();
        }

        std::string DroneScheduleString(const Instance &inst, const CPResult &r)
        {
            std::ostringstream oss;
            for (size_t k = 0; k < r.chains.size(); ++k)
            {
                if (k)
                    oss << " || ";
                oss << "Drone " << k << ": ";
                const auto &c = r.chains[k];
                for (size_t a = 0; a < c.size(); ++a)
                {
                    if (a)
                        oss << " | ";
                    if (k < r.swap_before.size() &&
                        a < r.swap_before[k].size() && r.swap_before[k][a])
                        oss << "[SWAP] ";
                    oss << inst.start_depot << " -> " << c[a]
                        << " -> " << inst.start_depot;
                }
            }
            for (size_t k = r.chains.size(); k < static_cast<size_t>(inst.n_drones); ++k)
            {
                if (k || !r.chains.empty())
                    oss << " || ";
                oss << "Drone " << k << ": (not used)";
            }
            return oss.str();
        }

        std::string CsvEscape(const std::string &s)
        {
            std::string out = "\"";
            for (char c : s)
            {
                if (c == '"')
                    out += "\"\"";
                else
                    out += c;
            }
            return out + "\"";
        }

        void WriteCsv(const std::string &filename,
                      const Instance &inst,
                      const MasterOptions &options,
                      int status,
                      double objective, double lower_bound, double gap, double runtime,
                      double truck_time, double drone_makespan, double final_makespan,
                      const std::vector<Node> &truck_customers,
                      const std::vector<Node> &drone_customers,
                      const std::vector<Node> &route,
                      const CPResult &schedule,
                      int64_t cp_calls, int64_t cache_hits, int64_t lbbd_cuts,
                      bool certified)
        {
            if (filename.empty())
                return;
            bool header = true;
            {
                std::ifstream in(filename);
                if (in.good())
                    header = (in.peek() == std::ifstream::traits_type::eof());
            }
            std::ofstream out(filename, std::ios::app);
            if (!out.is_open())
            {
                std::cerr << "[WARN] Cannot open CSV " << filename << "\n";
                return;
            }
            if (header)
            {
                // Original columns preserved; new diagnostic columns appended.
                out << "instance,n_customers,n_drones,swap_time_sec,swap_time_min,"
                    << "battery_capacity,master_mode,master_time_limit_sec,"
                    << "cp_time_limit_sec,mip_gap_param,big_m,status,obj,lower_bound,"
                    << "mip_gap,runtime_sec,truck_time,drone_makespan,final_makespan,"
                    << "num_truck_customers,num_drone_customers,truck_customers,"
                    << "drone_customers,truck_route,drone_schedules,"
                    << "cp_calls,cp_cache_hits,lbbd_cuts,certified\n";
            }
            out << std::fixed << std::setprecision(6)
                << CsvEscape(options.instance_name) << ',' << inst.n << ','
                << inst.n_drones << ',' << inst.swap_time << ','
                << inst.swap_time / 60.0 << ',' << inst.battery_capacity << ','
                << CsvEscape(MasterSubtourModeName(options.subtour_mode)) << ','
                << options.time_limit_seconds << ','
                << options.cp_options.time_limit_seconds << ','
                << options.mip_gap << ',' << options.big_m << ','
                << status << ',' << objective << ',' << lower_bound << ','
                << gap << ',' << runtime << ',' << truck_time << ','
                << drone_makespan << ',' << final_makespan << ','
                << truck_customers.size() << ',' << drone_customers.size() << ','
                << CsvEscape(JoinRoute(truck_customers, " ")) << ','
                << CsvEscape(JoinRoute(drone_customers, " ")) << ','
                << CsvEscape(JoinRoute(route, " -> ")) << ','
                << CsvEscape(DroneScheduleString(inst, schedule)) << ','
                << cp_calls << ',' << cache_hits << ',' << lbbd_cuts << ','
                << (certified ? 1 : 0) << '\n';
            std::cout << "Result exported to: " << filename << "\n";
        }

        class MasterCallback : public GRBCallback
        {
        public:
            MasterCallback(const Instance &inst, const MasterOptions &opt,
                           const CPSubproblem &cp,
                           const std::vector<GRBVar> &y,
                           const std::vector<GRBVar> &t,
                           const std::vector<std::vector<GRBVar>> &x,
                           const GRBVar &zD)
                : inst_(inst), opt_(opt), cp_(cp), y_(y), t_(t), x_(x), zD_(zD) {}

            struct Stats
            {
                int64_t cp_calls = 0;
                int64_t cache_hits = 0;
                int64_t cuts = 0;
                bool unresolved = false;
            };

            Stats GetStats() const { return stats_; }

            bool RetrieveSchedule(const std::vector<int> &assignment, CPResult &out) const
            {
                const auto it = cache_.find(AssignmentKey(assignment));
                if (it == cache_.end() || !it->second.schedule.feasible)
                    return false;
                out = it->second.schedule;
                return true;
            }

        protected:
            void callback() override
            {
                if (where != GRB_CB_MIPSOL)
                    return;
                // Gurobi may call callbacks from different threads; keep cache consistent.
                std::lock_guard<std::mutex> lock(mutex_);
                try
                {
                    SeparateIntegerSolution();
                }
                catch (GRBException &e)
                {
                    Fail(std::string("Gurobi callback: ") + e.getMessage());
                }
                catch (const std::exception &e)
                {
                    Fail(std::string("Callback: ") + e.what());
                }
            }

        private:
            struct CacheEntry
            {
                int64_t lb = 0;
                int64_t ub = INF64;
                CPResult schedule;
            };

            const Instance &inst_;
            const MasterOptions &opt_;
            const CPSubproblem &cp_;
            const std::vector<GRBVar> &y_;
            const std::vector<GRBVar> &t_;
            const std::vector<std::vector<GRBVar>> &x_;
            GRBVar zD_;
            std::map<std::string, CacheEntry> cache_;
            Stats stats_;
            mutable std::mutex mutex_;
            double best_valid_ub_ = static_cast<double>(INF64);

            std::string AssignmentKey(const std::vector<int> &y) const
            {
                std::string key;
                key.reserve(inst_.n);
                for (int i = 1; i <= inst_.n; ++i)
                    key.push_back(y[i] ? '1' : '0');
                return key;
            }

            void Fail(const std::string &reason)
            {
                stats_.unresolved = true;
                std::cerr << "[NOT CERTIFIED] " << reason << '\n';
                abort(); // Never silently accept an unresolved CP subproblem.
            }

            // SEC on a cycle Q not connected to depot:
            // sum_{i,j in Q} x_ij <= |Q|-1.
            bool SeparateSubtours(const XMatrix &xb, const std::vector<int> &tb)
            {
                const int N = inst_.NumNodes();
                std::vector<int> next(N, -1);
                for (int i = 0; i < N; ++i)
                    for (int j = 0; j < N; ++j)
                        if (xb[i][j])
                            next[i] = j;
                std::vector<bool> visited(N, false);
                int cur = inst_.start_depot;
                while (cur != -1 && cur != inst_.end_depot && !visited[cur])
                {
                    visited[cur] = true;
                    cur = next[cur];
                }
                if (cur == inst_.end_depot)
                    visited[cur] = true;

                bool found = false;
                for (int i = 1; i <= inst_.n; ++i)
                {
                    if (!tb[i] || visited[i])
                        continue;
                    std::vector<int> cycle;
                    std::set<int> local;
                    cur = i;
                    while (cur != -1 && cur != inst_.start_depot &&
                           cur != inst_.end_depot && !local.count(cur))
                    {
                        local.insert(cur);
                        visited[cur] = true;
                        cycle.push_back(cur);
                        cur = next[cur];
                    }
                    if (cycle.empty())
                        continue;
                    GRBLinExpr lhs = 0.0;
                    for (int a : cycle)
                        for (int b : cycle)
                            if (a != b)
                                lhs += x_[a][b];
                    addLazy(lhs <= static_cast<int>(cycle.size()) - 1);
                    found = true;
                }
                return found;
            }

            void AddLBBDCut(const std::vector<Node> &D, int64_t L, double zD_value)
            {
                if (static_cast<double>(L) <= zD_value + 1e-6)
                    return;
                GRBLinExpr lhs = zD_;
                for (Node i : D)
                {
                    const int64_t delta = std::min(
                        inst_.DroneRoundTripTime(i) + inst_.swap_time, L);
                    lhs += static_cast<double>(delta) * (1.0 - y_[i]);
                }
                addLazy(lhs >= static_cast<double>(L));
                ++stats_.cuts;
                if (opt_.cut_verbose)
                    std::cout << "[LBBD] |D|=" << D.size() << " certified_lb=" << L << '\n';
            }

            static int64_t CeilDivNonnegative(int64_t a, int64_t b)
            {
                return a / b + (a % b != 0);
            }

            // Exact Martello-Toth L2 lower bound for 1D Bin Packing (energy into battery capacity B).
            static int64_t BinPackingL2Bound(const std::vector<int64_t> &items, int64_t B)
            {
                if (items.empty() || B <= 0)
                    return 0;
                int64_t total_e = 0;
                std::vector<int64_t> sorted = items;
                std::sort(sorted.begin(), sorted.end(), std::greater<int64_t>());
                for (int64_t e : sorted)
                    total_e += e;
                int64_t best_c = (total_e + B - 1) / B;

                std::vector<int64_t> alphas = {0};
                for (int64_t e : sorted)
                    if (e <= B / 2)
                        alphas.push_back(e);
                std::sort(alphas.begin(), alphas.end());
                alphas.erase(std::unique(alphas.begin(), alphas.end()), alphas.end());

                for (int64_t alpha : alphas)
                {
                    int64_t n1 = 0, n2 = 0;
                    int64_t sum_n2 = 0, sum_n3 = 0;
                    for (int64_t e : sorted)
                    {
                        if (e > B - alpha)
                            ++n1;
                        else if (e > B / 2)
                        {
                            ++n2;
                            sum_n2 += e;
                        }
                        else if (e >= alpha)
                            sum_n3 += e;
                    }
                    const int64_t waste = n2 * B - sum_n2;
                    const int64_t excess = std::max(int64_t(0), sum_n3 - waste);
                    const int64_t bound = n1 + n2 + (excess + B - 1) / B;
                    if (bound > best_c)
                        best_c = bound;
                }
                return best_c;
            }

            void AddCutoffCut(const std::vector<Node> &D, int64_t cutoff)
            {
                if (cutoff <= 0)
                    return;
                GRBLinExpr missing = 0.0;
                for (Node i : D)
                    missing += (1.0 - y_[i]);
                addLazy(zD_ + static_cast<double>(cutoff) * missing >= static_cast<double>(cutoff));
                ++stats_.cuts;
                if (opt_.cut_verbose)
                    std::cout << "[CUTOFF] |D|=" << D.size() << " cutoff=" << cutoff << '\n';
            }

            void SeparateIntegerSolution()
            {
                const int N = inst_.NumNodes();
                std::vector<int> ybar(N, 0), tbar(N, 0);
                XMatrix xb(N, std::vector<int>(N, 0));
                for (int i = 1; i <= inst_.n; ++i)
                {
                    ybar[i] = getSolution(y_[i]) > 0.5;
                    tbar[i] = getSolution(t_[i]) > 0.5;
                }
                double truck_work = 0.0;
                for (int i = 0; i < N; ++i)
                    for (int j = 0; j < N; ++j)
                        if (TruckArcAllowed(inst_, i, j))
                        {
                            xb[i][j] = getSolution(x_[i][j]) > 0.5;
                            if (xb[i][j])
                                truck_work += static_cast<double>(inst_.truck_time[i][j]);
                        }

                if (opt_.subtour_mode == MasterSubtourMode::LazyCut &&
                    SeparateSubtours(xb, tbar))
                    return;

                std::vector<Node> D;
                for (int i = 1; i <= inst_.n; ++i)
                    if (ybar[i])
                        D.push_back(i);

                const double zD_value = getSolution(zD_);
                // Integer CP makespan; tolerance accounts for Gurobi numerical noise.
                const int64_t T = static_cast<int64_t>(std::floor(zD_value + 1e-6));
                CacheEntry &cache = cache_[AssignmentKey(ybar)];

                int64_t total_r = 0, max_r = 0;
                for (Node i : D)
                {
                    const int64_t r = inst_.DroneRoundTripTime(i);
                    if (r < 0 || total_r > INF64 - r)
                        throw std::runtime_error("Invalid / overflowing drone flight time");
                    total_r += r;
                    max_r = std::max(max_r, r);
                }
                cache.lb = std::max(cache.lb, std::max(max_r,
                                                       CeilDivNonnegative(total_r, inst_.n_drones)));

                // Analytical 1D Bin Packing lower bound on battery swaps:
                if (opt_.use_bpp_lb && !D.empty())
                {
                    std::vector<int64_t> items;
                    items.reserve(D.size());
                    for (Node i : D)
                        items.push_back(inst_.energy[i]);
                    const int64_t C_min = BinPackingL2Bound(items, inst_.battery_capacity);
                    const int64_t Q_min = std::max(int64_t(0), C_min - inst_.n_drones);
                    const int64_t bpp_work = total_r + Q_min * inst_.swap_time;
                    const int64_t bpp_lb = CeilDivNonnegative(bpp_work, inst_.n_drones);
                    cache.lb = std::max(cache.lb, bpp_lb);
                }

                // Construct a physically feasible schedule without calling CP:
                // one drone handles all jobs with a full battery swap between sorties.
                if (cache.ub == INF64)
                {
                    const int64_t swap_count = (D.empty() ? 0 : static_cast<int64_t>(D.size()) - 1);
                    if (inst_.swap_time > 0 &&
                        swap_count > (INF64 - total_r) / inst_.swap_time)
                        throw std::runtime_error("Overflow in simple drone UB");
                    const int64_t ub = total_r + swap_count * inst_.swap_time;
                    cache.ub = ub;
                    cache.schedule.status = CPStatus::Feasible;
                    cache.schedule.feasible = true;
                    cache.schedule.objective = ub;
                    if (!D.empty())
                    {
                        cache.schedule.chains.push_back(D);
                        std::vector<int> swaps(D.size(), 1);
                        swaps[0] = 0;
                        cache.schedule.swap_before.push_back(std::move(swaps));
                    }
                }

                const double sol_makespan = std::max(truck_work, static_cast<double>(cache.ub));
                if (sol_makespan < best_valid_ub_)
                {
                    best_valid_ub_ = sol_makespan;
                    if (opt_.verbose)
                        std::cout << "[INCUMBENT] Initial/updated valid UB: " << best_valid_ub_ << '\n';
                }

                if (cache.lb > T)
                {
                    ++stats_.cache_hits;
                    AddLBBDCut(D, cache.lb, zD_value);
                    return;
                }
                if (cache.ub <= T)
                {
                    ++stats_.cache_hits;
                    return; // Already have a concrete feasible drone schedule.
                }

                // Two-Phase Incumbent Cutoff strategy:
                if (opt_.use_incumbent_cutoff && best_valid_ub_ < static_cast<double>(INF64))
                {
                    const int64_t cutoff = static_cast<int64_t>(std::floor(best_valid_ub_ - 1.0 + 1e-9));
                    if (cutoff > 0 && cutoff < cache.lb)
                    {
                        AddCutoffCut(D, static_cast<int64_t>(std::ceil(best_valid_ub_)));
                        return;
                    }

                    if (cutoff > 0)
                    {
                        // Phase 1: Search for any schedule with makespan <= cutoff (bounded budget)
                        CPOptions p1_cfg = opt_.cp_options;
                        p1_cfg.time_limit_seconds = std::min(2.0, std::max(0.5, opt_.cp_options.time_limit_seconds));
                        p1_cfg.stop_after_first_solution = true;
                        ++stats_.cp_calls;
                        CPResult r1 = cp_.SolveDroneSubproblem(D, p1_cfg, cutoff);
                        if (r1.status == CPStatus::Infeasible)
                        {
                            cache.lb = std::max(cache.lb, cutoff + 1);
                            AddCutoffCut(D, static_cast<int64_t>(std::ceil(best_valid_ub_)));
                            return;
                        }
                        if (r1.feasible && r1.objective <= cutoff)
                        {
                            if (r1.objective < cache.ub)
                            {
                                cache.ub = r1.objective;
                                cache.schedule = std::move(r1);
                            }
                            const double new_sol = std::max(truck_work, static_cast<double>(cache.ub));
                            if (new_sol < best_valid_ub_)
                            {
                                best_valid_ub_ = new_sol;
                                if (opt_.verbose)
                                    std::cout << "[INCUMBENT-P1] Improved valid UB: " << best_valid_ub_ << '\n';
                            }
                            if (cache.ub <= T)
                                return; // Fully verified under candidate zD!

                            if (cache.lb > T)
                            {
                                AddLBBDCut(D, cache.lb, zD_value);
                                return;
                            }
                            AddCutoffCut(D, std::min(cache.ub, static_cast<int64_t>(T + 1)));
                            return;
                        }
                    }
                }

                // Adaptive exact decision calls for T with short budgets:
                const std::vector<double> budgets = {
                    0.2, 1.0, std::min(2.0, opt_.cp_options.time_limit_seconds)};
                for (double seconds : budgets)
                {
                    if (seconds <= 0)
                        continue;
                    CPOptions cfg = opt_.cp_options;
                    cfg.time_limit_seconds = seconds;
                    cfg.stop_after_first_solution = false;
                    ++stats_.cp_calls;
                    CPResult r = cp_.SolveDroneSubproblem(D, cfg, T);
                    if (r.status == CPStatus::ModelInvalid)
                    {
                        Fail("CP model invalid");
                        return;
                    }
                    if (r.status == CPStatus::Infeasible)
                    {
                        cache.lb = std::max(cache.lb, T + 1);
                        AddLBBDCut(D, cache.lb, zD_value);
                        return;
                    }
                    if (r.feasible && r.objective <= T)
                    {
                        if (r.objective < cache.ub)
                        {
                            cache.ub = r.objective;
                            cache.schedule = std::move(r);
                        }
                        return;
                    }
                }

                // Optimization fallback (capped at 2.0s):
                CPOptions cfg = opt_.cp_options;
                cfg.time_limit_seconds = std::min(2.0, std::max(1.0, cfg.time_limit_seconds));
                cfg.stop_after_first_solution = false;
                ++stats_.cp_calls;
                CPResult opt = cp_.SolveDroneSubproblem(D, cfg);
                if (opt.status != CPStatus::ModelInvalid && opt.status != CPStatus::Infeasible)
                {
                    cache.lb = std::max(cache.lb, opt.lower_bound);
                    if (opt.feasible && opt.objective < cache.ub)
                    {
                        cache.ub = opt.objective;
                        cache.schedule = std::move(opt);
                    }
                }
                if (cache.lb > T)
                {
                    AddLBBDCut(D, cache.lb, zD_value);
                    return;
                }
                if (cache.ub <= T)
                    return;

                // CP inconclusive within time budget: cut off this candidate to prevent solver stall.
                AddCutoffCut(D, static_cast<int64_t>(T + 1));
            }
        };

    } // namespace

    MasterProblem::MasterProblem(const Instance &inst, const MasterOptions &options)
        : inst_(inst), options_(options) {}

    void MasterProblem::Solve()
    {
        const int N = inst_.NumNodes();
        const int depot = inst_.start_depot, finish = inst_.end_depot;
        if (inst_.n_drones < 1 || inst_.battery_capacity <= 0 || inst_.swap_time < 0)
            throw std::runtime_error("Invalid K, battery capacity or swap time");

        GRBEnv env(true);
        env.set(GRB_IntParam_OutputFlag, options_.verbose ? 1 : 0);
        env.start();
        GRBModel model(env);
        model.set(GRB_StringAttr_ModelName, "PDSTSP_BS_certified_branch_and_check");
        model.set(GRB_IntParam_LazyConstraints, 1);
        if (options_.time_limit_seconds > 0)
            model.set(GRB_DoubleParam_TimeLimit, options_.time_limit_seconds);
        if (options_.mip_gap > 0)
            model.set(GRB_DoubleParam_MIPGap, options_.mip_gap);

        std::vector<GRBVar> y(N), t(N);
        std::vector<std::vector<GRBVar>> x(N, std::vector<GRBVar>(N));
        for (int i = 1; i <= inst_.n; ++i)
        {
            y[i] = model.addVar(0, 1, 0, GRB_BINARY, "y_" + std::to_string(i));
            t[i] = model.addVar(0, 1, 0, GRB_BINARY, "t_" + std::to_string(i));
        }
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                if (TruckArcAllowed(inst_, i, j))
                    x[i][j] = model.addVar(0, 1, 0, GRB_BINARY,
                                           "x_" + std::to_string(i) + "_" + std::to_string(j));
        // A feasible schedule always exists using at most n-min(K,n) swaps:
        // give each drone one initial battery cycle and swap before each
        // additional sortie. q is a lower-bound auxiliary, so this upper
        // bound preserves all assignments while avoiding an unbounded q.
        const int max_swaps = std::max(0, inst_.n - std::min(inst_.n_drones, inst_.n));
        GRBVar q = model.addVar(0, max_swaps, 0, GRB_INTEGER, "q_swap_lb");
        GRBVar zT = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "z_truck");
        GRBVar zD = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "z_drone");
        GRBVar z = model.addVar(0, GRB_INFINITY, 1, GRB_CONTINUOUS, "z");

        std::vector<GRBVar> ord(N);
        if (options_.subtour_mode == MasterSubtourMode::MTZ)
            for (int i = 1; i <= inst_.n; ++i)
                ord[i] = model.addVar(0, inst_.n, 0, GRB_CONTINUOUS,
                                      "ord_" + std::to_string(i));
        model.update();

        for (int i = 1; i <= inst_.n; ++i)
        {
            if (inst_.drone_feasible[i])
                model.addConstr(y[i] + t[i] == 1.0, "assign_" + std::to_string(i));
            else
            {
                model.addConstr(y[i] == 0.0, "force_truck_drone_off_" + std::to_string(i));
                model.addConstr(t[i] == 1.0, "force_truck_" + std::to_string(i));
            }
        }

        GRBLinExpr out_depot = 0, in_finish = 0;
        for (int j = 0; j < N; ++j)
            if (TruckArcAllowed(inst_, depot, j))
                out_depot += x[depot][j];
        for (int i = 0; i < N; ++i)
            if (TruckArcAllowed(inst_, i, finish))
                in_finish += x[i][finish];
        model.addConstr(out_depot == 1.0, "depot_out");
        model.addConstr(in_finish == 1.0, "finish_in");

        for (int i = 1; i <= inst_.n; ++i)
        {
            GRBLinExpr in = 0, out = 0;
            for (int j = 0; j < N; ++j)
            {
                if (TruckArcAllowed(inst_, j, i))
                    in += x[j][i];
                if (TruckArcAllowed(inst_, i, j))
                    out += x[i][j];
            }
            model.addConstr(in == t[i], "truck_in_" + std::to_string(i));
            model.addConstr(out == t[i], "truck_out_" + std::to_string(i));
        }

        if (options_.subtour_mode == MasterSubtourMode::MTZ)
        {
            // Big-M=n+1 is needed: an unvisited j has ord_j=0 and a visited
            // predecessor i may have ord_i=n when x_ij=0.
            const double M = static_cast<double>(inst_.n + 1);
            for (int i = 1; i <= inst_.n; ++i)
            {
                model.addConstr(ord[i] <= inst_.n * t[i],
                                "ord_ub_" + std::to_string(i));
                model.addConstr(ord[i] >= t[i],
                                "ord_lb_" + std::to_string(i));
            }
            for (int i = 1; i <= inst_.n; ++i)
                for (int j = 1; j <= inst_.n; ++j)
                    if (i != j)
                        model.addConstr(ord[j] >= ord[i] + 1.0 - M * (1.0 - x[i][j]),
                                        "mtz_" + std::to_string(i) + "_" + std::to_string(j));
        }

        GRBLinExpr truck_work = 0, drone_work = 0, energy_work = 0;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                if (TruckArcAllowed(inst_, i, j))
                    truck_work += static_cast<double>(inst_.truck_time[i][j]) * x[i][j];
        model.addConstr(zT >= truck_work, "truck_time_lb");

        for (int i = 1; i <= inst_.n; ++i)
        {
            if (!inst_.drone_feasible[i])
                continue;
            const double r = static_cast<double>(inst_.DroneRoundTripTime(i));
            drone_work += r * y[i];
            energy_work += static_cast<double>(inst_.energy[i]) * y[i];
            model.addConstr(zD >= r * y[i], "single_drone_lb_" + std::to_string(i));
        }
        model.addConstr(static_cast<double>(inst_.n_drones) * zD >=
                            drone_work + static_cast<double>(inst_.swap_time) * q,
                        "total_drone_work_lb");
        model.addConstr(static_cast<double>(inst_.battery_capacity) *
                                (static_cast<double>(inst_.n_drones) + q) >=
                            energy_work,
                        "battery_energy_lb");

        // Valid packing inequalities: jobs with e_i>B/(p+1) consume at least
        // one of p slots available per battery cycle, p=1,2,3.
        for (int p = 1; p <= 3; ++p)
        {
            GRBLinExpr heavy = 0;
            for (int i = 1; i <= inst_.n; ++i)
            {
                if (inst_.drone_feasible[i] &&
                    static_cast<int64_t>(p + 1) * inst_.energy[i] > inst_.battery_capacity)
                    heavy += y[i];
            }
            model.addConstr(static_cast<double>(p) *
                                    (static_cast<double>(inst_.n_drones) + q) >=
                                heavy,
                            "battery_cycle_p" + std::to_string(p));
        }

        model.addConstr(z >= zT, "z_ge_truck");
        model.addConstr(z >= zD, "z_ge_drone");
        model.setObjective(GRBLinExpr(z), GRB_MINIMIZE);

        CPSubproblem cp(inst_);
        MasterCallback cb(inst_, options_, cp, y, t, x, zD);
        model.setCallback(&cb);
        model.optimize();

        const MasterCallback::Stats stats = cb.GetStats();
        std::cout << "[LBBD STATS] cp_calls=" << stats.cp_calls
                  << " cache_hits=" << stats.cache_hits
                  << " cuts=" << stats.cuts
                  << " unresolved=" << stats.unresolved << '\n';
        if (stats.unresolved)
        {
            std::cerr << "[NOT CERTIFIED] CP or callback inconclusive; "
                         "no final CSV will be exported.\n";
            return;
        }

        const int status = model.get(GRB_IntAttr_Status);
        const int sol_count = model.get(GRB_IntAttr_SolCount);
        const double runtime = model.get(GRB_DoubleAttr_Runtime);
        const double lower_bound = model.get(GRB_DoubleAttr_ObjBound);
        std::cout << "\n=== Master solve finished ===\nGurobi status: " << status
                  << "\nLB: " << lower_bound << "\n";

        if (sol_count == 0)
        {
            CPResult empty;
            WriteCsv(options_.result_file, inst_, options_, status,
                     -1, lower_bound, -1, runtime, -1, -1, -1,
                     {}, {}, {}, empty, stats.cp_calls, stats.cache_hits,
                     stats.cuts, false);
            return;
        }

        std::vector<int> ysol(N, 0), tsol(N, 0);
        XMatrix xsol(N, std::vector<int>(N, 0));
        for (int i = 1; i <= inst_.n; ++i)
        {
            ysol[i] = y[i].get(GRB_DoubleAttr_X) > 0.5;
            tsol[i] = t[i].get(GRB_DoubleAttr_X) > 0.5;
        }
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                if (TruckArcAllowed(inst_, i, j))
                    xsol[i][j] = x[i][j].get(GRB_DoubleAttr_X) > 0.5;

        CPResult schedule;
        if (!cb.RetrieveSchedule(ysol, schedule) ||
            !schedule.feasible ||
            schedule.objective > zD.get(GRB_DoubleAttr_X) + 1e-6)
        {
            std::cerr << "[NOT CERTIFIED] Final incumbent has no certified "
                         "drone schedule under its z_D.\n";
            return;
        }

        const std::vector<Node> route = DecodeTruckRoute(inst_, xsol);
        std::vector<Node> D, T;
        for (int i = 1; i <= inst_.n; ++i)
        {
            if (ysol[i])
                D.push_back(i);
            else
                T.push_back(i);
        }
        // Check every truck-assigned customer appears exactly once.
        std::set<Node> served(route.begin(), route.end());
        if (served.size() != route.size() ||
            route.size() != T.size() + 2)
        {
            std::cerr << "[NOT CERTIFIED] Truck route decoding failed.\n";
            return;
        }
        for (int i : T)
            if (!served.count(i))
            {
                std::cerr << "[NOT CERTIFIED] Truck customer missing: " << i << '\n';
                return;
            }

        const double tt = static_cast<double>(ComputeTruckTime(inst_, xsol));
        const double dt = static_cast<double>(schedule.objective);
        const double achieved = std::max(tt, dt);
        const double obj = model.get(GRB_DoubleAttr_ObjVal);
        if (achieved > obj + 1e-6)
        {
            std::cerr << "[NOT CERTIFIED] Decoded schedule exceeds Master objective.\n";
            return;
        }

        double gap = -1;
        try
        {
            gap = model.get(GRB_DoubleAttr_MIPGap);
        }
        catch (const GRBException &)
        {
            gap = -1;
        }
        std::cout << "Best master objective: " << obj
                  << "\nDecoded makespan: " << achieved
                  << "\nTruck route: " << JoinRoute(route, " -> ")
                  << "\nDrone schedule: " << DroneScheduleString(inst_, schedule)
                  << "\nGurobi status: " << status
                  << " (only OPTIMAL with zero gap proves full optimality)\n";
        WriteCsv(options_.result_file, inst_, options_, status, obj,
                 lower_bound, gap, runtime, tt, dt, achieved, T, D,
                 route, schedule, stats.cp_calls, stats.cache_hits,
                 stats.cuts, true);
    }

} // namespace bbcp
