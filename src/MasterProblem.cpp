#include "MasterProblem.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>

#include "gurobi_c++.h"

namespace bbcp {

MasterSubtourMode ParseMasterSubtourMode(const std::string& mode_raw) {
    std::string mode = mode_raw;
    for (char& c : mode) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }

    if (mode == "cut" || mode == "lazy" || mode == "lazycut" || mode == "branch-and-cut") {
        return MasterSubtourMode::LazyCut;
    }

    if (mode == "mtz") {
        return MasterSubtourMode::MTZ;
    }

    throw std::runtime_error(
        "Unknown master mode: " + mode_raw + ". Use 'cut' or 'mtz'."
    );
}

std::string MasterSubtourModeName(MasterSubtourMode mode) {
    switch (mode) {
        case MasterSubtourMode::LazyCut:
            return "cut";
        case MasterSubtourMode::MTZ:
            return "mtz";
    }
    return "unknown";
}

namespace {

struct Subtour {
    std::vector<Node> nodes;
};

bool IsMasterTruckArcAllowed(const Instance& inst, int i, int j) {
    if (i == j) return false;
    if (i == inst.end_depot) return false;
    if (j == inst.start_depot) return false;
    return true;
}

std::vector<Node> DecodeTruckRouteFromX(
    const Instance& inst,
    const std::vector<std::vector<int>>& xsol
) {
    const int N = inst.NumNodes();
    const int start = inst.start_depot;
    const int end = inst.end_depot;

    std::vector<Node> route;
    std::vector<bool> visited(N, false);

    int cur = start;
    route.push_back(cur);
    visited[cur] = true;

    while (cur != end) {
        int nxt = -1;

        for (int j = 0; j < N; ++j) {
            if (xsol[cur][j] == 1) {
                nxt = j;
                break;
            }
        }

        if (nxt == -1) {
            std::cerr << "[WARN] Cannot decode truck route: no successor after node "
                      << cur << "\n";
            break;
        }

        if (nxt != end && visited[nxt]) {
            std::cerr << "[WARN] Cycle detected while decoding truck route at node "
                      << nxt << "\n";
            route.push_back(nxt);
            break;
        }

        route.push_back(nxt);
        visited[nxt] = true;
        cur = nxt;
    }

    return route;
}

void PrintRoute(const std::string& label, const std::vector<Node>& route) {
    std::cout << label;

    if (route.empty()) {
        std::cout << "(empty)\n";
        return;
    }

    for (size_t k = 0; k < route.size(); ++k) {
        if (k > 0) std::cout << " -> ";
        std::cout << route[k];
    }

    std::cout << "\n";
}

void PrintDroneChainsAsTours(const Instance& inst, const CPResult& cp_res) {
    std::cout << std::endl;
    std::cout << "Drone schedules:" << std::endl;

    for (size_t k = 0; k < cp_res.chains.size(); ++k) {
        const std::vector<Node>& chain = cp_res.chains[k];

        std::cout << "  Drone " << k << ": ";

        if (chain.empty()) {
            std::cout << "(not used)" << std::endl;
            continue;
        }

        const std::vector<int>* swaps = nullptr;
        if (k < cp_res.swap_before.size()) {
            swaps = &cp_res.swap_before[k];
        }

        std::vector<Node> swap_nodes;

        for (size_t p = 0; p < chain.size(); ++p) {
            const bool has_swap =
                (swaps != nullptr && p < swaps->size() && (*swaps)[p] == 1);

            if (p > 0) {
                std::cout << " | ";
            }

            if (has_swap) {
                std::cout << "[SWAP] ";
                swap_nodes.push_back(chain[p]);
            }

            std::cout << inst.start_depot
                      << " -> " << chain[p]
                      << " -> " << inst.start_depot;
        }

        std::cout << std::endl;

        std::cout << "           ";
        if (!swap_nodes.empty()) {
            std::cout << "swaps before customer(s): ";
            for (size_t idx = 0; idx < swap_nodes.size(); ++idx) {
                if (idx > 0) {
                    std::cout << ", ";
                }
                std::cout << swap_nodes[idx];
            }
        } else {
            std::cout << "swaps: none";
        }

        std::cout << std::endl;
    }

    for (int k = static_cast<int>(cp_res.chains.size()); k < inst.n_drones; ++k) {
        std::cout << "  Drone " << k << ": (not used)" << std::endl;
        std::cout << "           swaps: none" << std::endl;
    }

    std::cout << std::endl;
}


std::string CsvEscape(const std::string& s) {
    std::string out = "\"";

    for (char c : s) {
        if (c == '"') {
            out += "\"\"";
        } else {
            out += c;
        }
    }

    out += "\"";
    return out;
}

std::string JoinRouteString(const std::vector<Node>& route) {
    std::ostringstream oss;

    for (size_t i = 0; i < route.size(); ++i) {
        if (i > 0) {
            oss << " -> ";
        }
        oss << route[i];
    }

    return oss.str();
}

std::string JoinNodeListString(const std::vector<Node>& nodes) {
    std::ostringstream oss;

    for (size_t i = 0; i < nodes.size(); ++i) {
        if (i > 0) {
            oss << " ";
        }
        oss << nodes[i];
    }

    return oss.str();
}

std::string DroneSchedulesString(const Instance& inst, const CPResult& cp_res) {
    std::ostringstream oss;

    for (size_t k = 0; k < cp_res.chains.size(); ++k) {
        if (k > 0) {
            oss << " || ";
        }

        oss << "Drone " << k << ": ";

        const std::vector<Node>& chain = cp_res.chains[k];

        if (chain.empty()) {
            oss << "(not used)";
            continue;
        }

        const std::vector<int>* swaps = nullptr;
        if (k < cp_res.swap_before.size()) {
            swaps = &cp_res.swap_before[k];
        }

        for (size_t p = 0; p < chain.size(); ++p) {
            const bool has_swap =
                (swaps != nullptr && p < swaps->size() && (*swaps)[p] == 1);

            if (p > 0) {
                oss << " | ";
            }

            if (has_swap) {
                oss << "[SWAP] ";
            }

            oss << inst.start_depot
                << " -> " << chain[p]
                << " -> " << inst.start_depot;
        }
    }

    for (int k = static_cast<int>(cp_res.chains.size()); k < inst.n_drones; ++k) {
        if (!oss.str().empty()) {
            oss << " || ";
        }
        oss << "Drone " << k << ": (not used)";
    }

    return oss.str();
}

double ComputeTruckTimeFromX(
    const Instance& inst,
    const std::vector<std::vector<int>>& xsol
) {
    double value = 0.0;
    const int N = inst.NumNodes();

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (xsol[i][j] == 1) {
                value += static_cast<double>(inst.truck_time[i][j]);
            }
        }
    }

    return value;
}


bool CsvFileNeedsHeader(const std::string& filepath) {
    std::ifstream fin(filepath);

    if (!fin.good()) {
        return true;  // file does not exist or cannot be opened for reading
    }

    return fin.peek() == std::ifstream::traits_type::eof();  // empty file
}

void WriteResultHeader(std::ofstream& fout) {
    fout << "instance,"
         << "n_customers,"
         << "n_drones,"
         << "swap_time_sec,"
         << "swap_time_min,"
         << "battery_capacity,"
         << "master_mode,"
         << "master_time_limit_sec,"
         << "cp_time_limit_sec,"
         << "mip_gap_param,"
         << "big_m,"
         << "status,"
         << "obj,"
         << "lower_bound,"
         << "mip_gap,"
         << "runtime_sec,"
         << "truck_time,"
         << "drone_makespan,"
         << "final_makespan,"
         << "num_truck_customers,"
         << "num_drone_customers,"
         << "truck_customers,"
         << "drone_customers,"
         << "truck_route,"
         << "drone_schedules\n";
}

void ExportFinalResultCSV(
    const std::string& filepath,
    const MasterOptions& options,
    const Instance& inst,
    int gurobi_status,
    double obj,
    double lower_bound,
    double mip_gap,
    double runtime_sec,
    double truck_time,
    double drone_makespan,
    double final_makespan,
    const std::vector<Node>& truck_customers,
    const std::vector<Node>& drone_customers,
    const std::vector<Node>& truck_route,
    const CPResult& final_drone
) {
    if (filepath.empty()) {
        return;
    }

    const bool need_header = CsvFileNeedsHeader(filepath);

    std::ofstream fout(filepath, std::ios::app);
    if (!fout.is_open()) {
        std::cerr << "[WARN] Cannot open result file: " << filepath << "\n";
        return;
    }

    fout << std::fixed << std::setprecision(6);

    if (need_header) {
        WriteResultHeader(fout);
    }

    fout << CsvEscape(options.instance_name) << ","
         << inst.n << ","
         << inst.n_drones << ","
         << inst.swap_time << ","
         << static_cast<double>(inst.swap_time) / 60.0 << ","
         << inst.battery_capacity << ","
         << CsvEscape(MasterSubtourModeName(options.subtour_mode)) << ","
         << options.time_limit_seconds << ","
         << options.cp_options.time_limit_seconds << ","
         << options.mip_gap << ","
         << options.big_m << ","
         << gurobi_status << ","
         << obj << ","
         << lower_bound << ","
         << mip_gap << ","
         << runtime_sec << ","
         << truck_time << ","
         << drone_makespan << ","
         << final_makespan << ","
         << truck_customers.size() << ","
         << drone_customers.size() << ","
         << CsvEscape(JoinNodeListString(truck_customers)) << ","
         << CsvEscape(JoinNodeListString(drone_customers)) << ","
         << CsvEscape(JoinRouteString(truck_route)) << ","
         << CsvEscape(DroneSchedulesString(inst, final_drone))
         << "\n";

    fout.close();

    std::cout << "Final result exported to: " << filepath << "\n";
}

void ExportNoSolutionCSV(
    const std::string& filepath,
    const MasterOptions& options,
    const Instance& inst,
    int gurobi_status,
    double lower_bound,
    double runtime_sec
) {
    if (filepath.empty()) {
        return;
    }

    const bool need_header = CsvFileNeedsHeader(filepath);

    std::ofstream fout(filepath, std::ios::app);
    if (!fout.is_open()) {
        std::cerr << "[WARN] Cannot open result file: " << filepath << "\n";
        return;
    }

    fout << std::fixed << std::setprecision(6);

    if (need_header) {
        WriteResultHeader(fout);
    }

    fout << CsvEscape(options.instance_name) << ","
         << inst.n << ","
         << inst.n_drones << ","
         << inst.swap_time << ","
         << static_cast<double>(inst.swap_time) / 60.0 << ","
         << inst.battery_capacity << ","
         << CsvEscape(MasterSubtourModeName(options.subtour_mode)) << ","
         << options.time_limit_seconds << ","
         << options.cp_options.time_limit_seconds << ","
         << options.mip_gap << ","
         << options.big_m << ","
         << gurobi_status << ","
         << -1.0 << ","
         << lower_bound << ","
         << -1.0 << ","
         << runtime_sec << ","
         << -1.0 << ","
         << -1.0 << ","
         << -1.0 << ","
         << 0 << ","
         << 0 << ","
         << "\"\",\"\",\"\",\"\""
         << "\n";

    fout.close();

    std::cout << "No-solution result exported to: " << filepath << "\n";
}


class MasterCallback : public GRBCallback {
public:
    MasterCallback(const Instance& inst,
                   const MasterOptions& options,
                   const CPSubproblem& cp,
                   const std::vector<GRBVar>& y,
                   const std::vector<GRBVar>& t,
                   const std::vector<std::vector<GRBVar>>& x,
                   const GRBVar& z)
        : inst_(inst),
          options_(options),
          cp_(cp),
          y_(y),
          t_(t),
          x_(x),
          z_(z) {}

protected:
    void callback() override {
        if (where != GRB_CB_MIPSOL) return;

        try {
            HandleIntegerSolution();
        } catch (GRBException& e) {
            std::cerr << "Gurobi callback error: " << e.getMessage() << "\n";
        } catch (std::exception& e) {
            std::cerr << "Callback error: " << e.what() << "\n";
        }
    }

private:
    const Instance& inst_;
    MasterOptions options_;
    const CPSubproblem& cp_;

    const std::vector<GRBVar>& y_;
    const std::vector<GRBVar>& t_;
    const std::vector<std::vector<GRBVar>>& x_;
    GRBVar z_;

    bool IsTruckArcAllowed(int i, int j) const {
        return IsMasterTruckArcAllowed(inst_, i, j);
    }

    void HandleIntegerSolution() {
        const int N = inst_.NumNodes();

        std::vector<int> ybar(N, 0);
        std::vector<int> tbar(N, 0);
        std::vector<std::vector<int>> xbar(N, std::vector<int>(N, 0));

        for (int i = 1; i <= inst_.n; ++i) {
            ybar[i] = (getSolution(y_[i]) > 0.5 ? 1 : 0);
            tbar[i] = (getSolution(t_[i]) > 0.5 ? 1 : 0);
        }

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (!IsTruckArcAllowed(i, j)) continue;
                xbar[i][j] = (getSolution(x_[i][j]) > 0.5 ? 1 : 0);
            }
        }

        // In the lazy-cut master, the callback separates truck subtours.
        // In the MTZ master, subtours are already eliminated by constraints,
        // so this block is skipped.
        if (options_.subtour_mode == MasterSubtourMode::LazyCut) {
            std::vector<Subtour> subtours = FindTruckSubtours(xbar, tbar);
            if (!subtours.empty()) {
                for (const Subtour& st : subtours) {
                    AddSubtourCut(st);
                }
                return;
            }
        }

        std::vector<Node> drone_customers;
        for (int i = 1; i <= inst_.n; ++i) {
            if (ybar[i] == 1) {
                drone_customers.push_back(i);
            }
        }

        CPResult cp_res = cp_.SolveDroneSubproblem(
            drone_customers,
            options_.cp_options
        );

        if (!cp_res.feasible) {
            AddDroneNoGoodCut(drone_customers);
            return;
        }

        const double truck_value = ComputeTruckValue(xbar);
        const double phi = std::max(
            truck_value,
            static_cast<double>(cp_res.objective)
        );
        const double master_z = getSolution(z_);

        if (options_.verbose) {
            std::cout << "[MIPSOL] master_z=" << master_z
                      << ", truck=" << truck_value
                      << ", cp_drone=" << cp_res.objective
                      << ", phi=" << phi
                      << ", drone_set_size=" << drone_customers.size()
                      << "\n";
        }

        if (master_z + 1e-6 < phi) {
            AddOptimalityCut(ybar, phi);
        }
    }

    std::vector<Subtour> FindTruckSubtours(
        const std::vector<std::vector<int>>& xbar,
        const std::vector<int>& tbar
    ) const {
        const int N = inst_.NumNodes();
        const int start = inst_.start_depot;
        const int end = inst_.end_depot;

        std::vector<int> succ(N, -1);

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (xbar[i][j] == 1) {
                    succ[i] = j;
                    break;
                }
            }
        }

        std::vector<bool> visited(N, false);

        int cur = start;
        while (cur != -1 && cur != end && !visited[cur]) {
            visited[cur] = true;
            cur = succ[cur];
        }

        if (cur == end) {
            visited[end] = true;
        }

        std::vector<Subtour> subtours;

        for (int i = 1; i <= inst_.n; ++i) {
            if (tbar[i] == 0 || visited[i]) continue;

            Subtour st;
            int v = i;
            std::set<int> local;

            while (v != -1 && !local.count(v) && v != start && v != end) {
                local.insert(v);
                st.nodes.push_back(v);
                visited[v] = true;
                v = succ[v];
            }

            if (!st.nodes.empty()) {
                subtours.push_back(st);
            }
        }

        return subtours;
    }

    void AddSubtourCut(const Subtour& st) {
        if (st.nodes.empty()) return;

        GRBLinExpr lhs = 0.0;

        for (Node i : st.nodes) {
            for (Node j : st.nodes) {
                if (i == j) continue;
                lhs += x_[i][j];
            }
        }

        addLazy(lhs <= static_cast<int>(st.nodes.size()) - 1);

        if (options_.cut_verbose) {
            std::cout << "[CUT] subtour size=" << st.nodes.size() << "\n";
        }
    }

    void AddDroneNoGoodCut(const std::vector<Node>& drone_customers) {
        if (drone_customers.empty()) return;

        GRBLinExpr lhs = 0.0;

        for (Node i : drone_customers) {
            lhs += y_[i];
        }

        addLazy(lhs <= static_cast<int>(drone_customers.size()) - 1);

        if (options_.cut_verbose) {
            std::cout << "[CUT] drone no-good size=" << drone_customers.size() << "\n";
        }
    }

    void AddOptimalityCut(const std::vector<int>& ybar, double phi) {
        GRBLinExpr hamming = 0.0;

        for (int i = 1; i <= inst_.n; ++i) {
            if (!inst_.drone_feasible[i]) continue;

            if (ybar[i] == 1) {
                hamming += (1.0 - y_[i]);
            } else {
                hamming += y_[i];
            }
        }

        GRBLinExpr lhs = 0.0;
        lhs += z_;
        lhs += options_.big_m * hamming;

        addLazy(lhs >= phi);

        if (options_.cut_verbose) {
            std::cout << "[CUT] optimality phi=" << phi << "\n";
        }
    }

    double ComputeTruckValue(
        const std::vector<std::vector<int>>& xbar
    ) const {
        double value = 0.0;
        const int N = inst_.NumNodes();

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (xbar[i][j] == 1) {
                    value += static_cast<double>(inst_.truck_time[i][j]);
                }
            }
        }

        return value;
    }
};

}  // namespace

MasterProblem::MasterProblem(const Instance& inst, const MasterOptions& options)
    : inst_(inst), options_(options) {}

void MasterProblem::Solve() {
    const int N = inst_.NumNodes();
    const int start = inst_.start_depot;
    const int end = inst_.end_depot;

    if (options_.verbose) {
        std::cout << "Master subtour mode: "
                  << MasterSubtourModeName(options_.subtour_mode)
                  << "\n";
    }

    GRBEnv env(true);
    env.set(GRB_IntParam_OutputFlag, options_.verbose ? 1 : 0);
    env.start();

    GRBModel model(env);
    model.set(GRB_StringAttr_ModelName, "MILP_master_CP_subproblem");

    // Lazy constraints are needed for:
    //   - subtour cuts in LazyCut mode,
    //   - CP no-good cuts and optimality cuts in both modes.
    model.set(GRB_IntParam_LazyConstraints, 1);

    if (options_.time_limit_seconds > 0.0) {
        model.set(GRB_DoubleParam_TimeLimit, options_.time_limit_seconds);
    }

    if (options_.mip_gap > 0.0) {
        model.set(GRB_DoubleParam_MIPGap, options_.mip_gap);
    }

    std::vector<GRBVar> y(N);
    std::vector<GRBVar> t(N);

    for (int i = 1; i <= inst_.n; ++i) {
        y[i] = model.addVar(
            0.0,
            1.0,
            0.0,
            GRB_BINARY,
            "y_" + std::to_string(i)
        );

        t[i] = model.addVar(
            0.0,
            1.0,
            0.0,
            GRB_BINARY,
            "t_" + std::to_string(i)
        );
    }

    std::vector<std::vector<GRBVar>> x(N, std::vector<GRBVar>(N));

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (!IsMasterTruckArcAllowed(inst_, i, j)) continue;

            x[i][j] = model.addVar(
                0.0,
                1.0,
                0.0,
                GRB_BINARY,
                "x_" + std::to_string(i) + "_" + std::to_string(j)
            );
        }
    }

    GRBVar q = model.addVar(
        0.0,
        GRB_INFINITY,
        0.0,
        GRB_INTEGER,
        "q_swap_lb"
    );

    GRBVar zT = model.addVar(
        0.0,
        GRB_INFINITY,
        0.0,
        GRB_CONTINUOUS,
        "z_truck"
    );

    GRBVar zD = model.addVar(
        0.0,
        GRB_INFINITY,
        0.0,
        GRB_CONTINUOUS,
        "z_drone"
    );

    GRBVar z = model.addVar(
        0.0,
        GRB_INFINITY,
        1.0,
        GRB_CONTINUOUS,
        "z"
    );

    // MTZ order variable for truck path.
    // It is only used in MTZ mode.
    std::vector<GRBVar> ord(N);
    if (options_.subtour_mode == MasterSubtourMode::MTZ) {
        for (int i = 1; i <= inst_.n; ++i) {
            ord[i] = model.addVar(
                0.0,
                static_cast<double>(inst_.n),
                0.0,
                GRB_CONTINUOUS,
                "ord_" + std::to_string(i)
            );
        }
    }

    model.update();

    // Assignment constraints.
    for (int i = 1; i <= inst_.n; ++i) {
        if (inst_.drone_feasible[i]) {
            model.addConstr(
                y[i] + t[i] == 1.0,
                "assign_" + std::to_string(i)
            );
        } else {
            model.addConstr(
                y[i] == 0.0,
                "force_not_drone_" + std::to_string(i)
            );

            model.addConstr(
                t[i] == 1.0,
                "force_truck_" + std::to_string(i)
            );
        }
    }

    // Start depot: exactly one outgoing truck arc.
    {
        GRBLinExpr out_start = 0.0;

        for (int j = 0; j < N; ++j) {
            if (!IsMasterTruckArcAllowed(inst_, start, j)) continue;
            out_start += x[start][j];
        }

        model.addConstr(out_start == 1.0, "start_out");
    }

    // End depot: exactly one incoming truck arc.
    {
        GRBLinExpr in_end = 0.0;

        for (int i = 0; i < N; ++i) {
            if (!IsMasterTruckArcAllowed(inst_, i, end)) continue;
            in_end += x[i][end];
        }

        model.addConstr(in_end == 1.0, "end_in");
    }

    // Customer degree constraints.
    for (int i = 1; i <= inst_.n; ++i) {
        GRBLinExpr out_i = 0.0;
        GRBLinExpr in_i = 0.0;

        for (int j = 0; j < N; ++j) {
            if (IsMasterTruckArcAllowed(inst_, i, j)) {
                out_i += x[i][j];
            }

            if (IsMasterTruckArcAllowed(inst_, j, i)) {
                in_i += x[j][i];
            }
        }

        model.addConstr(
            out_i == t[i],
            "truck_out_" + std::to_string(i)
        );

        model.addConstr(
            in_i == t[i],
            "truck_in_" + std::to_string(i)
        );
    }

    // ------------------------------------------------------------------
    // MTZ subtour elimination version.
    //
    // These constraints are added only in MTZ mode.
    //
    // For truck customers:
    //   1 <= ord_i <= n if t_i = 1
    //   ord_i = 0 if t_i = 0
    //
    // For any arc i -> j between two real customers:
    //   ord_j >= ord_i + 1 - n(1 - x_ij)
    //
    // This eliminates cycles among customer nodes.
    // ------------------------------------------------------------------
    if (options_.subtour_mode == MasterSubtourMode::MTZ) {
        const double M = static_cast<double>(inst_.n);

        for (int i = 1; i <= inst_.n; ++i) {
            model.addConstr(
                ord[i] <= M * t[i],
                "mtz_ord_ub_" + std::to_string(i)
            );

            model.addConstr(
                ord[i] >= t[i],
                "mtz_ord_lb_" + std::to_string(i)
            );
        }

        for (int i = 1; i <= inst_.n; ++i) {
            for (int j = 1; j <= inst_.n; ++j) {
                if (i == j) continue;
                if (!IsMasterTruckArcAllowed(inst_, i, j)) continue;

                model.addConstr(
                    ord[j] >= ord[i] + 1.0 - M * (1.0 - x[i][j]),
                    "mtz_" + std::to_string(i) + "_" + std::to_string(j)
                );
            }
        }
    }

    GRBLinExpr truck_cost = 0.0;

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (!IsMasterTruckArcAllowed(inst_, i, j)) continue;

            truck_cost += static_cast<double>(inst_.truck_time[i][j]) * x[i][j];
        }
    }

    model.addConstr(zT >= truck_cost, "truck_cost_lb");

    GRBLinExpr drone_work = 0.0;
    GRBLinExpr energy_required = 0.0;

    for (int i = 1; i <= inst_.n; ++i) {
        if (!inst_.drone_feasible[i]) continue;

        const double ri = static_cast<double>(inst_.DroneRoundTripTime(i));
        const double ei = static_cast<double>(inst_.energy[i]);

        drone_work += ri * y[i];
        energy_required += ei * y[i];

        model.addConstr(
            zD >= ri * y[i],
            "drone_single_" + std::to_string(i)
        );
    }

    model.addConstr(
        static_cast<double>(inst_.n_drones) * zD >=
            drone_work + static_cast<double>(inst_.swap_time) * q,
        "drone_workload_lb"
    );

    model.addConstr(
        static_cast<double>(inst_.battery_capacity) *
            (static_cast<double>(inst_.n_drones) + q) >= energy_required,
        "drone_energy_cycle_lb"
    );

    model.addConstr(z >= zT, "z_ge_truck");
    model.addConstr(z >= zD, "z_ge_drone");

    GRBLinExpr objective = 0.0;
    objective += z;
    model.setObjective(objective, GRB_MINIMIZE);

    CPSubproblem cp(inst_);
    MasterCallback cb(inst_, options_, cp, y, t, x, z);
    model.setCallback(&cb);

    model.optimize();

    const int status = model.get(GRB_IntAttr_Status);

    std::cout << "\n=== Master solve finished ===\n";
    std::cout << "Gurobi status: " << status << "\n";

    if (model.get(GRB_IntAttr_SolCount) > 0) {
        std::cout << "Best objective z: "
                  << model.get(GRB_DoubleAttr_ObjVal)
                  << "\n";

        std::vector<int> ysol(N, 0);
        std::vector<int> tsol(N, 0);
        std::vector<std::vector<int>> xsol(N, std::vector<int>(N, 0));

        for (int i = 1; i <= inst_.n; ++i) {
            ysol[i] = (
                inst_.drone_feasible[i] &&
                y[i].get(GRB_DoubleAttr_X) > 0.5
            ) ? 1 : 0;

            tsol[i] = (t[i].get(GRB_DoubleAttr_X) > 0.5) ? 1 : 0;
        }

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (!IsMasterTruckArcAllowed(inst_, i, j)) continue;

                xsol[i][j] = (x[i][j].get(GRB_DoubleAttr_X) > 0.5) ? 1 : 0;
            }
        }

        std::vector<Node> truck_route = DecodeTruckRouteFromX(inst_, xsol);

        std::vector<Node> drone_customers;
        std::vector<Node> truck_customers;

        for (int i = 1; i <= inst_.n; ++i) {
            if (ysol[i] == 1) {
                drone_customers.push_back(i);
            } else {
                truck_customers.push_back(i);
            }
        }

        CPSubproblem cp_final(inst_);
        CPOptions final_cp_options = options_.cp_options;
        final_cp_options.time_limit_seconds =
            std::max(final_cp_options.time_limit_seconds, 10.0);

        CPResult final_drone = cp_final.SolveDroneSubproblem(
            drone_customers,
            final_cp_options
        );

        std::cout << "\n=== Final decoded solution ===\n";

        std::cout << "Truck customers set: ";
        for (Node i : truck_customers) {
            std::cout << i << " ";
        }
        std::cout << "\n";

        std::cout << "Drone customers set: ";
        for (Node i : drone_customers) {
            std::cout << i << " ";
        }
        std::cout << "\n";

        std::cout << "\nOrdered truck route:\n  ";
        PrintRoute("", truck_route);

        if (final_drone.feasible) {
            std::cout << "\nDrone makespan: "
                      << final_drone.objective
                      << " seconds ("
                      << final_drone.objective / 60.0
                      << " minutes)\n";

            PrintDroneChainsAsTours(inst_, final_drone);
        } else {
            std::cout << "\n[WARN] Final CP drone subproblem was not solved feasible.\n";
        }

        // ---------------------------------------------------------------
        // 6. Export final result to CSV.
        // ---------------------------------------------------------------
        double obj = model.get(GRB_DoubleAttr_ObjVal);
        double lower_bound = model.get(GRB_DoubleAttr_ObjBound);
        double runtime_sec = model.get(GRB_DoubleAttr_Runtime);

        double mip_gap = -1.0;
        try {
            mip_gap = model.get(GRB_DoubleAttr_MIPGap);
        } catch (...) {
            mip_gap = -1.0;
        }

        double truck_time = ComputeTruckTimeFromX(inst_, xsol);

        double drone_makespan = -1.0;
        if (final_drone.feasible) {
            drone_makespan = static_cast<double>(final_drone.objective);
        }

        double final_makespan = truck_time;
        if (final_drone.feasible) {
            final_makespan = std::max(truck_time, drone_makespan);
        }

        ExportFinalResultCSV(
            options_.result_file,
            options_,
            inst_,
            status,
            obj,
            lower_bound,
            mip_gap,
            runtime_sec,
            truck_time,
            drone_makespan,
            final_makespan,
            truck_customers,
            drone_customers,
            truck_route,
            final_drone
        );
    } else {
        double runtime_sec = model.get(GRB_DoubleAttr_Runtime);

        double lower_bound = -1.0;
        try {
            lower_bound = model.get(GRB_DoubleAttr_ObjBound);
        } catch (...) {
            lower_bound = -1.0;
        }

        ExportNoSolutionCSV(
            options_.result_file,
            options_,
            inst_,
            status,
            lower_bound,
            runtime_sec
        );
    }
}

}  // namespace bbcp
