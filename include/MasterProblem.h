#ifndef BBCP_MASTER_PROBLEM_H
#define BBCP_MASTER_PROBLEM_H

#include <string>
#include <vector>

#include "Types.h"
#include "CPSubproblem.h"

namespace bbcp {

// MasterSubtourMode controls how the MILP master removes truck subtours.
enum class MasterSubtourMode {
    LazyCut,  // Use lazy subtour cuts in the Gurobi callback.
    MTZ       // Add MTZ order constraints directly in the MILP master.
};

// Convert command-line string to MasterSubtourMode.
// Accepted values are implemented in MasterProblem.cpp:
//   "cut", "lazy", "lazycut", "branch-and-cut" -> LazyCut
//   "mtz"                                     -> MTZ
MasterSubtourMode ParseMasterSubtourMode(const std::string& mode);

// Return a short printable name:
//   LazyCut -> "cut"
//   MTZ     -> "mtz"
std::string MasterSubtourModeName(MasterSubtourMode mode);

struct MasterOptions {
    double time_limit_seconds = 3600.0;
    double mip_gap = 0.0;

    // If true, Gurobi log and [MIPSOL] messages are printed.
    bool verbose = true;

    // If true, lazy cut messages are printed.
    // Keep false by default to avoid excessive callback output.
    bool cut_verbose = false;

    // Select master version.
    MasterSubtourMode subtour_mode = MasterSubtourMode::LazyCut;

    // CP subproblem options.
    CPOptions cp_options;

    // Big-M for assignment-specific optimality cuts:
    //   z + M * HammingDistance(y, ybar) >= phi(ybar)
    double big_m = 1e7;

    // Output CSV file for final result summary.
    // If empty, result export is disabled.
    std::string result_file = "final_result.csv";

    // Instance name/path used in the result CSV.
    std::string instance_name = "";
};

class MasterProblem {
public:
    MasterProblem(const Instance& inst, const MasterOptions& options);

    void Solve();

private:
    const Instance& inst_;
    MasterOptions options_;
};

}  // namespace bbcp

#endif
