#ifndef BBCP_CP_SUBPROBLEM_H
#define BBCP_CP_SUBPROBLEM_H

#include <vector>
#include "Types.h"

namespace bbcp {

struct CPOptions {
    double time_limit_seconds = 5.0;
    bool log_search = false;
    bool stop_after_first_solution = false;
};

class CPSubproblem {
public:
    explicit CPSubproblem(const Instance& inst);

    // Solve the drone subproblem for a fixed drone customer set.
    //
    // Input:
    //   drone_customers: customers assigned to drone by the MILP master.
    //
    // Output:
    //   minimum drone makespan under anonymous drone chains, swap, and energy.
    CPResult SolveDroneSubproblem(const std::vector<Node>& drone_customers,
                                  const CPOptions& options) const;

private:
    const Instance& inst_;
};

}  // namespace bbcp

#endif
