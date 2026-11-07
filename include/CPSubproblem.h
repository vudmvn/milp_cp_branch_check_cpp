#ifndef BBCP_CP_SUBPROBLEM_H
#define BBCP_CP_SUBPROBLEM_H

#include <cstdint>
#include <vector>
#include "Types.h"

namespace bbcp
{

    struct CPOptions
    {
        double time_limit_seconds = 5.0;
        bool log_search = false;
        bool stop_after_first_solution = false;
        bool use_warm_start = true;
    };

    class CPSubproblem
    {
    public:
        explicit CPSubproblem(const Instance &inst);

        // deadline < 0: solve MIN z and return valid bound + feasible schedule.
        // deadline >= 0: solve feasibility z <= deadline (NOT the full optimum).
        CPResult SolveDroneSubproblem(const std::vector<Node> &drone_customers,
                                      const CPOptions &options,
                                      int64_t deadline = -1) const;

    private:
        const Instance &inst_;
    };

} // namespace bbcp
#endif // BBCP_CP_SUBPROBLEM_H
