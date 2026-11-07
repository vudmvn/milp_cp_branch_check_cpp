#ifndef BBCP_MASTER_PROBLEM_H
#define BBCP_MASTER_PROBLEM_H

#include <string>
#include <vector>

#include "Types.h"
#include "CPSubproblem.h"

namespace bbcp
{

    enum class MasterSubtourMode
    {
        LazyCut,
        MTZ
    };

    MasterSubtourMode ParseMasterSubtourMode(const std::string &mode);
    std::string MasterSubtourModeName(MasterSubtourMode mode);

    struct MasterOptions
    {
        double time_limit_seconds = 3600.0;
        double mip_gap = 0.0;
        bool verbose = true;
        bool cut_verbose = false;
        MasterSubtourMode subtour_mode = MasterSubtourMode::LazyCut;
        CPOptions cp_options;
        // Retained for source-level and CSV backward compatibility; UNUSED in cuts.
        double big_m = 1e7;
        std::string result_file = "final_result.csv";
        std::string instance_name = "";

        // Advanced LBBD & Cutoff options:
        bool use_bpp_lb = true;           // Analytical 1D Bin-Packing & swap lower bound (0ms pre-filter)
        bool use_incumbent_cutoff = true; // Two-phase incumbent-directed subproblem cutoff
    };

    class MasterProblem
    {
    public:
        MasterProblem(const Instance &inst, const MasterOptions &options);
        void Solve();

    private:
        const Instance &inst_;
        MasterOptions options_;
    };

} // namespace bbcp
#endif // BBCP_MASTER_PROBLEM_H
