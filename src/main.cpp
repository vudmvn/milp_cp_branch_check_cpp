#include <cstdlib>
#include <iostream>
#include <string>

#include "DataReader.h"
#include "MasterProblem.h"

namespace {

std::string BaseName(const std::string& path) {
    const size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) {
        return path;
    }
    return path.substr(pos + 1);
}

}  // namespace

int main(int argc, char** argv) {
    using namespace bbcp;

    if (argc < 2) {
        std::cerr
            << "Usage:\n"
            << "  " << argv[0]
            << " <data_file> [n_drones] [swap_minutes] [master_time] [cp_time] [master_mode] [result_csv]\n\n"
            << "Arguments:\n"
            << "  data_file     : input instance file\n"
            << "  n_drones      : optional override for number of drones\n"
            << "  swap_minutes  : optional override for swap time, in minutes\n"
            << "  master_time   : optional MILP master time limit, in seconds\n"
            << "  cp_time       : optional CP subproblem time limit, in seconds\n"
            << "  master_mode   : optional master version\n"
            << "                  cut = lazy subtour cuts in callback, default\n"
            << "                  mtz = MTZ constraints in master\n"
            << "  result_csv    : optional output CSV file, default final_result.csv\n\n"
            << "Examples:\n"
            << "  " << argv[0] << " data/30-c-0-c-swap-0.txt\n"
            << "  " << argv[0] << " data/30-c-0-c-swap-0.txt 4 3 3600 5 cut result_30_cut.csv\n"
            << "  " << argv[0] << " data/30-c-0-c-swap-0.txt 4 3 3600 5 mtz result_30_mtz.csv\n";
        return 1;
    }

    const std::string filepath = argv[1];

    ReadOptions read_options;

    if (argc >= 3) {
        read_options.override_n_drones = std::atoi(argv[2]);
    }

    if (argc >= 4) {
        read_options.override_swap_minutes = std::atof(argv[3]);
    }

    MasterOptions master_options;
    master_options.instance_name = BaseName(filepath);

    if (argc >= 5) {
        master_options.time_limit_seconds = std::atof(argv[4]);
    }

    if (argc >= 6) {
        master_options.cp_options.time_limit_seconds = std::atof(argv[5]);
    }

    if (argc >= 7) {
        master_options.subtour_mode = ParseMasterSubtourMode(argv[6]);
    }

    if (argc >= 8) {
        master_options.result_file = argv[7];
    }

    if (argc >= 9) {
        std::string strat = argv[8];
        for (char &c : strat) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        if (strat == "baseline" || strat == "none" || strat == "off") {
            master_options.use_bpp_lb = false;
            master_options.use_incumbent_cutoff = false;
        } else if (strat == "bpp") {
            master_options.use_bpp_lb = true;
            master_options.use_incumbent_cutoff = false;
        } else if (strat == "cutoff") {
            master_options.use_bpp_lb = false;
            master_options.use_incumbent_cutoff = true;
        } else if (strat == "all" || strat == "bpp+cutoff" || strat == "both") {
            master_options.use_bpp_lb = true;
            master_options.use_incumbent_cutoff = true;
        } else {
            std::cerr << "Warning: Unknown strategy '" << argv[8] << "'. Using default (all).\n";
        }
    }

    master_options.verbose = true;
    master_options.cut_verbose = false;
    master_options.cp_options.log_search = false;
    master_options.cp_options.stop_after_first_solution = false;

    try {
        Instance inst = ReadInstanceFromFile(filepath, read_options);

        MasterProblem master(inst, master_options);
        master.Solve();

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 2;
    }

    return 0;
}
