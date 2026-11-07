#ifndef BBCP_DATA_READER_H
#define BBCP_DATA_READER_H

#include <string>
#include "Types.h"

namespace bbcp {

struct ReadOptions {
    int64_t battery_capacity = 928;

    // If > 0, override NUM DRONES from file.
    int override_n_drones = -1;

    // If >= 0, override SWAP TIME from file, unit = minutes.
    double override_swap_minutes = -1.0;
};

Instance ReadInstanceFromFile(const std::string& filepath, const ReadOptions& opt = ReadOptions());

}  // namespace bbcp

#endif
