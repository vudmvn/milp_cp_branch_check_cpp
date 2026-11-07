#include "DataReader.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace bbcp {

namespace {

std::string Trim(const std::string& s) {
    size_t l = 0;
    while (l < s.size() && std::isspace(static_cast<unsigned char>(s[l]))) ++l;

    size_t r = s.size();
    while (r > l && std::isspace(static_cast<unsigned char>(s[r - 1]))) --r;

    return s.substr(l, r - l);
}

std::string ToUpper(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return s;
}

struct RawNode {
    int id = -1;
    double x = 0.0;
    double y = 0.0;
    double demand = 0.0;
};

double GetDouble(const std::map<std::string, std::string>& header,
                 const std::string& key,
                 double default_value,
                 bool required) {
    auto it = header.find(ToUpper(key));
    if (it == header.end()) {
        if (required) {
            throw std::runtime_error("Missing header key: " + key);
        }
        return default_value;
    }
    return std::stod(it->second);
}

int GetInt(const std::map<std::string, std::string>& header,
           const std::string& key,
           int default_value,
           bool required) {
    return static_cast<int>(std::llround(GetDouble(header, key, default_value, required)));
}

}  // namespace

Instance ReadInstanceFromFile(const std::string& filepath, const ReadOptions& opt) {
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        throw std::runtime_error("Cannot open data file: " + filepath);
    }

    std::map<std::string, std::string> header;
    std::vector<RawNode> records;

    std::string line;
    while (std::getline(fin, line)) {
        line = Trim(line);
        if (line.empty()) continue;

        // Header line: KEY,VALUE
        if (line.find(',') != std::string::npos) {
            const size_t pos = line.find(',');
            std::string key = Trim(line.substr(0, pos));
            std::string value = Trim(line.substr(pos + 1));
            header[ToUpper(key)] = value;
            continue;
        }

        // Node line: id x y demand
        std::stringstream ss(line);
        RawNode rec;
        ss >> rec.id >> rec.x >> rec.y >> rec.demand;
        if (ss.fail()) {
            throw std::runtime_error("Invalid node line: " + line);
        }
        records.push_back(rec);
    }

    if (records.empty()) {
        throw std::runtime_error("No node records in file: " + filepath);
    }

    int max_id = 0;
    std::set<int> seen;
    for (const RawNode& rec : records) {
        if (rec.id < 0) {
            throw std::runtime_error("Negative node id is not supported.");
        }
        if (seen.count(rec.id)) {
            throw std::runtime_error("Duplicate node id: " + std::to_string(rec.id));
        }
        seen.insert(rec.id);
        max_id = std::max(max_id, rec.id);
    }

    for (int i = 0; i <= max_id; ++i) {
        if (!seen.count(i)) {
            throw std::runtime_error("Missing node id: " + std::to_string(i));
        }
    }

    Instance inst;
    inst.n = max_id;                 // customers are 1..max_id
    inst.start_depot = 0;
    inst.end_depot = inst.n + 1;     // dummy end depot
    inst.battery_capacity = opt.battery_capacity;

    inst.n_drones = GetInt(header, "NUM DRONES", 1, false);
    if (opt.override_n_drones > 0) inst.n_drones = opt.override_n_drones;

    inst.drone_cap = GetDouble(header, "DRONE CAP", 0.0, true);
    inst.truck_speed = GetDouble(header, "TRUCK SPEED", 0.0, true);
    inst.drone_speed = GetDouble(header, "DRONE SPEED", 0.0, true);

    double swap_minutes = GetDouble(header, "SWAP TIME", 0.0, false);
    if (opt.override_swap_minutes >= 0.0) swap_minutes = opt.override_swap_minutes;
    inst.swap_time = static_cast<int64_t>(std::llround(swap_minutes * 60.0));

    const int N = inst.NumNodes();
    inst.coords.assign(N, {});
    inst.demand.assign(N, 0.0);
    inst.truck_time.assign(N, std::vector<int64_t>(N, 0));
    inst.drone_time.assign(N, std::vector<int64_t>(N, 0));
    inst.energy.assign(N, 0);
    inst.drone_feasible.assign(N, false);

    for (const RawNode& rec : records) {
        inst.coords[rec.id] = {rec.x, rec.y};
        inst.demand[rec.id] = (rec.id == 0 ? 0.0 : rec.demand);
    }

    // Dummy end depot has the same coordinate as depot 0.
    inst.coords[inst.end_depot] = inst.coords[0];
    inst.demand[inst.end_depot] = 0.0;

    // Truck travel time: Manhattan distance / truck speed.
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i == j) continue;
            const double d = std::abs(inst.coords[i].x - inst.coords[j].x)
                           + std::abs(inst.coords[i].y - inst.coords[j].y);
            inst.truck_time[i][j] = static_cast<int64_t>(
                std::llround(3600.0 * d / inst.truck_speed)
            );
        }
    }

    // Drone travel time: Euclidean distance / drone speed.
    for (int i = 0; i <= inst.n; ++i) {
        for (int j = 0; j <= inst.n; ++j) {
            if (i == j) continue;
            const double dx = inst.coords[i].x - inst.coords[j].x;
            const double dy = inst.coords[i].y - inst.coords[j].y;
            const double d = std::sqrt(dx * dx + dy * dy);
            inst.drone_time[i][j] = static_cast<int64_t>(
                std::llround(3600.0 * d / inst.drone_speed)
            );
        }
    }

    // Energy calculation copied from the original Python read_data().
    const double alpha = 0.217;
    const double beta = 0.185;
    const double xi = 650.0;

    for (int i = 1; i <= inst.n; ++i) {
        const double w = inst.demand[i];

        if (w > inst.drone_cap) {
            inst.energy[i] = INF_ENERGY;
            inst.drone_feasible[i] = false;
            continue;
        }

        const double t = static_cast<double>(inst.drone_time[0][i]);
        const double hrt = 1.0 - (alpha / xi) * 2.0 * t;

        if (hrt <= 0.0) {
            inst.energy[i] = INF_ENERGY;
            inst.drone_feasible[i] = false;
            continue;
        }

        double e = ((alpha * (t * w)) + (2.0 * beta * t)) / hrt;
        e = e / 3600.0;

        const int64_t scaled = static_cast<int64_t>(std::llround(e * 1000.0 + 0.5));
        inst.energy[i] = scaled;
        inst.drone_feasible[i] = (scaled > 0 && scaled <= inst.battery_capacity);
    }

    std::cout << "Loaded instance: " << filepath << "\n";
    std::cout << "  customers      : " << inst.n << "\n";
    std::cout << "  drone feasible : " << inst.DroneCustomers().size() << "\n";
    std::cout << "  drones         : " << inst.n_drones << "\n";
    std::cout << "  swap time      : " << inst.swap_time << " seconds\n";
    std::cout << "  battery cap    : " << inst.battery_capacity << "\n";

    return inst;
}

}  // namespace bbcp
