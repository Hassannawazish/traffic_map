#include "map_processor.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

namespace
{
double value_or_zero(const std::map<std::string, double> &values, const std::string &key)
{
    const auto it = values.find(key);
    return it == values.end() ? 0.0 : it->second;
}

double evaluate_width(const std::vector<std::map<std::string, double>> &records, double road_s)
{
    if (records.empty()) return 0.0;
    const auto next = std::upper_bound(records.begin(), records.end(), road_s,
        [](double s, const auto &record) { return s < value_or_zero(record, "sOffset"); });
    const auto &active_record = next == records.begin() ? records.front() : *std::prev(next);
    const auto *active = &active_record;
    const double ds = std::max(0.0, road_s - value_or_zero(*active, "sOffset"));
    return value_or_zero(*active, "a") + value_or_zero(*active, "b") * ds +
           value_or_zero(*active, "c") * ds * ds + value_or_zero(*active, "d") * ds * ds * ds;
}

std::vector<LaneCoordinates> create_boundaries(
    const std::map<int, std::vector<std::map<std::string, double>>> &lane_widths, bool left_side)
{
    std::vector<std::pair<int, const std::vector<std::map<std::string, double>> *>> ordered;
    for (const auto &[lane_id, widths] : lane_widths) {
        if ((left_side && lane_id > 0) || (!left_side && lane_id < 0))
            ordered.emplace_back(std::abs(lane_id), &widths);
    }
    std::sort(ordered.begin(), ordered.end(), [](const auto &a, const auto &b) { return a.first < b.first; });

    std::vector<LaneCoordinates> boundaries(ordered.size());
    for (const auto &sample : Config::singleton().planeview_data) {
        const double road_s = value_or_zero(sample, "s");
        const double heading = value_or_zero(sample, "hdg");
        double offset = 0.0;
        for (std::size_t lane = 0; lane < ordered.size(); ++lane) {
            offset += evaluate_width(*ordered[lane].second, road_s);
            const double side = left_side ? 1.0 : -1.0;
            boundaries[lane]["x"].push_back(value_or_zero(sample, "x") - side * std::sin(heading) * offset);
            boundaries[lane]["y"].push_back(value_or_zero(sample, "y") + side * std::cos(heading) * offset);
        }
    }
    return boundaries;
}
}  // namespace

map_process::map_process()
  : left_lanes_(create_boundaries(Config::singleton().left_lanes_frames, true)),
    right_lanes_(create_boundaries(Config::singleton().right_lanes_frames, false)) {}

const std::vector<LaneCoordinates> &map_process::get_left_lanes() const { return left_lanes_; }
const std::vector<LaneCoordinates> &map_process::get_right_lanes() const { return right_lanes_; }
