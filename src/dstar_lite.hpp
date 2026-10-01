#pragma once

#include <cstddef>
#include <vector>

namespace traffic_map {

// D* Lite over a forward-only, lane-level rolling road graph. Nodes are laid
// out as [longitudinal step][lane]. The returned vector contains the selected
// lane at every step from the ego position to the planning horizon.
class DStarLite {
 public:
  std::vector<std::size_t> Plan(std::size_t lane_count,
                                std::size_t longitudinal_steps,
                                std::size_t start_lane,
                                std::size_t goal_lane,
                                const std::vector<bool> &blocked);
};

}  // namespace traffic_map
