#include "dstar_lite.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

int main() {
  traffic_map::DStarLite planner;
  constexpr std::size_t lanes = 3, steps = 20;

  std::vector<bool> clear(lanes * steps, false);
  const auto straight = planner.Plan(lanes, steps, 0, 0, clear);
  if (straight.size() != steps ||
      std::any_of(straight.begin(), straight.end(), [](std::size_t lane) { return lane != 0; })) {
    std::cerr << "D* Lite did not retain the clear lane\n";
    return 1;
  }

  std::vector<bool> obstruction(lanes * steps, false);
  for (std::size_t step = 1; step <= 8; ++step) obstruction[step * lanes] = true;
  const auto avoidance = planner.Plan(lanes, steps, 0, 0, obstruction);
  if (avoidance.size() != steps ||
      std::none_of(avoidance.begin(), avoidance.end(), [](std::size_t lane) { return lane == 1; })) {
    std::cerr << "D* Lite did not route around the blocked lane\n";
    return 2;
  }

  std::vector<bool> closed(lanes * steps, false);
  for (std::size_t lane = 0; lane < lanes; ++lane) closed[5 * lanes + lane] = true;
  if (!planner.Plan(lanes, steps, 0, 0, closed).empty()) {
    std::cerr << "D* Lite returned a path through a closed road\n";
    return 3;
  }
  return 0;
}
