#include "dstar_lite.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace traffic_map {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kLaneChangeCost = 2.4;
constexpr double kEpsilon = 1e-9;

struct QueueEntry {
  std::size_t node;
  double first;
  double second;
};

struct LaterKey {
  bool operator()(const QueueEntry &a, const QueueEntry &b) const {
    if (std::abs(a.first - b.first) > kEpsilon) return a.first > b.first;
    return a.second > b.second;
  }
};

bool KeyLess(const QueueEntry &a, const QueueEntry &b) {
  if (a.first + kEpsilon < b.first) return true;
  if (b.first + kEpsilon < a.first) return false;
  return a.second + kEpsilon < b.second;
}
}  // namespace

std::vector<std::size_t> DStarLite::Plan(
    const std::size_t lane_count, const std::size_t longitudinal_steps,
    const std::size_t start_lane, const std::size_t goal_lane,
    const std::vector<bool> &blocked) {
  if (lane_count == 0 || longitudinal_steps < 2 || start_lane >= lane_count ||
      goal_lane >= lane_count || blocked.size() != lane_count * longitudinal_steps) {
    return {};
  }

  const auto node = [lane_count](std::size_t step, std::size_t lane) {
    return step * lane_count + lane;
  };
  const auto lane_of = [lane_count](std::size_t value) { return value % lane_count; };
  const auto step_of = [lane_count](std::size_t value) { return value / lane_count; };
  const std::size_t start = node(0, start_lane);
  const std::size_t goal = node(longitudinal_steps - 1, goal_lane);
  std::vector<double> g(blocked.size(), kInf), rhs(blocked.size(), kInf);
  rhs[goal] = 0.0;

  auto heuristic = [&](std::size_t a, std::size_t b) {
    return std::abs(static_cast<double>(step_of(a)) - static_cast<double>(step_of(b))) +
           1.4 * std::abs(static_cast<double>(lane_of(a)) - static_cast<double>(lane_of(b)));
  };
  auto key = [&](std::size_t value) {
    const double best = std::min(g[value], rhs[value]);
    return QueueEntry{value, best + heuristic(start, value), best};
  };
  auto successors = [&](std::size_t value) {
    std::vector<std::size_t> result;
    const std::size_t step = step_of(value), lane = lane_of(value);
    if (step + 1 >= longitudinal_steps) return result;
    result.push_back(node(step + 1, lane));
    if (lane > 0) result.push_back(node(step + 1, lane - 1));
    if (lane + 1 < lane_count) result.push_back(node(step + 1, lane + 1));
    return result;
  };
  auto predecessors = [&](std::size_t value) {
    std::vector<std::size_t> result;
    const std::size_t step = step_of(value), lane = lane_of(value);
    if (step == 0) return result;
    result.push_back(node(step - 1, lane));
    if (lane > 0) result.push_back(node(step - 1, lane - 1));
    if (lane + 1 < lane_count) result.push_back(node(step - 1, lane + 1));
    return result;
  };
  auto edge_cost = [&](std::size_t from, std::size_t to) {
    if (blocked[to]) return kInf;
    return lane_of(from) == lane_of(to) ? 1.0 : kLaneChangeCost;
  };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LaterKey> open;
  open.push(key(goal));
  auto update_vertex = [&](std::size_t value) {
    if (value != goal) {
      rhs[value] = kInf;
      for (const std::size_t next : successors(value))
        rhs[value] = std::min(rhs[value], edge_cost(value, next) + g[next]);
    }
    if (std::abs(g[value] - rhs[value]) > kEpsilon) open.push(key(value));
  };

  while (!open.empty() &&
         (KeyLess(open.top(), key(start)) || std::abs(rhs[start] - g[start]) > kEpsilon)) {
    const QueueEntry old_key = open.top();
    open.pop();
    const QueueEntry new_key = key(old_key.node);
    if (std::abs(g[old_key.node] - rhs[old_key.node]) <= kEpsilon) continue;
    if (KeyLess(old_key, new_key)) {
      open.push(new_key);
    } else if (g[old_key.node] > rhs[old_key.node]) {
      g[old_key.node] = rhs[old_key.node];
      for (const std::size_t previous : predecessors(old_key.node)) update_vertex(previous);
    } else {
      g[old_key.node] = kInf;
      update_vertex(old_key.node);
      for (const std::size_t previous : predecessors(old_key.node)) update_vertex(previous);
    }
  }

  if (!std::isfinite(g[start])) return {};
  std::vector<std::size_t> path{start_lane};
  std::size_t current = start;
  while (current != goal) {
    double best_cost = kInf;
    std::size_t best = current;
    for (const std::size_t next : successors(current)) {
      const double candidate = edge_cost(current, next) + g[next];
      if (candidate + kEpsilon < best_cost) {
        best_cost = candidate;
        best = next;
      }
    }
    if (best == current || !std::isfinite(best_cost)) return {};
    current = best;
    path.push_back(lane_of(current));
  }
  return path;
}

}  // namespace traffic_map
