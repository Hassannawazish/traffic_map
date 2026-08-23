#pragma once
#include "parser.hpp"
#include <map>
#include <string>
#include <vector>

using LaneCoordinates = std::map<std::string, std::vector<double>>;

class map_process
{
public:
    map_process();
    const std::vector<LaneCoordinates> &get_left_lanes() const;
    const std::vector<LaneCoordinates> &get_right_lanes() const;

private:
    std::vector<LaneCoordinates> left_lanes_;
    std::vector<LaneCoordinates> right_lanes_;
};
