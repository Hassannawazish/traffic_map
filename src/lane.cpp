#include "lane.hpp"

int MapLane::lane_id = 0;

MapLane::MapLane() {
    set_base_attributes();
    set_visual_attributes();
    set_lane_id(lane_id);
    set_color();
    lane_id++;
}

void MapLane::set_base_attributes(){
    line.header.frame_id = "map";
    line.ns = "road_visualization";
}

void MapLane::set_visual_attributes(){
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    // The OpenDRIVE reference line is the solid middle of the road.
    line.scale.x = 0.25;
}

void MapLane::set_color(){
    line.color.a = 1.0;
    line.color.r = 1.0;
    line.color.g = 0.85;
    line.color.b = 0.0;
}

void MapLane::set_lane_id(int id){
    line.id = id;
}

visualization_msgs::msg::Marker MapLane::get_marker() const {
    return line;
}

void MapLane::pushback(geometry_msgs::msg::Point p){
    line.points.push_back(p);
}

SideLane::SideLane(){
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.scale.x = 0.18;
    set_color();
}

void SideLane::set_color(){
    line.color.a = 1.0;
    line.color.r = 1.0;
    line.color.g = 1.0;
    line.color.b = 1.0;
}

BorderLane::BorderLane(){
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.scale.x = 0.18;
    set_color();
}

void BorderLane::set_color(){
    line.color.a = 1.0;
    line.color.r = 1.0;
    line.color.g = 1.0;
    line.color.b = 1.0;
}
