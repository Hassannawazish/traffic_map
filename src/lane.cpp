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
    line.lifetime = ros::Duration(marker_lifetime);
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "road_visualization";
}

void MapLane::set_visual_attributes(){
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 2.0;
    line.scale.x = 0.1;
    line.scale.y = 0.1;
    line.scale.z = 0;
}

void MapLane::set_color(){
    line.color.a = 1.0;
    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;
}

void MapLane::set_lane_id(int id){
    line.id = id;
}

visualization_msgs::Marker MapLane::get_marker(){
    return line;
}

void MapLane::pushback(geometry_msgs::Point p){
    line.points.push_back(p);
}

SideLane::SideLane(){
    set_color();
}

void SideLane::set_color(){
    line.color.a = 1.0;
    line.color.r = 0.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
}

BorderLane::BorderLane(){
    set_color();
}

void BorderLane::set_color(){
    line.color.a = 1.0;
    line.color.r = 0.0;
    line.color.g = 0.0;
    line.color.b = 1.0;
}
