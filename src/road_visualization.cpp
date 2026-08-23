#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "lane.hpp"
#include "map_processor.h"
#include "types.h"
extern const int procs;

namespace {
void add_painted_segment(visualization_msgs::msg::Marker &marker, double x0, double y0,
    double x1, double y1, double width) {
  const double dx=x1-x0, dy=y1-y0;
  const double length=std::hypot(dx,dy);
  if (length<1e-6) return;
  const double nx=-dy*width/(2.0*length), ny=dx*width/(2.0*length);
  geometry_msgs::msg::Point a,b,c,d;
  a.x=x0+nx; a.y=y0+ny; a.z=0.15;
  b.x=x0-nx; b.y=y0-ny; b.z=0.15;
  c.x=x1+nx; c.y=y1+ny; c.z=0.15;
  d.x=x1-nx; d.y=y1-ny; d.z=0.15;
  marker.points.insert(marker.points.end(),{a,b,c,c,b,d});
}

void use_painted_geometry(visualization_msgs::msg::Marker &marker) {
  marker.type=visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.scale.x=1.0; marker.scale.y=1.0; marker.scale.z=1.0;
}
}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("road_visualization");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
  // Dynamic poses must never queue behind old frames. Keep only the newest
  // cube pose and allow stale packets to be dropped.
  auto vehicle_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto vehicle_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("vehicle_markers", vehicle_qos);
  auto map_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", qos);
  auto camera_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
  tf2_ros::TransformBroadcaster tf_broadcaster(node);
  Config::parse();
  RCLCPP_INFO(node->get_logger(), "Threads: %d, lanes: %d, geometries: %d, road length: %.2f",
    procs, Config::singleton().num_of_lanes, Config::singleton().num_of_geometeries, Config::singleton().road_length);
  map_process processed;
  const auto &left_boundaries = processed.get_left_lanes();
  const auto &right_boundaries = processed.get_right_lanes();
  MapLane reference;
  std::vector<double> route_x, route_y;
  for (const auto & data : Config::singleton().planeview_data) {
    geometry_msgs::msg::Point p; p.x=data.at("x"); p.y=data.at("y");
    p.z=0.15;
    reference.pushback(p); route_x.push_back(p.x); route_y.push_back(p.y);
  }
  // Drive through the center of lane -1 instead of on the center divider.
  if (!right_boundaries.empty()) {
    const auto &first_lane_edge=right_boundaries.front();
    const auto count=std::min({route_x.size(),route_y.size(),first_lane_edge.at("x").size(),first_lane_edge.at("y").size()});
    for (std::size_t i=0; i<count; ++i) {
      route_x[i]=(route_x[i]+first_lane_edge.at("x")[i])*0.5;
      route_y[i]=(route_y[i]+first_lane_edge.at("y")[i])*0.5;
    }
  }
  std::vector<double> route_distance(route_x.size(),0.0);
  for (std::size_t i=1; i<route_x.size(); ++i) {
    route_distance[i]=route_distance[i-1]+std::hypot(route_x[i]-route_x[i-1],route_y[i]-route_y[i-1]);
  }
  auto route_position = [&](double distance) {
    if (route_distance.size()<2 || route_distance.back()<=0.0) return std::pair<double,double>{0.0,0.0};
    distance=std::fmod(distance,route_distance.back());
    const auto upper=std::upper_bound(route_distance.begin(),route_distance.end(),distance);
    const std::size_t next=std::min<std::size_t>(std::distance(route_distance.begin(),upper),route_distance.size()-1);
    const std::size_t previous=next-1;
    const double segment=route_distance[next]-route_distance[previous];
    const double ratio=segment>0.0?(distance-route_distance[previous])/segment:0.0;
    return std::pair<double,double>{route_x[previous]+(route_x[next]-route_x[previous])*ratio,
      route_y[previous]+(route_y[next]-route_y[previous])*ratio};
  };
  const auto center_samples=reference.line.points;
  reference.line.points.clear();
  use_painted_geometry(reference.line);
  for (std::size_t i=0; i+1<center_samples.size(); ++i) {
    add_painted_segment(reference.line,center_samples[i].x,center_samples[i].y,
      center_samples[i+1].x,center_samples[i+1].y,0.30);
  }
  std::vector<SideLane> boundary_markers;
  boundary_markers.reserve(left_boundaries.size() + right_boundaries.size());
  auto append = [&boundary_markers](const LaneCoordinates &lane, bool solid_edge) {
    boundary_markers.emplace_back();
    auto &marker = boundary_markers.back();
    use_painted_geometry(marker.line);
    const auto count = std::min(lane.at("x").size(), lane.at("y").size());
    if (solid_edge) {
      for (std::size_t i=0; i+1<count; ++i) {
        add_painted_segment(marker.line,lane.at("x")[i],lane.at("y")[i],
          lane.at("x")[i+1],lane.at("y")[i+1],0.28);
      }
    } else {
      for (std::size_t i=0; i+2<count; i += 6) {
        add_painted_segment(marker.line,lane.at("x")[i],lane.at("y")[i],
          lane.at("x")[i+2],lane.at("y")[i+2],0.24);
      }
    }
  };
  for (std::size_t i=0; i<left_boundaries.size(); ++i) append(left_boundaries[i],i+1==left_boundaries.size());
  for (std::size_t i=0; i<right_boundaries.size(); ++i) append(right_boundaries[i],i+1==right_boundaries.size());
  visualization_msgs::msg::Marker road_surface;
  road_surface.header.frame_id="map"; road_surface.ns="road_visualization"; road_surface.id=900;
  road_surface.type=visualization_msgs::msg::Marker::TRIANGLE_LIST;
  road_surface.action=visualization_msgs::msg::Marker::ADD; road_surface.pose.orientation.w=1.0;
  road_surface.scale.x=1.0; road_surface.scale.y=1.0; road_surface.scale.z=1.0;
  road_surface.color.r=0.12F; road_surface.color.g=0.13F; road_surface.color.b=0.14F; road_surface.color.a=1.0F;
  if (!left_boundaries.empty() && !right_boundaries.empty()) {
    const auto &left_edge=left_boundaries.back();
    const auto &right_edge=right_boundaries.back();
    const auto count=std::min({left_edge.at("x").size(),left_edge.at("y").size(),
      right_edge.at("x").size(),right_edge.at("y").size()});
    for (std::size_t i=0; i+5<count; i += 5) {
      const std::size_t next=i+5;
      geometry_msgs::msg::Point l0,l1,r0,r1;
      l0.x=left_edge.at("x")[i]; l0.y=left_edge.at("y")[i]; l0.z=-0.05;
      l1.x=left_edge.at("x")[next]; l1.y=left_edge.at("y")[next]; l1.z=-0.05;
      r0.x=right_edge.at("x")[i]; r0.y=right_edge.at("y")[i]; r0.z=-0.05;
      r1.x=right_edge.at("x")[next]; r1.y=right_edge.at("y")[next]; r1.z=-0.05;
      road_surface.points.insert(road_surface.points.end(),{l0,r0,l1,l1,r0,r1});
    }
  }
  std::vector<visualization_msgs::msg::Marker> lanes={road_surface,reference.get_marker()};
  for (const auto &boundary : boundary_markers) lanes.push_back(boundary.get_marker());
  RCLCPP_INFO(node->get_logger(), "Publishing center line and %zu lane boundaries", boundary_markers.size());
  auto publish_map = [&]() {
    visualization_msgs::msg::MarkerArray map_message;
    for (auto &marker : lanes) {
      marker.header.stamp=node->now();
      map_message.markers.push_back(marker);
    }
    map_pub->publish(map_message);
  };
  publish_map();
  const double vehicle_speed=node->declare_parameter<double>("vehicle_speed_mps",6.0);
  constexpr double update_rate=60.0;
  constexpr std::size_t vehicle_count=16;
  std::vector<double> vehicle_distances(vehicle_count,0.0);
  std::vector<double> vehicle_speeds(vehicle_count,vehicle_speed);
  for (std::size_t i=1; i<vehicle_count; ++i) {
    vehicle_distances[i]=80.0*static_cast<double>(i);
    vehicle_speeds[i]=3.0+1.25*static_cast<double>(i%6);
  }
  double camera_yaw=0.0;
  bool camera_yaw_initialized=false;
  rclcpp::Rate rate(update_rate);
  while (rclcpp::ok()) {
    const auto stamp=node->now();
    if (route_distance.size()>1 && route_distance.back()>0.0) {
      const auto position=route_position(vehicle_distances[0]);
      const auto look_ahead=route_position(vehicle_distances[0]+10.0);
      const double yaw=std::atan2(look_ahead.second-position.second,look_ahead.first-position.first);

      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp=stamp; transform.header.frame_id="map"; transform.child_frame_id="vehicle";
      transform.transform.translation.x=position.first;
      transform.transform.translation.y=position.second;
      transform.transform.translation.z=0.75;
      transform.transform.rotation.z=std::sin(yaw*0.5);
      transform.transform.rotation.w=std::cos(yaw*0.5);
      tf_broadcaster.sendTransform(transform);

      // Smooth only the chase camera heading. The vehicle itself keeps the
      // exact road heading, while small XODR angle changes no longer shake the
      // entire rendered scene from frame to frame.
      if (!camera_yaw_initialized) {
        camera_yaw=yaw;
        camera_yaw_initialized=true;
      } else {
        const double yaw_error=std::atan2(std::sin(yaw-camera_yaw),std::cos(yaw-camera_yaw));
        camera_yaw+=0.04*yaw_error;
      }
      geometry_msgs::msg::TransformStamped camera_transform;
      camera_transform.header.stamp=stamp;
      camera_transform.header.frame_id="map";
      camera_transform.child_frame_id="camera_follow";
      camera_transform.transform.translation=transform.transform.translation;
      camera_transform.transform.rotation.z=std::sin(camera_yaw*0.5);
      camera_transform.transform.rotation.w=std::cos(camera_yaw*0.5);
      tf_broadcaster.sendTransform(camera_transform);

      // Send every vehicle atomically in one depth-1 array. RViz can only see
      // a complete newest traffic frame, never a mixture of stale poses.
      visualization_msgs::msg::MarkerArray traffic;
      traffic.markers.reserve(vehicle_count);
      for (std::size_t i=0; i<vehicle_count; ++i) {
        const auto car_position=route_position(vehicle_distances[i]);
        const auto car_ahead=route_position(vehicle_distances[i]+10.0);
        const double car_yaw=std::atan2(car_ahead.second-car_position.second,car_ahead.first-car_position.first);
        const double lateral=3.2*static_cast<double>(i%4);
        visualization_msgs::msg::Marker car;
        car.header.stamp=stamp; car.header.frame_id="map"; car.ns="traffic";
        car.id=1000+static_cast<int>(i); car.type=visualization_msgs::msg::Marker::CUBE;
        car.action=visualization_msgs::msg::Marker::ADD; car.pose.orientation.z=std::sin(car_yaw*0.5);
        car.pose.orientation.w=std::cos(car_yaw*0.5);
        car.pose.position.x=car_position.first+std::sin(car_yaw)*lateral;
        car.pose.position.y=car_position.second-std::cos(car_yaw)*lateral;
        car.pose.position.z=0.75;
        car.scale.x=4.5; car.scale.y=2.0; car.scale.z=1.5;
        car.color.r=0.15F+0.12F*static_cast<float>(i%5);
        car.color.g=0.25F+0.10F*static_cast<float>((i+2)%5);
        car.color.b=0.9F-0.10F*static_cast<float>(i%5); car.color.a=1.0F;
        traffic.markers.push_back(car);
        vehicle_distances[i]=std::fmod(vehicle_distances[i]+vehicle_speeds[i]/update_rate,route_distance.back());
      }
      vehicle_pub->publish(traffic);
    }
    sensor_msgs::msg::CameraInfo camera; camera.header.stamp=stamp; camera.header.frame_id="map";
    camera.distortion_model="equidistant"; camera.p[3]=4808.0; camera.p[7]=3099.0; camera_pub->publish(camera);
    rclcpp::spin_some(node); rate.sleep();
  }
  rclcpp::shutdown(); return 0;
}
