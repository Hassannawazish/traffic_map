#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/CameraInfo.h>

#include <fstream>
#include "map_processor.h"
#include "types.h"
#include "lane.hpp"
#include <random>

extern const int procs;

int main(int argc, char** argv )
{  
  ros::init(argc, argv, "road_visualization");
  ros::NodeHandle n;
  ros::Rate r(30);
  ros::Publisher camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10, 0);
  float f = 0.0;
  
  Config::singleton().parse();
  std::cout<<"Total Number of thread = "<<procs<<std::endl;
  std::cout<<"lanes = "<<Config::singleton().num_of_lanes<<std::endl;
  std::cout<<"No. of Geometeries = "<<Config::singleton().num_of_geometeries<<std::endl; 
  std::cout<<"Road Length = "<<Config::singleton().road_length<<std::endl;
  map_process processed;
  // std::map< std::string, std::vector<double>> lanes_coords = processed.get_right_lane(1);
  std::vector< double> x1 =  processed.get_lane(1)["x1"];
  std::vector< double> y1 =  processed.get_lane(1)["y1"];
  std::vector< double> x2 =  processed.get_lane(1)["x2"];
  std::vector< double> y2 =  processed.get_lane(1)["y2"];
  std::vector< double> x3 =  processed.get_lane(1)["x3"];
  std::vector< double> y3 =  processed.get_lane(1)["y3"];

  std::vector< double> xr1 =  processed.get_lane(2)["x1"];
  std::vector< double> yr1 =  processed.get_lane(2)["y1"];
  std::vector< double> xr2 =  processed.get_lane(2)["x2"];
  std::vector< double> yr2 =  processed.get_lane(2)["y2"];
  std::vector< double> xr3 =  processed.get_lane(2)["x3"];
  std::vector< double> yr3 =  processed.get_lane(2)["y3"];
  int t=0;

  MapLanes Ref_line, left_lane1, left_lane2, left_lane3, right_lane1, right_lane2, right_lane3;
  Ref_line.set_lane_id(0);
  left_lane1.set_lane_id(1);
  left_lane2.set_lane_id(2);
  left_lane3.set_lane_id(3);
  right_lane1.set_lane_id(4);
  right_lane2.set_lane_id(5);
  right_lane3.set_lane_id(6);
  Ref_line.set_color(1.0, 0.0, 0.0);
  left_lane1.set_color(0.0, 1.0, 0.0);
  left_lane2.set_color(0.0, 1.0, 0.0);
  left_lane3.set_color(0.0, 0.0, 1.0);
  right_lane1.set_color(0.0, 1.0, 0.0);
  right_lane2.set_color(0.0, 1.0, 0.0);
  right_lane3.set_color(0.0, 0.0, 1.0);


  for (int i = 0; i < Config::singleton().planeview_data.size(); ++i)
  {
      geometry_msgs::Point p;
      std::map<std::string, double> planeview_dat;
      planeview_dat = Config::singleton().planeview_data[i];
      p.x = planeview_dat["x"];
      p.y = planeview_dat["y"];
      p.z = 0.0;
      Ref_line.pushback(p);
      planeview_dat.clear();
  }

  for (int i = 0; i < Config::singleton().planeview_data.size(); ++i)
  {
    geometry_msgs::Point pl1, pl2, pl3;
    #pragma omp critical
    {
    pl1.x=x1[i];
    pl1.y=y1[i];
    pl1.z=0.0;
    pl2.x=x2[i];
    pl2.y=y2[i];
    pl2.z=0.0;
    pl3.x=x3[i];
    pl3.y=y3[i];
    pl3.z=0.0;

    left_lane1.pushback(pl1);
    left_lane2.pushback(pl2);
    left_lane3.pushback(pl3);
    }
  }

  for (int i = 0; i < Config::singleton().planeview_data.size(); ++i)
  {
    geometry_msgs::Point pr1, pr2, pr3;
    #pragma omp critical
    {
    pr1.x=xr1[i];
    pr1.y=yr1[i];
    pr1.z=0.0;
    pr2.x=xr2[i];
    pr2.y=yr2[i];
    pr2.z=0.0;
    pr3.x=xr3[i];
    pr3.y=yr3[i];
    pr3.z=0.0;

    right_lane1.pushback(pr1);
    right_lane2.pushback(pr2);
    right_lane3.pushback(pr3);
    }
  }

  marker_pub.publish(Ref_line.get_marker());
  marker_pub.publish(left_lane1.get_marker());
  marker_pub.publish(left_lane2.get_marker());
  marker_pub.publish(left_lane3.get_marker());
  marker_pub.publish(right_lane1.get_marker());
  marker_pub.publish(right_lane2.get_marker());
  marker_pub.publish(right_lane3.get_marker());

  while (ros::ok())
  {
    visualization_msgs::Marker cube_marker;
    sensor_msgs::CameraInfo camera_info_msg;
    cube_marker.header.stamp = camera_info_msg.header.stamp = ros::Time::now();
    cube_marker.header.frame_id = camera_info_msg.header.frame_id = "map";

    cube_marker.ns = "road_visualization";  
    cube_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    cube_marker.action = visualization_msgs::Marker::ADD; 
    cube_marker.id = 7;

    camera_info_msg.distortion_model = "equidistant";
    camera_info_msg.P[3] = 4808.0;  // Set the initial x position
    camera_info_msg.P[7] = 3099.0;  // Set the initial y position
    camera_info_msg.P[11] = 0.0; // Set the initial z position

    // Eigen::Quaterniond rotation_quaternion(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())); // Example: Rotate around X-axis by pi radians
    cube_marker.mesh_resource = "file:///home/maanz/Downloads/Audi_Q7_2009.stl";
    cube_marker.mesh_use_embedded_materials = true;
    cube_marker.pose.position.x = x1[t];
    cube_marker.pose.position.y = y1[t];
    cube_marker.pose.position.z = 0.0;
    cube_marker.scale.x = 1.0;
    cube_marker.scale.y = 1.0;
    cube_marker.scale.z = 1.0;

    // cube_marker.color.r = 1.0;
    // cube_marker.color.g = 0.0;
    // cube_marker.color.b = 0.0;
    // cube_marker.color.a = 1.0;

    cube_marker.pose.orientation.x = 0.0;
    cube_marker.pose.orientation.y = 0.0;
    cube_marker.pose.orientation.z = 0.0;
    cube_marker.pose.orientation.w = 1.0;
    
    camera_info_pub.publish(camera_info_msg);

    marker_pub.publish(cube_marker);
    
    if (t < x1.size())
      t++;
    
    r.sleep();
    f+= 0.04;
  }
}
