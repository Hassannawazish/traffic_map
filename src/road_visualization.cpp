#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include "map_processor.h"
#include "types.h"
#include <random>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

extern const int procs;

int main(int argc, char** argv )
{  
  ros::init(argc, argv, "road_visualization");
  ros::NodeHandle n;
  ros::Rate r(30);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  float f = 0.0;
  
  Config::singleton().parse();
  std::cout<<"Total Number of thread = "<<procs<<std::endl;
  std::cout<<"lanes = "<<Config::singleton().num_of_lanes<<std::endl;
  std::cout<<"No. of Geometeries = "<<Config::singleton().num_of_geometeries<<std::endl; 
  std::cout<<"Road Length = "<<Config::singleton().road_length<<std::endl;
  map_process processed;
  // std::map< std::string, std::vector<double>> lanes_coords = processed.get_right_lane(1);
  std::vector< double> x1 =  processed.get_right_lane(1)["x1"];
  std::vector< double> y1 =  processed.get_right_lane(1)["y1"];
  std::vector< double> x2 =  processed.get_right_lane(1)["x2"];
  std::vector< double> y2 =  processed.get_right_lane(1)["y2"];

  while (ros::ok())
  {
    visualization_msgs::Marker Ref_line, right_lane1, right_lane2;   
    right_lane2.header.frame_id = right_lane1.header.frame_id = Ref_line.header.frame_id= "map";  
    right_lane2.header.stamp = right_lane1.header.stamp = Ref_line.header.stamp = ros::Time::now();   
    right_lane2.ns = right_lane1.ns = Ref_line.ns="road_visualization";  
    Ref_line.type = visualization_msgs::Marker::LINE_STRIP;
    right_lane2.type = right_lane1.type = visualization_msgs::Marker::LINE_STRIP; 
    right_lane2.action = right_lane1.action = Ref_line.action = visualization_msgs::Marker::ADD;
    Ref_line.id = 0;
    right_lane1.id = 1;
    right_lane2.id = 2;
    Ref_line.pose.orientation.w = 1.0;//stod(Config::singleton().center_lane_dimentions_rm["width"]);
    //std::map <std::string, std::string> rm_data = stod(Config::singleton().left_lane_dimentions_rm[3]);
    //right_lane1.pose.orientation.w = rm_data["width"];
    right_lane2.pose.orientation.w = right_lane1.pose.orientation.w = 1.0;
    right_lane2.scale.x  = right_lane1.scale.x  = Ref_line.scale.x = 0.1;
    right_lane2.scale.y = right_lane1.scale.y  = Ref_line.scale.y = 0.1;
    right_lane2.scale.z = right_lane1.scale.z = Ref_line.scale.z = 0;
    
    right_lane2.color.a = right_lane1.color.a = Ref_line.color.a = 1.0;

    Ref_line.color.r = 1.0;
    Ref_line.color.g = 0.0;
    Ref_line.color.b = 0.0;

    right_lane1.color.r = 0.0;
    right_lane1.color.g = 1.0;
    right_lane1.color.b = 0.0;

    right_lane2.color.r = 0.0;
    right_lane2.color.g = 1.0;
    right_lane2.color.b = 0.0;

    for (int i = 0; i < Config::singleton().planeview_data.size(); ++i)
      {
          geometry_msgs::Point p;
          std::map<std::string, double> planeview_dat;
          planeview_dat = Config::singleton().planeview_data[i];
          p.x = planeview_dat["x"];
          p.y = planeview_dat["y"];
          p.z = 0.0;
          Ref_line.points.push_back(p);
          planeview_dat.clear();
      }
    
    for (int i = 0; i < Config::singleton().planeview_data.size(); ++i){
      geometry_msgs::Point pr1;
      #pragma omp critical
      {
      pr1.x=x1[i];
      pr1.y=y1[i];
      pr1.z=0.0;
      right_lane1.points.push_back(pr1);
      }
    }

    for (int i = 0; i < Config::singleton().planeview_data.size(); ++i){
      geometry_msgs::Point pr2;
      #pragma omp critical
      {
      pr2.x=x2[i];
      pr2.y=y2[i];
      pr2.z=0.0;
      right_lane2.points.push_back(pr2);
      }
    }

    
    marker_pub.publish(Ref_line);
    marker_pub.publish(right_lane1);
    marker_pub.publish(right_lane2);
      r.sleep();
      f+= 0.04;
  }
}


