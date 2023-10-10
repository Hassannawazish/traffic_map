#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/CameraInfo.h>

#include <fstream>
#include <random>

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include "map_processor.h"
#include "types.h"

extern const int procs;

int main(int argc, char** argv )
{  
  ros::init(argc, argv, "road_visualization");
  ros::NodeHandle n;
  ros::Rate r(30);
  ros::Publisher camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  float f = 0.0;
  
  Config::singleton().parse();
  std::cout<<"Total Number of thread = "<<procs<<std::endl;
  std::cout<<"lanes = "<<Config::singleton().num_of_lanes<<std::endl;
  std::cout<<"No. of Geometeries = "<< *Config::singleton().num_of_geometeries<<std::endl; 
  std::cout<<"Road Length = "<<Config::singleton().road_length<<std::endl;
  map_process processed;

  double *x1 {nullptr};
  x1 = new double [processed.get_lane(1)["x1"].size()];
  for (int i = 0; i < processed.get_lane(1)["x1"].size(); i++)
    x1[i] = processed.get_lane(1)["x1"][i];
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

  visualization_msgs::Marker moving_cube;
  moving_cube.header.frame_id = "map";
  moving_cube.header.stamp = ros::Time::now();
  moving_cube.ns = "road_visualization";
  moving_cube.type = visualization_msgs::Marker::CUBE;
  moving_cube.action = visualization_msgs::Marker::ADD;
  moving_cube.id = 7;  // Make sure the ID is unique
  moving_cube.scale.x = 1.0;
  moving_cube.scale.y = 1.0;
  moving_cube.scale.z = 1.0;
  moving_cube.color.r = 1.0;
  moving_cube.color.g = 0.0;
  moving_cube.color.b = 0.0;
  moving_cube.color.a = 1.0;
  moving_cube.pose.orientation.w = 1.0;

  while (ros::ok())
  {
    visualization_msgs::Marker Ref_line, left_lane1, left_lane2, left_lane3, right_lane1, right_lane2, right_lane3;
    sensor_msgs::CameraInfo camera_info_msg;
    camera_info_msg.header.stamp = ros::Time::now();
    camera_info_msg.header.frame_id = "map";

    right_lane3.header.frame_id = right_lane2.header.frame_id = right_lane1.header.frame_id = left_lane3.header.frame_id = left_lane2.header.frame_id = left_lane1.header.frame_id = Ref_line.header.frame_id= "map";  
    right_lane3.header.stamp = right_lane2.header.stamp = right_lane1.header.stamp = left_lane3.header.stamp = left_lane2.header.stamp = left_lane1.header.stamp = Ref_line.header.stamp = ros::Time::now();   
    right_lane3.ns = right_lane2.ns = right_lane1.ns = left_lane3.ns = left_lane2.ns = left_lane1.ns = Ref_line.ns="road_visualization";  
    Ref_line.type = visualization_msgs::Marker::LINE_STRIP;
    right_lane3.type = right_lane2.type = right_lane1.type = left_lane3.type = left_lane2.type = left_lane1.type = visualization_msgs::Marker::LINE_STRIP; 
    right_lane3.action = right_lane2.action = right_lane1.action = left_lane3.action = left_lane2.action = left_lane1.action = Ref_line.action = visualization_msgs::Marker::ADD;
    Ref_line.id = 0;
    left_lane1.id = 1;
    left_lane2.id = 2;
    left_lane3.id = 3;
    right_lane1.id = 4;
    right_lane2.id = 5;
    right_lane3.id = 6;
    Ref_line.pose.orientation.w = 2.0;//stod(Config::singleton().center_lane_dimentions_rm["width"]);
    //std::map <std::string, std::string> rm_data = stod(Config::singleton().left_lane_dimentions_rm[3]);
    //right_lane1.pose.orientation.w = rm_data["width"];
    left_lane3.pose.orientation.w = left_lane2.pose.orientation.w = left_lane1.pose.orientation.w = 1.0;
    right_lane3.pose.orientation.w = right_lane2.pose.orientation.w = right_lane1.pose.orientation.w = 1.0;
    right_lane3.scale.x  = right_lane2.scale.x  = right_lane1.scale.x  = left_lane3.scale.x  = left_lane2.scale.x  = left_lane1.scale.x  = Ref_line.scale.x = 0.1;
    right_lane3.scale.y = right_lane2.scale.y = right_lane1.scale.y  = left_lane3.scale.y = left_lane2.scale.y = left_lane1.scale.y  = Ref_line.scale.y = 0.1;
    right_lane3.scale.z = right_lane2.scale.z = right_lane1.scale.z = left_lane3.scale.z = left_lane2.scale.z = left_lane1.scale.z = Ref_line.scale.z = 0;
    
    right_lane3.color.a = right_lane2.color.a = right_lane1.color.a = left_lane3.color.a = left_lane2.color.a = left_lane1.color.a = Ref_line.color.a = 1.0;

    Ref_line.color.r = 1.0;
    Ref_line.color.g = 0.0;
    Ref_line.color.b = 0.0;

    left_lane1.color.r = 0.0;
    left_lane1.color.g = 1.0;
    left_lane1.color.b = 0.0;

    left_lane2.color.r = 0.0;
    left_lane2.color.g = 1.0;
    left_lane2.color.b = 0.0;

    left_lane3.color.r = 0.0;
    left_lane3.color.g = 0.0;
    left_lane3.color.b = 1.0;

    right_lane1.color.r = 0.0;
    right_lane1.color.g = 1.0;
    right_lane1.color.b = 0.0;

    right_lane2.color.r = 0.0;
    right_lane2.color.g = 1.0;
    right_lane2.color.b = 0.0;

    right_lane3.color.r = 0.0;
    right_lane3.color.g = 0.0;
    right_lane3.color.b = 1.0;

    // sensor_msgs::CameraInfo camera_info_msg;
    //     camera_info_msg.header.stamp = ros::Time::now();
    //     camera_info_msg.width = 640;
    //     camera_info_msg.height = 480;
    //     camera_info_msg.K[0] = 4808.0;  // fx
    //     camera_info_msg.K[4] = 3099.0;  // fy
    //     camera_info_msg.K[2] = camera_info_msg.width / 2.0;  // cx
    //     camera_info_msg.K[5] = camera_info_msg.height / 2.0;  // cy
    //     camera_info_msg.distortion_model = "equidistant";
    camera_info_msg.P[3] = 4808.0;  // Set the initial x position
    camera_info_msg.P[7] = 3099.0;  // Set the initial y position
    camera_info_msg.P[11] = 0.0; // Set the initial z position

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
      geometry_msgs::Point pl1, pl2, pl3;
      #pragma omp critical
      {
      pl1.x = *(x1 + i);               // subscript/offset notaions to access pointer array data      name[index] / *(name + index) 
      pl1.y = y1[i];
      pl1.z = 0.0;
      pl2.x = x2[i];
      pl2.y = y2[i];
      pl2.z = 0.0;
      pl3.x = x3[i];
      pl3.y = y3[i];
      pl3.z = 0.0;

      left_lane1.points.push_back(pl1);
      left_lane2.points.push_back(pl2);
      left_lane3.points.push_back(pl3);
      }
    }

    for (int i = 0; i < Config::singleton().planeview_data.size(); ++i){
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

      right_lane1.points.push_back(pr1);
      right_lane2.points.push_back(pr2);
      right_lane3.points.push_back(pr3);
      }
    }

    moving_cube.pose.position.x = 4808;
    moving_cube.pose.position.y = 3099;
    moving_cube.pose.position.z = 0.5;
    
    camera_info_pub.publish(camera_info_msg);
    marker_pub.publish(Ref_line);
    marker_pub.publish(left_lane1);
    marker_pub.publish(left_lane2);
    marker_pub.publish(left_lane3);
    marker_pub.publish(right_lane1);
    marker_pub.publish(right_lane2);
    marker_pub.publish(right_lane3);
    marker_pub.publish(moving_cube);

      r.sleep();
      f+= 0.04;
  }
  delete [] x1;
}
