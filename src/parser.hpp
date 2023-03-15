#include <pugixml.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <map>
#include <deque>
#include <chrono>

class Config
{
  Config();

public:

  // Dont allow copy of singleton
  Config& operator=(const Config&) = delete;
  Config(const Config&) = delete;

  // Initialise the singleton and load data
  static void parse();

  // Get singleton
  static Config& singleton();

  // Vars ...
  double road_length;
	int num_of_lanes;
  int num_of_geometeries;
  std::map <std::string, std::string> road_specs;
  std::vector <std::map<std::string, double> > planeview_data;

//////////////////////////Left Lanes////////////////////////////////////////////
  int number_of_left_lanes, number_of_right_lanes;
  std::vector <std::map<std::string, std::string> > left_lane_attributes, left_lane_dimentions_rm;
  std::map<int, std::vector <std::map <std::string, double> > > left_lanes_frames;
  // std::vector<std::map<int, std::vector<std::map<std::string, double>> >> left_lanes_frames;

/////////////////////Center Lane ////////////////////////////////////////
  std::deque <std::map<std::string, std::string> > center_lane_attributes;
  std::map <std::string, std::string> center_lane_dimentions_rm;

//////////////////////Right Lane///////////////////////////////////////////
  std::vector <std::map<std::string, std::string> > right_lane_attributes, right_lane_dimentions_rm;
  std::map<int, std::vector <std::map <std::string, double> > > right_lanes_frames;
  // std::vector <std::vector <std::map<std::string, double> > > right_lane_dimentions;
  // std::vector<std::map<int, int> > right_lanes_frames;

};
