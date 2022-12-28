#include "map_processor.h"

map_process::map_process(){
    std::map<int, std::vector <std::map <std::string, double> > > mmm = Config::singleton().left_lanes_frames;
    // std::map<std::vector <std::map <std::string, double>>> mmm = Config::singleton().left_lanes_frames;
    
    std::vector <std::map <std::string, double> > left_lane_one = mmm[1];
    std::vector<double> right_lane_X1,right_lane_Y1;
    double pol = 0.0;
    // int kk = 0;

    for (int lane_data = 0; lane_data < Config::singleton().planeview_data.size(); lane_data++)
    {
        std::map<std::string, double> planeview_dat = Config::singleton().planeview_data[lane_data];

        // std::map <std::string, double> data = left_lane_one[kk];
        // if(planeview_dat["s"] == data["sOffset"]){
        //     double pol = data["a"] + (data["b"]*planeview_dat["length"]) + (data["c"]*pow(2,planeview_dat["length"])) + (data["d"]*pow(3,planeview_dat["length"]));
        //     kk++;
        // }        


        for (int i = 0; i < left_lane_one.size(); i++){
            std::map <std::string, double> data = left_lane_one[i];
            if(planeview_dat["s"] == data["sOffset"]){
                pol = data["a"] + (data["b"]*planeview_dat["length"]) + (data["c"]*pow(2,planeview_dat["length"])) + (data["d"]*pow(3,planeview_dat["length"]));
                break;
            }
        }

        right_lane_X1.push_back(planeview_dat["x"] + pol*cos(planeview_dat["hdg"]+(270*(PI/180))));
        right_lane_Y1.push_back(planeview_dat["y"] + pol*sin(planeview_dat["hdg"]+(270*(PI/180))));
    }
    right_lane.insert(std::pair<std::string, std::vector<double>>("x1", right_lane_X1));
    right_lane.insert(std::pair<std::string, std::vector<double>>("y1", right_lane_Y1));
    right_lane_X1.clear();
    right_lane_Y1.clear();
}

std::map< std::string, std::vector<double>> map_process::get_right_lane(int lane_number){
    if(lane_number == 1)
    return right_lane;
}