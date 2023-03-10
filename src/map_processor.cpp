#include "map_processor.h"

void fill_lane_containers(std::vector <std::map <std::string, double> > &right_lane_one, std::vector<double> &right_lane_X, std::vector<double> &right_lane_Y, int lane_num){
    double pol = 0.0;
    for (int lane_data = 0; lane_data < Config::singleton().planeview_data.size(); lane_data++)
    {
        std::map<std::string, double> planeview_dat = Config::singleton().planeview_data[lane_data];
        // int kk = 0;
        // std::map <std::string, double> data = left_lane_one[kk];
        // if(planeview_dat["s"] == data["sOffset"]){
        //     double pol = data["a"] + (data["b"]*planeview_dat["length"]) + (data["c"]*pow(2,planeview_dat["length"])) + (data["d"]*pow(3,planeview_dat["length"]));
        //     kk++;
        // }        


        for (int i = 0; i < (right_lane_one.size()-1); i+=2){
            std::map <std::string, double> data = right_lane_one[i];
            if(planeview_dat["s"] == data["sOffset"]){
                pol = data["a"] + (data["b"]*planeview_dat["length"]) + (data["c"]*pow(2,planeview_dat["length"])) + (data["d"]*pow(3,planeview_dat["length"]));
                break;
            }
            std::map <std::string, double> data1 = right_lane_one[i+1];
            if(planeview_dat["s"] == data1["sOffset"]){
                pol = data1["a"] + (data1["b"]*planeview_dat["length"]) + (data1["c"]*pow(2,planeview_dat["length"])) + (data1["d"]*pow(3,planeview_dat["length"]));
                break;
            }
        }

        if (lane_num == 1 ) {
            right_lane_X.push_back(planeview_dat["x"] + pol*cos(planeview_dat["hdg"]+(270*(PI/180))));
            right_lane_Y.push_back(planeview_dat["y"] + pol*sin(planeview_dat["hdg"]+(270*(PI/180))));
        }
        if (lane_num == 2 ) {
            right_lane_X.push_back(planeview_dat["x"] + pol*cos(planeview_dat["hdg"]+(270*(PI/180))));
            right_lane_Y.push_back(planeview_dat["y"] + pol*sin(planeview_dat["hdg"]+(270*(PI/180))));
        }
    }
}

map_process::map_process(){
    std::map<int, std::vector <std::map <std::string, double> > > mmm = Config::singleton().left_lanes_frames;
    // std::map<std::vector <std::map <std::string, double>>> mmm = Config::singleton().left_lanes_frames;
    
    std::vector <std::map <std::string, double> > right_lane_one = mmm[1];
    std::vector <std::map <std::string, double> > right_lane_two = mmm[2];
    std::vector<double> right_lane_X1, right_lane_Y1, right_lane_X2, right_lane_Y2;
    fill_lane_containers(right_lane_one, 
                         right_lane_X1,
                         right_lane_Y1,
                         1);
    fill_lane_containers(right_lane_two, 
                        right_lane_X2,
                        right_lane_Y2,
                        2);

    right_lane.insert(std::pair<std::string, std::vector<double>>("x1", right_lane_X1));
    right_lane.insert(std::pair<std::string, std::vector<double>>("y1", right_lane_Y1));
    right_lane.insert(std::pair<std::string, std::vector<double>>("x2", right_lane_X2));
    right_lane.insert(std::pair<std::string, std::vector<double>>("y2", right_lane_Y2));
    right_lane_X1.clear();
    right_lane_Y1.clear();
    right_lane_X2.clear();
    right_lane_Y2.clear();
}

std::map< std::string, std::vector<double>> map_process::get_right_lane(int lane_number){
    if(lane_number == 1)
    return right_lane;
}
