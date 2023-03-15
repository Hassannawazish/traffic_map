#include "map_processor.h"

void fill_lane_containers(std::vector <std::map <std::string, double> > &left_lane_one,
                        std::vector <std::map <std::string, double> > &left_lane_two,
                        std::vector <std::map <std::string, double> > &left_lane_three,  
                        std::vector<double> &left_lane_X1, 
                        std::vector<double> &left_lane_Y1,
                        std::vector<double> &left_lane_X2, 
                        std::vector<double> &left_lane_Y2, 
                        std::vector<double> &left_lane_X3, 
                        std::vector<double> &left_lane_Y3,  
                        int lane_num){
    double pol = 0.0, pol1 = 0.0, pol2 = 0.0;
    for (int lane_data = 0; lane_data < Config::singleton().planeview_data.size(); lane_data++)
    {
        std::map<std::string, double> planeview_dat = Config::singleton().planeview_data[lane_data];

        for (int i = 0; i < (left_lane_one.size()-1); i+=2){
            std::map <std::string, double> data = left_lane_one[i];
            if(planeview_dat["s"] == data["sOffset"]){
                pol = data["a"] + (data["b"]*planeview_dat["length"]) + (data["c"]*pow(2,planeview_dat["length"])) + (data["d"]*pow(3,planeview_dat["length"]));
                break;
            }
            std::map <std::string, double> data1 = left_lane_one[i+1];
            if(planeview_dat["s"] == data1["sOffset"]){
                pol = data1["a"] + (data1["b"]*planeview_dat["length"]) + (data1["c"]*pow(2,planeview_dat["length"])) + (data1["d"]*pow(3,planeview_dat["length"]));
                break;
            }
        }

        for (int i = 0; i < (left_lane_two.size()-1); i+=2){
            std::map <std::string, double> data = left_lane_two[i];
            if(planeview_dat["s"] == data["sOffset"]){
                pol1 = data["a"] + (data["b"]*planeview_dat["length"]) + (data["c"]*pow(2,planeview_dat["length"])) + (data["d"]*pow(3,planeview_dat["length"]));
                break;
            }
            std::map <std::string, double> data1 = left_lane_two[i+1];
            if(planeview_dat["s"] == data1["sOffset"]){
                pol1 = data1["a"] + (data1["b"]*planeview_dat["length"]) + (data1["c"]*pow(2,planeview_dat["length"])) + (data1["d"]*pow(3,planeview_dat["length"]));
                break;
            }
        }

        for (int i = 0; i < (left_lane_three.size()-1); i+=2){
            std::map <std::string, double> data = left_lane_three[i];
            if(planeview_dat["s"] == data["sOffset"]){
                pol2 = data["a"] + (data["b"]*planeview_dat["length"]) + (data["c"]*pow(2,planeview_dat["length"])) + (data["d"]*pow(3,planeview_dat["length"]));
                break;
            }
            std::map <std::string, double> data1 = left_lane_three[i+1];
            if(planeview_dat["s"] == data1["sOffset"]){
                pol2 = data1["a"] + (data1["b"]*planeview_dat["length"]) + (data1["c"]*pow(2,planeview_dat["length"])) + (data1["d"]*pow(3,planeview_dat["length"]));
                break;
            }
        }

        if (lane_num == 1 ) {
            left_lane_X1.push_back(planeview_dat["x"] + pol*cos(planeview_dat["hdg"]+(270*(PI/180))));
            left_lane_Y1.push_back(planeview_dat["y"] + pol*sin(planeview_dat["hdg"]+(270*(PI/180))));

            left_lane_X2.push_back(planeview_dat["x"] + (pol + pol1)*cos(planeview_dat["hdg"]+(270*(PI/180))));
            left_lane_Y2.push_back(planeview_dat["y"] + (pol + pol1)*sin(planeview_dat["hdg"]+(270*(PI/180))));

            left_lane_X3.push_back(planeview_dat["x"] + (pol + pol1 + pol2)*cos(planeview_dat["hdg"]+(270*(PI/180))));
            left_lane_Y3.push_back(planeview_dat["y"] + (pol + pol1 + pol2)*sin(planeview_dat["hdg"]+(270*(PI/180))));
        }
    }
}

map_process::map_process(){
    std::map<int, std::vector <std::map <std::string, double> > > frameLeft = Config::singleton().left_lanes_frames;
    // std::map<int, std::vector <std::map <std::string, double> > > frameLeft = Config::singleton().left_lanes_frames;
    
    std::vector <std::map <std::string, double> > left_lane_one = frameLeft[1];
    std::vector <std::map <std::string, double> > left_lane_two = frameLeft[2];
    std::vector <std::map <std::string, double> > left_lane_three = frameLeft[2];
    std::vector<double> left_lane_X1, left_lane_Y1, left_lane_X2, left_lane_Y2, left_lane_X3, left_lane_Y3;
    fill_lane_containers(left_lane_one,
                         left_lane_two,
                         left_lane_three,
                         left_lane_X1,
                         left_lane_Y1,
                         left_lane_X2,
                         left_lane_Y2,
                         left_lane_X3,
                         left_lane_Y3,
                         1);
    // fill_lane_containers(right_lane_two, 
    //                     right_lane_X2,
    //                     right_lane_Y2,
    //                     2);

    left_lane.insert(std::pair<std::string, std::vector<double>>("x1", left_lane_X1));
    left_lane.insert(std::pair<std::string, std::vector<double>>("y1", left_lane_Y1));
    left_lane.insert(std::pair<std::string, std::vector<double>>("x2", left_lane_X2));
    left_lane.insert(std::pair<std::string, std::vector<double>>("y2", left_lane_Y2));
    left_lane.insert(std::pair<std::string, std::vector<double>>("x3", left_lane_X3));
    left_lane.insert(std::pair<std::string, std::vector<double>>("y3", left_lane_Y3));
    left_lane_X1.clear();
    left_lane_Y1.clear();
    left_lane_X2.clear();
    left_lane_Y2.clear();
    left_lane_X3.clear();
    left_lane_Y3.clear();
}

std::map< std::string, std::vector<double>> map_process::get_left_lane(int lane_number){
    if(lane_number == 1)
    return left_lane;
}
