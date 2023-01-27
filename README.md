# traffic_map
To test the Data visualization on Robot operating System(ROS).



void fill_lane_containers(std::vector <std::map <std::string, double> > &left_lane_one, std::vector<double> &right_lane_X1, std::vector<double> &right_lane_Y1){
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


        // for (int i = 0; i < (left_lane_one.size()-1); i+=2){
        for (int i = 0; i < left_lane_one.size(); i++){
            std::map <std::string, double> data = left_lane_one[i];
            if(planeview_dat["s"] == data["sOffset"]){
                pol = data["a"] + (data["b"]*planeview_dat["length"]) + (data["c"]*pow(2,planeview_dat["length"])) + (data["d"]*pow(3,planeview_dat["length"]));
                break;
            }
            // std::map <std::string, double> data1 = left_lane_one[i+1];
            // if(planeview_dat["s"] == data1["sOffset"]){
            //     pol = data1["a"] + (data1["b"]*planeview_dat["length"]) + (data1["c"]*pow(2,planeview_dat["length"])) + (data1["d"]*pow(3,planeview_dat["length"]));
            //     break;
            // }
        }

        right_lane_X1.push_back(planeview_dat["x"] + pol*cos(planeview_dat["hdg"]+(270*(PI/180))));
        right_lane_Y1.push_back(planeview_dat["y"] + pol*sin(planeview_dat["hdg"]+(270*(PI/180))));
    }
}
