# traffic_map
To test the Data visualization on Robot operating System(ROS).





    // for (int i = 0; i < Config::singleton().planeview_data.size(); ++i)
    //   {
    //       geometry_msgs::Point p;
    //       std::map<std::string, double> planeview_dat;
    //       planeview_dat = Config::singleton().planeview_data[i];
    //       p.x = planeview_dat["x"];
    //       p.y = planeview_dat["y"];
    //       p.z = 0.0;
    //       Ref_line.points.push_back(p);
    //       planeview_dat.clear();
    //   }
    // #pragma omp parallel for num_threads(4)
    // for (int i = 0; i < Config::singleton().planeview_data.size(); ++i){
    //       geometry_msgs::Point pl1;
    //       #pragma omp critical
    //       {
    //       pl1.x=x1[i];
    //       pl1.y=y1[i];
    //       pl1.z=0.0;
    //       left_lane1.points.push_back(pl1);
    //       }
    //   }
