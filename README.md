# traffic_map
To test the Data visualization on Robot operating System(ROS).
This repository provides the Vehicle clonning and simulation on XODR map using different models.


# Record DATA in BAG files
rosbag record -O markers_data.bag

For shell script : 
                  roscore & ./src/traffic_xodr/build/road_visualization -fopenmp & sleep 20s rosbag record -a -O my_data.bag
                  rosnode kill -a
