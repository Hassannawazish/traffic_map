# traffic_map
To test the Data visualization on Robot operating System(ROS).
This repository provides the Vehicle clonning and simulation on XODR map using different models.

Rapidly exploring Random Tree (RRT) are used to spawn ego and multiple vehicles on open drive's XODR map. The script is in C++ language which uses shell script to run all steps in one go. The Pugixml C++ library is used to parse the XML typed XODR map. Spawned vehicles on the road using transformations and lane attributes. Filled the data on marker array and use rviz to visualize the vehicle , Audi A5's model has been used and provided the Bag generation option as well to visualize the simulation on Foxglove visualizer.


# Record DATA in BAG files
rosbag record -O markers_data.bag

For shell script : 
                  roscore & ./src/traffic_xodr/build/road_visualization -fopenmp & sleep 20s rosbag record -a -O my_data.bag
                  rosnode kill -a
