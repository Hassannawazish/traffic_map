
mkdir -p ~/catkin_ws/src
# source /opt/ros/noetic/setup.bash
# cd ~/catkin_ws/
# catkin_make
# catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
# source devel/setup.bash
# echo $ROS_PACKAGE_PATH

# cd src
# roscreate-pkg traffic_xodr roscpp visualization_msgs

# if [ -d "/usr/local/lib/cmake/pugixml" ] 
# then
#     echo "Directory Pugixml exists." 
# else
#     echo "Error: Directory Pugixml does not exists."
#     echo "Installing PUGIXML"
#     git clone https://github.com/zeux/pugixml.git
#     cd pugixml
#     mkdir build
#     cd build
#     cmake ..
#     sudo make install
# fi

# cd traffic_xodr
# rm -rf include 
# rm -rf CMakeLists.txt
# CMAKE_PATH=$PWD
# cd src/
# C_CODE_PATH=$PWD
# echo ${C_CODE_PATH}
# cd ../../..
# pwd
# cd ..
# cd traffic_map
# cp CMakeLists.txt ${CMAKE_PATH}
# cd src
# cp bjb_sangjani_full.xodr ${C_CODE_PATH}
# cp road_visualization.cpp ${C_CODE_PATH}
# cp parser.hpp ${C_CODE_PATH}
# cp parser.cpp ${C_CODE_PATH}
# cp map_processor.h ${C_CODE_PATH}
# cp map_processor.cpp ${C_CODE_PATH}
# cp types.h ${C_CODE_PATH}
# cd ../..
# pwd
# cd catkin_ws
# source devel/setup.bash
# catkin_make
# rosmake traffic_xodr
# FILEPATH="${C_CODE_PATH}/bjb_sangjani_full.xodr" 
# export FILEPATH
# echo $FILEPATH

# roscore & ./src/traffic_xodr/build/road_visualization -fopenmp & rviz rviz

