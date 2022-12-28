cmake_minimum_required(VERSION 3.0.2)
project(using_markers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

add_library(parser src/parser.hpp src/parser.cpp src/map_processor.h src/map_processor.cpp src/types.h)
find_package(pugixml REQUIRED)

if(pugixml_FOUND)
    message("PUGIXML Found...........................!")
    target_link_libraries(parser pugixml::pugixml)
else()
    message("PUGIXML Not Found...........................!")
endif()

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message("OpenMP Found...........................!")
    target_link_libraries(parser OpenMP::OpenMP_CXX)
else()
    message("OpenMP Not Found...........................!")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(road_visualization src/road_visualization.cpp src/parser.hpp src/parser.cpp src/map_processor.h src/map_processor.cpp)

 target_link_libraries(road_visualization parser
   ${catkin_LIBRARIES}
 )
