#include "parser.hpp"
#include <cmath>

const double PI {3.1415926};                                 // DECLARED CONSTANTS  

class map_process
{
private:
    //std::map< std::string, std::vector<double>> right_lane;
public:
    std::map< std::string, std::vector<double>> right_lane;
    map_process();
    std::map< std::string, std::vector<double>> get_right_lane(int lane_number);
};