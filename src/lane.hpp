#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class MapLanes 
{
private:
    double marker_lifetime = 0;
public:
    visualization_msgs::Marker line;
    
    MapLanes();
    void set_base_attributes();
    void set_visual_attributes();
    void set_color(double, double, double);
    void set_lane_id(int);
    visualization_msgs::Marker get_marker();
    void pushback(geometry_msgs::Point);
};
