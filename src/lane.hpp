#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

class MapLane 
{
private:
    static int lane_id;
public:
    visualization_msgs::msg::Marker line;
    
    MapLane();
    void set_base_attributes();
    void set_visual_attributes();
    void set_lane_id(int);
    visualization_msgs::msg::Marker get_marker() const;
    void pushback(geometry_msgs::msg::Point);

    virtual void set_color();
};

class SideLane final : public MapLane {
public:
    SideLane();
    void set_color() override;
};

class BorderLane: public MapLane {
public:
    BorderLane();
    void set_color() override;
};
