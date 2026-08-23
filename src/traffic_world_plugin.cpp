#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

namespace gazebo {
class TrafficWorldPlugin : public WorldPlugin {
  physics::WorldPtr world_; event::ConnectionPtr update_; std::vector<ignition::math::Vector2d> route_;
  std::vector<double> arc_,distance_,speed_,desired_speed_,direction_,lateral_; common::Time last_;
  std::vector<std::size_t> lane_;std::vector<bool> crashed_,physics_released_;
  std::vector<double> pedestrian_distance_,pedestrian_speed_,pedestrian_direction_,pedestrian_lateral_;
  ignition::math::Vector2d sample(double s) const {s=std::fmod(s,arc_.back());if(s<0)s+=arc_.back();auto it=std::upper_bound(arc_.begin(),arc_.end(),s);
    std::size_t b=std::min<std::size_t>(std::distance(arc_.begin(),it),arc_.size()-1),a=b-1; double t=(s-arc_[a])/(arc_[b]-arc_[a]); return route_[a]+(route_[b]-route_[a])*t;}
 public:void Load(physics::WorldPtr world,sdf::ElementPtr) override {world_=world; const char *path=std::getenv("TRAFFIC_ROUTE_CSV"); if(!path)return;
    const char *car_mesh=std::getenv("TRAFFIC_CAR_MESH"); if(!car_mesh)return;
    std::ifstream in(path);std::string line;while(std::getline(in,line)){std::replace(line.begin(),line.end(),',',' ');std::istringstream s(line);double x,y;if(s>>x>>y)route_.emplace_back(x,y);} if(route_.size()<2)return;
    arc_.assign(route_.size(),0);for(std::size_t i=1;i<route_.size();++i)arc_[i]=arc_[i-1]+route_[i].Distance(route_[i-1]);
    std::size_t count=8;
    if(const char *value=std::getenv("TRAFFIC_VEHICLE_COUNT")){
      try { count=std::clamp<std::size_t>(std::stoul(value),1,16); }
      catch(const std::exception &) {}
    }
    const std::array<std::array<double,3>,12> colors{{
      {{0.85,0.05,0.03}},{{0.03,0.25,0.90}},{{0.05,0.70,0.18}},{{0.95,0.65,0.02}},
      {{0.55,0.08,0.75}},{{0.02,0.75,0.75}},{{0.95,0.30,0.55}},{{0.90,0.90,0.90}},
      {{0.95,0.35,0.02}},{{0.15,0.15,0.18}},{{0.45,0.75,0.05}},{{0.55,0.30,0.12}}
    }};
    distance_.resize(count);speed_.resize(count);desired_speed_.resize(count);direction_.resize(count);lateral_.resize(count);lane_.resize(count);crashed_.assign(count,false);physics_released_.assign(count,false);
    for(std::size_t i=0;i<count;++i){
      const std::size_t lane=i==0?0:(i-1)%3;
      distance_[i]=i==0?40.:55.+(i-1)*12.;speed_[i]=i==0?20.:6.+.7*i;desired_speed_[i]=speed_[i];direction_[i]=1.;lateral_[i]=3.2*(lane+1);lane_[i]=lane;
      auto initial=sample(distance_[i]),ahead=sample(distance_[i]+8);double initial_yaw=std::atan2(ahead.Y()-initial.Y(),ahead.X()-initial.X());
      double initial_x=initial.X()+std::sin(initial_yaw)*lateral_[i],initial_y=initial.Y()-std::cos(initial_yaw)*lateral_[i];
      std::ostringstream sdf;sdf<<"<sdf version='1.6'><model name='traffic_vehicle_"<<i<<"'><pose>"<<initial_x<<" "<<initial_y<<" .75 0 0 "<<initial_yaw<<"</pose><static>false</static><link name='body'>"
        <<"<inertial><mass>2100</mass><inertia><ixx>900</ixx><iyy>3200</iyy><izz>3500</izz></inertia></inertial>";
      const auto &color=colors[i==0?0:i%colors.size()];
      sdf<<"<visual name='visual'><pose>0 0 -0.85 1.570796 0 1.570796</pose><geometry><mesh><uri>file://"<<car_mesh<<"</uri></mesh></geometry>"
        <<"<material><ambient>"<<color[0]<<" "<<color[1]<<" "<<color[2]<<" 1</ambient>"
        <<"<diffuse>"<<color[0]<<" "<<color[1]<<" "<<color[2]<<" 1</diffuse></material></visual>";
      if(i==0){
        sdf<<"<visual name='dashboard_sensor_housing'><pose>1.15 0 .72 0 0 0</pose><geometry><box><size>.16 .32 .12</size></box></geometry>"
          <<"<material><ambient>.03 .03 .03 1</ambient><diffuse>.06 .06 .06 1</diffuse></material></visual>"
          <<"<sensor name='dashboard_lidar' type='ray'><pose>2.75 0 .72 0 0 0</pose><always_on>true</always_on><visualize>true</visualize><update_rate>20</update_rate>"
          <<"<ray><scan><horizontal><samples>1080</samples><resolution>1</resolution><min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal></scan>"
          <<"<range><min>.20</min><max>150</max><resolution>.02</resolution></range><noise><type>gaussian</type><mean>0</mean><stddev>.01</stddev></noise></ray>"
          <<"<plugin name='dashboard_lidar_ros' filename='libgazebo_ros_ray_sensor.so'><ros><namespace>/ego/dashboard_lidar</namespace><remapping>~/out:=scan</remapping></ros>"
          <<"<output_type>sensor_msgs/LaserScan</output_type><frame_name>dashboard_lidar</frame_name></plugin></sensor>"
          <<"<sensor name='dashboard_camera' type='camera'><pose>1.26 0 .84 0 0 0</pose><always_on>true</always_on><visualize>false</visualize><update_rate>20</update_rate>"
          <<"<camera><horizontal_fov>1.570796</horizontal_fov><image><width>640</width><height>360</height><format>R8G8B8</format></image>"
          <<"<clip><near>.10</near><far>100</far></clip></camera>"
          <<"<plugin name='dashboard_camera_ros' filename='libgazebo_ros_camera.so'><ros><namespace>/ego/dashboard_camera</namespace></ros>"
          <<"<camera_name>dashboard</camera_name><frame_name>dashboard_camera_optical</frame_name></plugin></sensor>";
      }
      sdf<<"<collision name='collision'><geometry><box><size>4.5 2 1.5</size></box></geometry></collision>"
        <<"</link></model></sdf>";world_->InsertModelString(sdf.str());}
    constexpr std::size_t pedestrian_count=12;
    pedestrian_distance_.resize(pedestrian_count);pedestrian_speed_.resize(pedestrian_count);
    pedestrian_direction_.resize(pedestrian_count);pedestrian_lateral_.resize(pedestrian_count);
    for(std::size_t i=0;i<pedestrian_count;++i){
      const auto &shirt=colors[(i+3)%colors.size()];
      pedestrian_distance_[i]=120.+i*190.;pedestrian_speed_[i]=1.1+.08*i;
      pedestrian_direction_[i]=(i%2)?-1.:1.;pedestrian_lateral_[i]=(i%2?1.:-1.)*(14.4+.15*(i%3));
      std::ostringstream sdf;
      sdf<<"<sdf version='1.6'><model name='pedestrian_"<<i<<"'><static>true</static><link name='body'>"
        <<"<visual name='head'><pose>0 0 1.72 0 0 0</pose><geometry><sphere><radius>.16</radius></sphere></geometry><material><diffuse>.72 .48 .34 1</diffuse></material></visual>"
        <<"<visual name='torso'><pose>0 0 1.15 0 0 0</pose><geometry><cylinder><radius>.22</radius><length>.82</length></cylinder></geometry><material><diffuse>"
        <<shirt[0]<<" "<<shirt[1]<<" "<<shirt[2]<<" 1</diffuse></material></visual>"
        <<"<visual name='leg_left'><pose>0 .11 .43 0 0 0</pose><geometry><cylinder><radius>.075</radius><length>.72</length></cylinder></geometry><material><diffuse>.08 .09 .12 1</diffuse></material></visual>"
        <<"<visual name='leg_right'><pose>0 -.11 .43 0 0 0</pose><geometry><cylinder><radius>.075</radius><length>.72</length></cylinder></geometry><material><diffuse>.08 .09 .12 1</diffuse></material></visual>"
        <<"</link></model></sdf>";world_->InsertModelString(sdf.str());
    }
    last_=world_->SimTime();update_=event::Events::ConnectWorldUpdateBegin(std::bind(&TrafficWorldPlugin::OnUpdate,this));}
  void OnUpdate(){auto now=world_->SimTime();double dt=(now-last_).Double();if(dt<1.0/120.0)return;last_=now;
    for(std::size_t i=0;i<distance_.size();++i){if(crashed_[i])continue;double target=desired_speed_[i];
      for(std::size_t j=0;j<distance_.size();++j){if(i==j||crashed_[j]||lane_[i]!=lane_[j])continue;
        double gap=std::fmod(distance_[j]-distance_[i]+arc_.back(),arc_.back());
        if(gap<55.)target=std::min(target,std::max(0.,(gap-5.)*.55));
        if(gap<4.7){crashed_[i]=true;crashed_[j]=true;}}
      const double change=target-speed_[i];speed_[i]+=std::clamp(change,-9.*dt,2.2*dt);
    }
    for(std::size_t i=0;i<distance_.size();++i){auto model=world_->ModelByName("traffic_vehicle_"+std::to_string(i));if(!model)continue;
      if(crashed_[i]){auto p=sample(distance_[i]),q=sample(distance_[i]+8);double yaw=std::atan2(q.Y()-p.Y(),q.X()-p.X());
        if(!physics_released_[i]){model->SetLinearVel({std::cos(yaw)*speed_[i],std::sin(yaw)*speed_[i],0});model->SetAngularVel({0,0,(i%2?.35:-.35)});physics_released_[i]=true;}continue;}
      if(i>0){double gap=std::fmod(distance_[i]-distance_[0]+arc_.back(),arc_.back());
        const double behind=arc_.back()-gap;
        if(gap>arc_.back()*.5&&behind>100.)distance_[i]=std::fmod(distance_[0]+35.+(i-1)*18.,arc_.back());}
      auto p=sample(distance_[i]),q=sample(distance_[i]+direction_[i]*8);double yaw=std::atan2(q.Y()-p.Y(),q.X()-p.X());
      model->SetWorldPose({p.X()+std::sin(yaw)*lateral_[i],p.Y()-std::cos(yaw)*lateral_[i],.75,0,0,yaw});
      distance_[i]=std::fmod(distance_[i]+direction_[i]*speed_[i]*dt,arc_.back());if(distance_[i]<0)distance_[i]+=arc_.back();}
    for(std::size_t i=0;i<pedestrian_distance_.size();++i){auto model=world_->ModelByName("pedestrian_"+std::to_string(i));if(!model)continue;
      auto p=sample(pedestrian_distance_[i]),q=sample(pedestrian_distance_[i]+pedestrian_direction_[i]*2);double yaw=std::atan2(q.Y()-p.Y(),q.X()-p.X());
      model->SetWorldPose({p.X()+std::sin(yaw)*pedestrian_lateral_[i],p.Y()-std::cos(yaw)*pedestrian_lateral_[i],.08,0,0,yaw});
      pedestrian_distance_[i]=std::fmod(pedestrian_distance_[i]+pedestrian_direction_[i]*pedestrian_speed_[i]*dt,arc_.back());if(pedestrian_distance_[i]<0)pedestrian_distance_[i]+=arc_.back();}}
};GZ_REGISTER_WORLD_PLUGIN(TrafficWorldPlugin)
}
