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
  std::vector<double> arc_,distance_,speed_,direction_,lateral_; common::Time last_;
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
    distance_.resize(count);speed_.resize(count);direction_.resize(count);lateral_.resize(count);
    for(std::size_t i=0;i<count;++i){
      const bool incoming=(i%2)==1;
      const std::size_t lane=(i/2)%3;
      distance_[i]=40.+(i/2)*110.;speed_[i]=20.+2.*i;direction_[i]=incoming?-1.:1.;lateral_[i]=3.2*(lane+1);
      std::ostringstream sdf;sdf<<"<sdf version='1.6'><model name='traffic_vehicle_"<<i<<"'><static>true</static><link name='body'>";
      const auto &color=colors[i==0?0:i%colors.size()];
      sdf<<"<visual name='visual'><pose>0 0 -0.85 1.570796 0 1.570796</pose><geometry><mesh><uri>file://"<<car_mesh<<"</uri></mesh></geometry>"
        <<"<material><ambient>"<<color[0]<<" "<<color[1]<<" "<<color[2]<<" 1</ambient>"
        <<"<diffuse>"<<color[0]<<" "<<color[1]<<" "<<color[2]<<" 1</diffuse></material></visual>";
      sdf<<"<collision name='collision'><geometry><box><size>4.5 2 1.5</size></box></geometry></collision>"
        <<"</link></model></sdf>";world_->InsertModelString(sdf.str());}
    last_=world_->SimTime();update_=event::Events::ConnectWorldUpdateBegin(std::bind(&TrafficWorldPlugin::OnUpdate,this));}
  void OnUpdate(){auto now=world_->SimTime();double dt=(now-last_).Double();if(dt<1.0/30.0)return;last_=now;for(std::size_t i=0;i<distance_.size();++i){auto model=world_->ModelByName("traffic_vehicle_"+std::to_string(i));if(!model)continue;
      auto p=sample(distance_[i]),q=sample(distance_[i]+direction_[i]*8);double yaw=std::atan2(q.Y()-p.Y(),q.X()-p.X());
      model->SetWorldPose({p.X()+std::sin(yaw)*lateral_[i],p.Y()-std::cos(yaw)*lateral_[i],.75,0,0,yaw});
      distance_[i]=std::fmod(distance_[i]+direction_[i]*speed_[i]*dt,arc_.back());if(distance_[i]<0)distance_[i]+=arc_.back();}}
};GZ_REGISTER_WORLD_PLUGIN(TrafficWorldPlugin)
}
