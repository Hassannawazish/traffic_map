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
  std::vector<double> arc_,distance_,speed_,desired_speed_,direction_,lateral_,target_lateral_; common::Time last_;
  std::vector<std::size_t> lane_;std::vector<bool> crashed_,physics_released_;
  std::vector<double> lane_change_cooldown_;
  std::vector<double> signal_distance_,signal_yaw_;
  std::vector<double> pedestrian_distance_,pedestrian_speed_,pedestrian_direction_,pedestrian_lateral_;
  ignition::math::Vector2d sample(double s) const {s=std::fmod(s,arc_.back());if(s<0)s+=arc_.back();auto it=std::upper_bound(arc_.begin(),arc_.end(),s);
    std::size_t b=std::min<std::size_t>(std::distance(arc_.begin(),it),arc_.size()-1),a=b-1; double t=(s-arc_[a])/(arc_[b]-arc_[a]); return route_[a]+(route_[b]-route_[a])*t;}
 public:void Load(physics::WorldPtr world,sdf::ElementPtr) override {world_=world; const char *path=std::getenv("TRAFFIC_ROUTE_CSV"); if(!path)return;
    const char *car_mesh=std::getenv("TRAFFIC_CAR_MESH"); if(!car_mesh)return;
    std::ifstream in(path);std::string line;while(std::getline(in,line)){std::replace(line.begin(),line.end(),',',' ');std::istringstream s(line);double x,y;if(s>>x>>y)route_.emplace_back(x,y);} if(route_.size()<2)return;
    arc_.assign(route_.size(),0);for(std::size_t i=1;i<route_.size();++i)arc_[i]=arc_[i-1]+route_[i].Distance(route_[i-1]);
    std::size_t count=24;
    if(const char *value=std::getenv("TRAFFIC_VEHICLE_COUNT")){
      try { count=std::clamp<std::size_t>(std::stoul(value),1,32); }
      catch(const std::exception &) {}
    }
    const std::array<std::array<double,3>,12> colors{{
      {{0.85,0.05,0.03}},{{0.03,0.25,0.90}},{{0.05,0.70,0.18}},{{0.95,0.65,0.02}},
      {{0.55,0.08,0.75}},{{0.02,0.75,0.75}},{{0.95,0.30,0.55}},{{0.90,0.90,0.90}},
      {{0.95,0.35,0.02}},{{0.15,0.15,0.18}},{{0.45,0.75,0.05}},{{0.55,0.30,0.12}}
    }};
    distance_.resize(count);speed_.resize(count);desired_speed_.resize(count);direction_.resize(count);lateral_.resize(count);target_lateral_.resize(count);lane_.resize(count);lane_change_cooldown_.assign(count,0.);crashed_.assign(count,false);physics_released_.assign(count,false);
    for(std::size_t i=0;i<count;++i){
      const bool opposing=i>0 && i%4==0;
      const std::size_t lane_in_direction=i==0?0:(i-1)%3;
      const std::size_t lane=opposing?3+lane_in_direction:lane_in_direction;
      distance_[i]=i==0?40.:90.+(i-1)*18.;speed_[i]=i==0?35.:11.+.55*i;desired_speed_[i]=speed_[i];direction_[i]=opposing?-1.:1.;
      lateral_[i]=opposing?-3.2*(lane_in_direction+1):3.2*lane_in_direction;target_lateral_[i]=lateral_[i];lane_[i]=lane;
      auto initial=sample(distance_[i]),route_ahead=sample(distance_[i]+8),travel_ahead=sample(distance_[i]+direction_[i]*8);
      double route_yaw=std::atan2(route_ahead.Y()-initial.Y(),route_ahead.X()-initial.X());
      double initial_yaw=std::atan2(travel_ahead.Y()-initial.Y(),travel_ahead.X()-initial.X());
      double initial_x=initial.X()+std::sin(route_yaw)*lateral_[i],initial_y=initial.Y()-std::cos(route_yaw)*lateral_[i];
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
          <<"<range><min>.20</min><max>80</max><resolution>.02</resolution></range><noise><type>gaussian</type><mean>0</mean><stddev>.01</stddev></noise></ray>"
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
    constexpr std::size_t signal_count=6;
    for(std::size_t i=0;i<signal_count;++i){double d=260.+i*650.;if(d>=arc_.back())break;
      auto p=sample(d),q=sample(d+8);double yaw=std::atan2(q.Y()-p.Y(),q.X()-p.X());signal_distance_.push_back(d);signal_yaw_.push_back(yaw);
      std::ostringstream housing;housing<<"<sdf version='1.6'><model name='traffic_signal_"<<i<<"'><static>true</static><pose>"<<p.X()<<" "<<p.Y()<<" 0 0 0 "<<yaw<<"</pose><link name='signal'>"
        <<"<visual name='pole'><pose>0 -10.5 2.5 0 0 0</pose><geometry><box><size>.22 .22 5</size></box></geometry><material><diffuse>.15 .15 .15 1</diffuse></material></visual>"
        <<"<visual name='arm'><pose>0 -4.7 4.85 0 0 0</pose><geometry><box><size>.20 11.6 .20</size></box></geometry><material><diffuse>.12 .12 .12 1</diffuse></material></visual>"
        <<"<visual name='head'><pose>0 -3.2 4.45 0 0 0</pose><geometry><box><size>.30 .72 1.35</size></box></geometry><material><diffuse>.035 .035 .035 1</diffuse></material></visual>"
        <<"</link></model></sdf>";world_->InsertModelString(housing.str());
      const std::array<std::array<double,3>,3> light_colors{{{{1.,.02,.01}},{{1.,.62,.01}},{{.02,1.,.04}}}};
      for(std::size_t light=0;light<3;++light){std::ostringstream lamp;const auto &c=light_colors[light];
        lamp<<"<sdf version='1.6'><model name='traffic_signal_"<<i<<"_lamp_"<<light<<"'><static>true</static><pose>0 0 -20 0 0 0</pose><link name='lamp'>"
          <<"<visual name='visual'><geometry><sphere><radius>.17</radius></sphere></geometry><material><ambient>"<<c[0]<<" "<<c[1]<<" "<<c[2]<<" 1</ambient><diffuse>"<<c[0]<<" "<<c[1]<<" "<<c[2]<<" 1</diffuse><emissive>"<<c[0]<<" "<<c[1]<<" "<<c[2]<<" 1</emissive></material></visual>"
          <<"</link></model></sdf>";world_->InsertModelString(lamp.str());}
    }
    for(std::size_t i=0;;++i){double d=140.+i*800.;if(d>=arc_.back())break;auto p=sample(d),q=sample(d+8);double yaw=std::atan2(q.Y()-p.Y(),q.X()-p.X());
      std::ostringstream sign;sign<<"<sdf version='1.6'><model name='speed_limit_100_"<<i<<"'><static>true</static><pose>"<<p.X()<<" "<<p.Y()<<" 0 0 0 "<<yaw<<"</pose><link name='sign'>"
        <<"<visual name='pole'><pose>0 -10.5 1.25 0 0 0</pose><geometry><cylinder><radius>.055</radius><length>2.5</length></cylinder></geometry><material><diffuse>.45 .47 .50 1</diffuse></material></visual>"
        <<"<visual name='red_disc'><pose>0 -10.5 2.75 0 1.570796 0</pose><geometry><cylinder><radius>.62</radius><length>.08</length></cylinder></geometry><material><diffuse>.85 .02 .02 1</diffuse></material></visual>"
        <<"<visual name='white_disc'><pose>-.05 -10.5 2.75 0 1.570796 0</pose><geometry><cylinder><radius>.50</radius><length>.035</length></cylinder></geometry><material><diffuse>.96 .96 .94 1</diffuse></material></visual>"
        <<"<visual name='one'><pose>-.08 -10.82 2.75 0 0 0</pose><geometry><box><size>.04 .075 .43</size></box></geometry><material><ambient>1 .82 0 1</ambient><diffuse>1 .82 0 1</diffuse><emissive>.35 .25 0 1</emissive></material></visual>";
      for(double center : {-10.48,-10.14}){for(double side : {-.11,.11})sign<<"<visual name='v"<<center<<side<<"'><pose>-.08 "<<center+side<<" 2.75 0 0 0</pose><geometry><box><size>.04 .055 .40</size></box></geometry><material><ambient>1 .82 0 1</ambient><diffuse>1 .82 0 1</diffuse><emissive>.35 .25 0 1</emissive></material></visual>";
        for(double height : {2.56,2.94})sign<<"<visual name='h"<<center<<height<<"'><pose>-.08 "<<center<<" "<<height<<" 0 0 0</pose><geometry><box><size>.04 .25 .055</size></box></geometry><material><ambient>1 .82 0 1</ambient><diffuse>1 .82 0 1</diffuse><emissive>.35 .25 0 1</emissive></material></visual>";}
      sign<<"</link></model></sdf>";world_->InsertModelString(sign.str());
    }
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
    for(std::size_t i=0;i<distance_.size();++i){
      lane_change_cooldown_[i]=std::max(0.,lane_change_cooldown_[i]-dt);
      if(crashed_[i]||lane_change_cooldown_[i]>0)continue;
      auto gaps=[&](std::size_t candidate){double front=arc_.back(),rear=arc_.back();
        for(std::size_t j=0;j<distance_.size();++j){if(i==j||crashed_[j]||lane_[j]!=candidate||direction_[i]!=direction_[j])continue;
          front=std::min(front,std::fmod(direction_[i]*(distance_[j]-distance_[i])+arc_.back(),arc_.back()));
          rear=std::min(rear,std::fmod(direction_[i]*(distance_[i]-distance_[j])+arc_.back(),arc_.back()));}
        return std::array<double,2>{{front,rear}};};
      const auto current=gaps(lane_[i]);const double trigger=i==0?75.:55.;
      if(current[0]<trigger){std::size_t best=lane_[i];double best_front=current[0];
        const std::size_t lane_begin=direction_[i]>0?0:3,lane_end=lane_begin+3;
        for(std::size_t candidate=lane_begin;candidate<lane_end;++candidate){if(candidate==lane_[i]||(candidate+1!=lane_[i]&&lane_[i]+1!=candidate))continue;const auto gap=gaps(candidate);
          const double safe_front=i==0?45.:35.,safe_rear=i==0?28.:22.;
          if(gap[0]>safe_front&&gap[1]>safe_rear&&gap[0]>best_front+10.){best=candidate;best_front=gap[0];}}
        if(best!=lane_[i]){lane_[i]=best;target_lateral_[i]=direction_[i]>0?3.2*best:-3.2*(best-2);lane_change_cooldown_[i]=4.+.15*i;}}
    }
    for(std::size_t i=0;i<lateral_.size();++i)lateral_[i]+=std::clamp(target_lateral_[i]-lateral_[i],-1.35*dt,1.35*dt);
    constexpr double speed_limit=100./3.6;
    for(std::size_t i=0;i<distance_.size();++i){if(crashed_[i])continue;double target=std::min(speed_limit,desired_speed_[i]+(i==0?0.:1.8*std::sin(now.Double()*.12+i)));
      for(std::size_t j=0;j<distance_.size();++j){if(i==j||crashed_[j]||lane_[i]!=lane_[j]||direction_[i]!=direction_[j])continue;
        double gap=std::fmod(direction_[i]*(distance_[j]-distance_[i])+arc_.back(),arc_.back());
        if(gap<95.)target=std::min(target,std::max(0.,(gap-10.)*.48));
        if(gap<4.7){crashed_[i]=true;crashed_[j]=true;}}
      for(std::size_t signal=0;signal<signal_distance_.size();++signal){double gap=std::fmod(direction_[i]*(signal_distance_[signal]-distance_[i])+arc_.back(),arc_.back());
        double phase=std::fmod(now.Double()+signal*7.,27.);const bool red=phase>=15.,yellow=phase>=12.&&phase<15.;
        if(gap<100.&&(red||(yellow&&gap>15.)))target=std::min(target,std::max(0.,(gap-7.)*.48));}
      const double change=target-speed_[i];speed_[i]+=std::clamp(change,-9.*dt,2.2*dt);
    }
    for(std::size_t i=0;i<distance_.size();++i){auto model=world_->ModelByName("traffic_vehicle_"+std::to_string(i));if(!model)continue;
      if(crashed_[i]){auto p=sample(distance_[i]),q=sample(distance_[i]+direction_[i]*8);double yaw=std::atan2(q.Y()-p.Y(),q.X()-p.X());
        if(!physics_released_[i]){model->SetLinearVel({std::cos(yaw)*speed_[i],std::sin(yaw)*speed_[i],0});model->SetAngularVel({0,0,(i%2?.35:-.35)});physics_released_[i]=true;}continue;}
      if(i>0){double gap=std::fmod(distance_[i]-distance_[0]+arc_.back(),arc_.back());
        const double behind=arc_.back()-gap;
        if(gap>arc_.back()*.5&&behind>100.)distance_[i]=std::fmod(distance_[0]+35.+(i-1)*18.,arc_.back());}
      auto p=sample(distance_[i]),route_q=sample(distance_[i]+8),travel_q=sample(distance_[i]+direction_[i]*8);
      double route_yaw=std::atan2(route_q.Y()-p.Y(),route_q.X()-p.X());
      double yaw=std::atan2(travel_q.Y()-p.Y(),travel_q.X()-p.X());
      model->SetWorldPose({p.X()+std::sin(route_yaw)*lateral_[i],p.Y()-std::cos(route_yaw)*lateral_[i],.75,0,0,yaw});
      distance_[i]=std::fmod(distance_[i]+direction_[i]*speed_[i]*dt,arc_.back());if(distance_[i]<0)distance_[i]+=arc_.back();}
    if(auto ego=world_->ModelByName("traffic_vehicle_0")){if(auto sun=world_->ModelByName("visible_sun")){
      const auto pose=ego->WorldPose();const double yaw=pose.Rot().Yaw();
      sun->SetWorldPose({pose.Pos().X()+std::cos(yaw)*210.,pose.Pos().Y()+std::sin(yaw)*210.,90.,0,0,0});}}
    for(std::size_t i=0;i<signal_distance_.size();++i){double phase=std::fmod(now.Double()+i*7.,27.);std::size_t active=phase<12.?2:(phase<15.?1:0);
      auto p=sample(signal_distance_[i]);double yaw=signal_yaw_[i];double x=p.X()+std::sin(yaw)*3.2,y=p.Y()-std::cos(yaw)*3.2;
      for(std::size_t light=0;light<3;++light){auto lamp=world_->ModelByName("traffic_signal_"+std::to_string(i)+"_lamp_"+std::to_string(light));if(!lamp)continue;
        double z=active==light?4.82-.37*light:-20.;lamp->SetWorldPose({x,y,z,0,0,0});}}
    for(std::size_t i=0;i<pedestrian_distance_.size();++i){auto model=world_->ModelByName("pedestrian_"+std::to_string(i));if(!model)continue;
      auto p=sample(pedestrian_distance_[i]),q=sample(pedestrian_distance_[i]+pedestrian_direction_[i]*2);double yaw=std::atan2(q.Y()-p.Y(),q.X()-p.X());
      model->SetWorldPose({p.X()+std::sin(yaw)*pedestrian_lateral_[i],p.Y()-std::cos(yaw)*pedestrian_lateral_[i],.08,0,0,yaw});
      pedestrian_distance_[i]=std::fmod(pedestrian_distance_[i]+pedestrian_direction_[i]*pedestrian_speed_[i]*dt,arc_.back());if(pedestrian_distance_[i]<0)pedestrian_distance_[i]+=arc_.back();}}
};GZ_REGISTER_WORLD_PLUGIN(TrafficWorldPlugin)
}
