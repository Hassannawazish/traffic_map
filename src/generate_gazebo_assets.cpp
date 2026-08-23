#include "map_processor.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace {
struct ObjWriter {
  std::ofstream out;
  std::size_t vertex{1};
  explicit ObjWriter(const std::string &path) : out(path) { out << "mtllib road.mtl\n"; }
  void quad(double ax,double ay,double bx,double by,double cx,double cy,double dx,double dy,double z) {
    out<<"v "<<ax<<' '<<ay<<' '<<z<<"\nv "<<bx<<' '<<by<<' '<<z<<"\nv "<<cx<<' '<<cy<<' '<<z<<"\nv "<<dx<<' '<<dy<<' '<<z<<'\n';
    out<<"f "<<vertex<<' '<<vertex+1<<' '<<vertex+2<<"\nf "<<vertex+2<<' '<<vertex+1<<' '<<vertex+3<<'\n'; vertex+=4;
  }
  void ribbon(double x0,double y0,double x1,double y1,double width,double z) {
    const double dx=x1-x0,dy=y1-y0,len=std::hypot(dx,dy); if(len<1e-6)return;
    const double nx=-dy*width/(2*len),ny=dx*width/(2*len);
    quad(x0+nx,y0+ny,x0-nx,y0-ny,x1+nx,y1+ny,x1-nx,y1-ny,z);
  }
};
}

int main(int argc,char **argv) {
  if(argc!=2){std::cerr<<"usage: generate_gazebo_assets OUTPUT_DIR\n";return 1;}
  Config::parse(); map_process map;
  const auto &samples=Config::singleton().planeview_data;
  const auto &left=map.get_left_lanes(); const auto &right=map.get_right_lanes();
  if(samples.size()<2||left.empty()||right.empty()){std::cerr<<"XODR has no usable road\n";return 1;}
  const double ox=samples.front().at("x"),oy=samples.front().at("y");
  const std::string dir=argv[1];
  ObjWriter asphalt(dir+"/road_surface.obj");
  ObjWriter white(dir+"/road_white.obj");
  ObjWriter yellow(dir+"/road_yellow.obj");
  std::ofstream mtl(dir+"/road.mtl");
  mtl<<"newmtl asphalt\nKd 0.055 0.06 0.065\nnewmtl white\nKd 0.9 0.9 0.9\nnewmtl yellow\nKd 1.0 0.72 0.0\n";
  const auto &le=left.back(),&re=right.back(); const std::size_t n=samples.size();
  asphalt.out<<"usemtl asphalt\n";
  for(std::size_t i=0;i+1<n;++i)asphalt.quad(le.at("x")[i]-ox,le.at("y")[i]-oy,re.at("x")[i]-ox,re.at("y")[i]-oy,
    le.at("x")[i+1]-ox,le.at("y")[i+1]-oy,re.at("x")[i+1]-ox,re.at("y")[i+1]-oy,0);
  yellow.out<<"usemtl yellow\n";
  for(std::size_t i=0;i+1<n;++i)yellow.ribbon(samples[i].at("x")-ox,samples[i].at("y")-oy,samples[i+1].at("x")-ox,samples[i+1].at("y")-oy,.28,.12);
  white.out<<"usemtl white\n";
  auto markings=[&](const auto &lanes){for(std::size_t l=0;l<lanes.size();++l){const auto &v=lanes[l];
    if(l+1==lanes.size()){for(std::size_t i=0;i+1<n;++i)white.ribbon(v.at("x")[i]-ox,v.at("y")[i]-oy,v.at("x")[i+1]-ox,v.at("y")[i+1]-oy,.22,.12);}
    else {for(std::size_t i=0;i+2<n;i+=6)white.ribbon(v.at("x")[i]-ox,v.at("y")[i]-oy,v.at("x")[i+2]-ox,v.at("y")[i+2]-oy,.18,.12);}}};
  markings(left); markings(right);
  std::ofstream route(dir+"/route.csv"); const auto &edge=right.front();
  for(std::size_t i=0;i<n;++i){const double x=(samples[i].at("x")+edge.at("x")[i])*0.5-ox;
    const double y=(samples[i].at("y")+edge.at("y")[i])*0.5-oy; route<<x<<','<<y<<'\n';}
  std::cout<<"Generated Gazebo road and route in "<<dir<<'\n'; return 0;
}
