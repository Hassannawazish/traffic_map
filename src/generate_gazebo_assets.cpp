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
  void doubleSidedQuad(double ax,double ay,double bx,double by,double cx,double cy,double dx,double dy,double z) {
    const std::size_t first=vertex;quad(ax,ay,bx,by,cx,cy,dx,dy,z);
    out<<"f "<<first+2<<' '<<first+1<<' '<<first<<"\nf "<<first+3<<' '<<first+1<<' '<<first+2<<'\n';
  }
  void ribbon(double x0,double y0,double x1,double y1,double width,double z) {
    const double dx=x1-x0,dy=y1-y0,len=std::hypot(dx,dy); if(len<1e-6)return;
    const double nx=-dy*width/(2*len),ny=dx*width/(2*len);
    quad(x0+nx,y0+ny,x0-nx,y0-ny,x1+nx,y1+ny,x1-nx,y1-ny,z);
  }
  void triangle(double ax,double ay,double az,double bx,double by,double bz,double cx,double cy,double cz) {
    out<<"v "<<ax<<' '<<ay<<' '<<az<<"\nv "<<bx<<' '<<by<<' '<<bz<<"\nv "<<cx<<' '<<cy<<' '<<cz<<'\n';
    out<<"f "<<vertex<<' '<<vertex+1<<' '<<vertex+2<<'\n';vertex+=3;
  }
  void cylinder(double x,double y,double z,double radius,double height,int sides=7) {
    for(int i=0;i<sides;++i){const double a=6.283185307179586*i/sides,b=6.283185307179586*(i+1)/sides;
      const double ax=x+radius*cos(a),ay=y+radius*sin(a),bx=x+radius*cos(b),by=y+radius*sin(b);
      triangle(ax,ay,z,bx,by,z,ax,ay,z+height);triangle(ax,ay,z+height,bx,by,z,bx,by,z+height);}
  }
  void cone(double x,double y,double z,double radius,double height,int sides=8) {
    for(int i=0;i<sides;++i){const double a=6.283185307179586*i/sides,b=6.283185307179586*(i+1)/sides;
      triangle(x+radius*cos(a),y+radius*sin(a),z,x+radius*cos(b),y+radius*sin(b),z,x,y,z+height);}
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
  ObjWriter vegetation(dir+"/vegetation.obj");
  ObjWriter sidewalks(dir+"/sidewalks.obj");
  std::ofstream mtl(dir+"/road.mtl");
  mtl<<"newmtl asphalt\nKd 0.055 0.06 0.065\nnewmtl white\nKd 0.9 0.9 0.9\nnewmtl yellow\nKd 1.0 0.72 0.0\n"
     <<"newmtl grass\nKd 0.12 0.30 0.07\nnewmtl trunk\nKd 0.24 0.11 0.04\nnewmtl foliage\nKd 0.08 0.34 0.06\nnewmtl shrub\nKd 0.16 0.42 0.08\n";
  mtl<<"newmtl sidewalk\nKd 0.38 0.39 0.40\n";
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
  sidewalks.out<<"usemtl sidewalk\n";
  for(std::size_t i=0;i+1<n;++i){
    const double tx=samples[i+1].at("x")-samples[i].at("x"),ty=samples[i+1].at("y")-samples[i].at("y"),len=std::hypot(tx,ty);
    if(len<1e-6)continue;
    const double nx=-ty/len,ny=tx/len;
    const double lx0=le.at("x")[i]-ox,ly0=le.at("y")[i]-oy,lx1=le.at("x")[i+1]-ox,ly1=le.at("y")[i+1]-oy;
    const double rx0=re.at("x")[i]-ox,ry0=re.at("y")[i]-oy,rx1=re.at("x")[i+1]-ox,ry1=re.at("y")[i+1]-oy;
    sidewalks.doubleSidedQuad(lx0+nx*.25,ly0+ny*.25,lx0+nx*4.0,ly0+ny*4.0,lx1+nx*.25,ly1+ny*.25,lx1+nx*4.0,ly1+ny*4.0,.16);
    sidewalks.doubleSidedQuad(rx0-nx*.25,ry0-ny*.25,rx0-nx*4.0,ry0-ny*4.0,rx1-nx*.25,ry1-ny*.25,rx1-nx*4.0,ry1-ny*4.0,.16);
  }
  vegetation.out<<"usemtl grass\n";
  for(std::size_t i=0;i+1<n;++i){
    const double tx=samples[i+1].at("x")-samples[i].at("x"),ty=samples[i+1].at("y")-samples[i].at("y"),len=std::hypot(tx,ty);
    if(len<1e-6)continue;
    const double nx=-ty/len,ny=tx/len;
    const double lx0=le.at("x")[i]-ox,ly0=le.at("y")[i]-oy,lx1=le.at("x")[i+1]-ox,ly1=le.at("y")[i+1]-oy;
    const double rx0=re.at("x")[i]-ox,ry0=re.at("y")[i]-oy,rx1=re.at("x")[i+1]-ox,ry1=re.at("y")[i+1]-oy;
    vegetation.quad(lx0,ly0,lx0+nx*18,ly0+ny*18,lx1,ly1,lx1+nx*18,ly1+ny*18,-.03);
    vegetation.quad(rx0,ry0,rx0-nx*18,ry0-ny*18,rx1,ry1,rx1-nx*18,ry1-ny*18,-.03);
  }
  for(std::size_t i=40;i+1<n;i+=90){
    const double tx=samples[i+1].at("x")-samples[i].at("x"),ty=samples[i+1].at("y")-samples[i].at("y"),len=std::hypot(tx,ty);
    if(len<1e-6)continue;
    const double nx=-ty/len,ny=tx/len;
    for(int side : {-1,1}){
      const auto &edge=side>0?le:re;const double variation=2.0+(i%5)*.7;
      const double x=edge.at("x")[i]-ox+side*nx*(7.+variation),y=edge.at("y")[i]-oy+side*ny*(7.+variation);
      const double height=4.5+(i%7)*.35;
      vegetation.out<<"usemtl trunk\n";vegetation.cylinder(x,y,0,.22,height*.48);
      vegetation.out<<"usemtl foliage\n";vegetation.cone(x,y,height*.30,1.65,height*.70);vegetation.cone(x,y,height*.57,1.2,height*.55);
      const double sx=x+side*nx*3.2,sy=y+side*ny*3.2;
      vegetation.out<<"usemtl shrub\n";vegetation.cone(sx,sy,0,.8,1.25,7);
    }
  }
  std::ofstream route(dir+"/route.csv"); const auto &edge=right.front();
  for(std::size_t i=0;i<n;++i){const double x=(samples[i].at("x")+edge.at("x")[i])*0.5-ox;
    const double y=(samples[i].at("y")+edge.at("y")[i])*0.5-oy; route<<x<<','<<y<<'\n';}
  std::cout<<"Generated Gazebo road and route in "<<dir<<'\n'; return 0;
}
