#ifndef PF_FILTER_SCAN_
#define PF_FILTER_SCAN_
#include <iostream>
#include <vector>
#include <utility>
#include <list>
#include <math.h>
#include "Geometry.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include<stdio.h>
using namespace std;
//scan data struct
struct scanCluster{
public:
  scanCluster(){}
  scanCluster(double ang,double dist,int echo=0):_angle(ang),_dist(dist),_echo(echo){}
  double angle(){return _angle;}
  double dist(){return _dist;}
  int echo(){return _echo;}
  private:
  double _angle;
  double _dist;
  int _echo;
};

class VecPositionDir:public VecPosition{
public:
  VecPositionDir(){}
  VecPositionDir(double x,double y,double ang =.0):_angle(ang),VecPosition(x,y){}
  VecPositionDir( const VecPosition & pos,double ang=0.0 ):_angle(ang),VecPosition(pos.getX(),pos.getY() ){}
  VecPositionDir(const VecPositionDir& posdir):_angle(posdir.angle()),VecPosition(posdir.getX(),posdir.getY()){}
  double angle() const//we need to add const in here other the last line will not be permitted
  {return _angle;}
private:
double _angle;
};

//filter scan class
class filterScan{
public:
  filterScan();
  filterScan(double reflector_radius,int echo_thread,double step,double err_thread);
  void getReflectorsCenter(const std::vector<sensor_msgs::LaserScan> & raw_scan,std::vector<VecPositionDir> & v_reflectors);

    //
    void groupingScan(const std::vector<sensor_msgs::LaserScan> &raw_scan,
                      std::vector<std::list<scanCluster> > & v_reflectors_group);

    void trimmingScanGroup(std::vector<std::list<scanCluster> > & v_reflectors_group);

    void getInitCenter(const std::vector<std::list<scanCluster> > & v_reflectors_group,
                       std::vector<VecPositionDir>& v_reflectors_center);
    void getRelativePointClouds(const std::vector<std::list<scanCluster> > & v_reflectors_group,
                            const std::vector<VecPositionDir>& v_reflectors_center,
                            std::vector<std::list<VecPositionDir> > & relative_pointclounds);

    void getOptimizedCenter(const std::vector<VecPositionDir>& v_pre_center,
                            const std::vector<std::list<VecPositionDir> > &relative_pointclounds,
                             std::vector<VecPositionDir>& v_opt_center
                            );
    void getFourDirValue( const std::list<VecPositionDir>  &relative_pointclounds,
                           VecPositionDir cen,double* array);//used to cal the four dir var err


    inline int getMin(double* array,double &v){
      int k = 0;
      for(int i = 0; i < 5; i++)//limit the size is 5
      {
        if(array[i] < array[k]) k = i;
      }
      v= array[k];
      //array[k] 就是最大值
      //k 就是对应下标
      return k;
      }
    void move_center( std::list<VecPositionDir> & relative_pointclounds,
                     const double x,const double y,const double _step,int &kx,int &ky,double& sum);
private:
  int _echo;
  double _rf_radius;
  double _step;
  double _err;
  std::vector<std::list<scanCluster> >  _reflectors_group;
  std::vector<std::list<VecPositionDir> > _relative_pointclounds;
  static const int sampe_limit_par1 = 15;
  static const int sampe_limit_par2 = 24;
};
#endif
