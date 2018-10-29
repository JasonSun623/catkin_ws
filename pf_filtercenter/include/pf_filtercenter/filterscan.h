
#ifndef PF_FILTER_SCAN_
#define PF_FILTER_SCAN_

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <list>
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>

#include "Geometry.h"

using namespace std;

/**
 * @class scanCluster
 *
 * @brief record the raw scan data. including angle ,dist and echo value
 * the data will be process by the filterScan class
 */
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

/**
 * @class VecPositionDir
 *
 * @brief drivated from VecPosition class ,and it also have the angle info of the cord pos
 * it used for math calculating
 */
class VecPositionDir:public VecPosition{
public:
  VecPositionDir(){}
  VecPositionDir(double x,double y,double ang =.0):_angle(ang),VecPosition(x,y){}
  VecPositionDir( const VecPosition & pos,double ang=0.0 ):_angle(ang),VecPosition(pos.getX(),pos.getY() ){}
  VecPositionDir(const VecPositionDir& posdir):_angle(posdir.angle()),VecPosition(posdir.getX(),posdir.getY()){}
  double angle() const//we need to add const in here, otherwise the last line will not be permitted
  {return _angle;}
private:
 double _angle;
};

/**
 * @class filterScan
 *
 * @brief the filterScan used for grouping the scan data into some reflectors
 * it also cal the center the center of rfs and also optimizes the center result
 * 依据激光强度数据实现对反光板的提取与反光板中心精确计算
 */
class filterScan{
public:
  filterScan();
  filterScan(double reflector_radius,int echo_thread,double step,double err_thread);
  /**
   * @brief the public func for calculating the rfs center called by user
   * @param raw_scan raw scan data published by laser scan
   * @return v_reflectors the rfs center pos in laser cord
   */
  void getReflectorsCenter(const std::vector<sensor_msgs::LaserScan> & raw_scan,std::vector<VecPositionDir> & v_reflectors);

private:
  //按照激光强度分布，将激光数据分成若干组，每一组代表一个激光反光板
  void groupingScan(const std::vector<sensor_msgs::LaserScan> &raw_scan,
                    std::vector<std::list<scanCluster> > & v_reflectors_group);
  //若激光数据帧过多，裁剪激光数据
  void trimmingScanGroup(std::vector<std::list<scanCluster> > & v_reflectors_group);

  void getInitCenter(const std::vector<std::list<scanCluster> > & v_reflectors_group,
                     std::vector<VecPositionDir>& v_reflectors_center);
  //获得激光数据点相对于初始计算的定位中心的位置
  void getRelativePointClouds(const std::vector<std::list<scanCluster> > & v_reflectors_group,
                          const std::vector<VecPositionDir>& v_reflectors_center,
                          std::vector<std::list<VecPositionDir> > & relative_pointclounds);
  //优化计算，获得最优的定位中心
  void getOptimizedCenter(const std::vector<VecPositionDir>& v_pre_center,
                          const std::vector<std::list<VecPositionDir> > &relative_pointclounds,
                           std::vector<VecPositionDir>& v_opt_center
                          );
  //获得某个位置左右上下，步进为_step的相对平方误差
  void getFourDirValue( const std::list<VecPositionDir>  &relative_pointclounds,VecPositionDir cen,double* array);//used to cal the four dir var err
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
  //依据输入x,y 和激光相对位置数据，计算要移动的左右上下方向
  void move_center( std::list<VecPositionDir> & relative_pointclounds,
                   const double x,const double y,const double _step,int &kx,int &ky,double& sum);
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
