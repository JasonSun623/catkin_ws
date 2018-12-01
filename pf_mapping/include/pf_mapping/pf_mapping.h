// Copyright (c) 2018, Pepperl+Fuchs Mapping @Houdar Company
// Copyright (c) 2018-11, @author CaiHuaQiang
// All rights reserved.
#ifndef PF_MAPPING_H_
#define PF_MAPPING_H_

#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int16.h> //callback
#include <boost/thread.hpp>
#include <deque>
#include <list>
#include <map>
#include <utility> //for pair
#include <iostream>
#include <algorithm> //for sort
#include <vector>

#include <pf_localization/pf_localize.h>

#include "CountTime.hpp"
#include "Geometry.h"

namespace pf_mapping_space {
using namespace pf_localization_space;
struct order_pair_rfs_para{
  order_pair_rfs_para(){}
  order_pair_rfs_para(std::set<int> pa,double dist):_pair(pa),_dist(dist){}
  double dist(){return _dist;}
  std::set<int> pair(){return _pair;}
  double _dist;
  std::set<int> _pair;
};
struct rfs_history_cord
{
  rfs_history_cord() {
    _dx = 1/(EPSILON);//large
    _use_num= 0;
    pos_array = new  std::list<VecPosition>;
  }
  void addPos(VecPosition pos){
     pos_array->push_back(pos);
     //_use_num+=1;
  }
  void getPara(std::list<VecPosition> & pos_arr,double&dx,double & use_num){
    pos_arr = *pos_array;
    dx= _dx;
    use_num = _use_num;
  }
  std::list<VecPosition> & getPosArray(){
     return *pos_array;
  }
  double size(){
    return _use_num;
  }
  double dx(){
    return _dx;
  }
  void setDx(double dx){
    _dx=dx;
  }
  void setAvgPos(VecPosition pos){
    avg_pos = pos;
  }
  void getAgvPos(VecPosition &pos){
    pos  = avg_pos;
  }
  void operator= ( rfs_history_cord & b){
    pos_array = b.pos_array;
    _dx = b.dx();
    _use_num = b.size();

  }
  void setUseNum(double size){
    _use_num = size;
  }
  std::list<VecPosition> * pos_array;

  VecPosition avg_pos;
  double _dx;
  double _use_num;
};
class pf_mapping{

public:
  pf_mapping();
  ~pf_mapping();
  void callBackMappingStart(const std_msgs::Int16  if_start);
  void callBackMappingCancel(const std_msgs::Int16  if_cancel);
  void callBackMappingStop(const std_msgs::String fname);
  void mappingThread();
  //每间隔一段时间，更新view范围内　已在地图中的反光板的坐标方差数据
  void updateHistoryRfsThread();
  /**
   * @brief the first step to loc  ,given a better init pos to start mathching the rfs in map to the rfs measured
   * @param x,y,a init pos: x,y,theta
   */
  void setInitPos(double x,double y,double a){
    init_x = x;
    init_y = y;
    init_theta = a;
  }
  /**
   * @brief get new  rfs
   * @param loc_pos input loc pos
   * @param mea_rfs input measured rfs
   * @param mea_rfs return every rfs index which not in abs rfs map data!!!!
   */
  void getNewRfs(geometry_msgs::Pose2D loc_pos, std::vector<VecPosition>& mea_rfs);
  void addingNewRfs(geometry_msgs::Pose2D loc_pos, std::vector<VecPosition> new_rfs);
  /**
   * @brief update triangle template data by new rfs data
   * @param new_rfs new rfs
   */
  void updateTriangleTemplate(std::vector<VecPosition> new_rfs);
  /**
   * @brief update triangle template data and pair dist data by history rfs average data
   * @param avg_rfs_index   map rfs index
   * @param avg_rfs average history rfs cord
   */
  void updateRfsPara(int avg_rfs_index,VecPosition avg_rfs);
  void insertSortRfs(std::vector<VecPosition> new_rfs);
  void sortPairRfsDist();
  void pubMarkerRfs();
  //void sortMapRfsPairDist(std::vector<VecPosition> matched_rfs); //TODO insert or heap sort


private:
  geometry_msgs::Pose2D _global_pos;//scan pos cal by pf_localization

  std::deque<VecPosition> _abs_map_rfs;//measured rfs in map cord
  std::map<std::set<int>,comparedata> _triangle_template;//measured rfs para( sum(side len) side dx (max side len)/(min side len) ) in map cord
  std::map<std::set<int>,double> _pair_rfs_dist;//the pair rfs in scan view  range
  std::deque<order_pair_rfs_para> ordered_pair_rfs_dist;//all the pair rfs ordered by largest to smallest
  std::deque<rfs_history_cord*> history_rfs;//every map rfs history cord para data

  double triangle_side_min_len;
  double triangle_grouping_thread;
  double scan_range_max;
  double new_rfs_dist,new_rfs_angle_deg;//是新的反光板的最小间隔距离和角度
  double history_rfs_reject_dist;//与平均值差值　大于此值，不纳入求历史反光板坐标
  int history_update_size_thread;              //the cord update min history size
  int min_his_rfs_avg_size;//使反光板更新为平均值的　历史反光板数据最少数量
  double his_rfs_update_thread;

  ros::Timer timer_mapping,timer_updatehistory ;
 ros::NodeHandle nh;
 ros::Publisher map_marker_pub;
 ros::Subscriber sub_mapping_task_start,sub_mapping_task_cancel,sub_mapping_task_stop;
 bool mapping_start;
 bool mapping_cancel;
 bool init_mapping;//firt to mapping
 double init_x,init_y,init_theta;//init cnofigured cord when start mapping
 pfLocalize localization;

 boost::mutex addrfs_mut;
};

}
#endif
