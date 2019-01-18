#ifndef PF_SLAM_H_
#define PF_SLAM_H_
#include <deque>
#include <fstream>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <pf_localization/pf_localize.h>

#include <std_msgs/Int16.h> //callback
#include <pf_filtercenter/CountTime.hpp>
#include <pf_localization/Geometry.h>

#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Vector3.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"

namespace pf_slam_space {
using namespace pf_localization_space;
struct order_pair_rfs_para{
  order_pair_rfs_para(){}
  order_pair_rfs_para(std::set<int> pa,double dist):_dist(dist){
    _pair=pa;//使用赋值运算符正确，如何使用拷贝构造会导致初始化错误，参见set拷贝构建函数
  }
  double dist(){return _dist;}
  std::set<int> pair(){return _pair;}
  friend std::ostream& operator <<(std::ostream &os,  order_pair_rfs_para p);
  double _dist;
  std::set<int> _pair;
};
 std::ostream& operator <<(std::ostream &os, order_pair_rfs_para p)
{
   std::set<int>::iterator itr = p.pair().begin();
   int d1 = *(itr++);
   int d2 = *(itr++);
   os << "set:("<< d1 <<","<< d2 <<")dist: " << p.dist() <<"\n";
   return os;
}
struct rfs_history_cord
{
  rfs_history_cord() {
    _dx = 1/(EPSILON);//large
    _cur_dx = 1/(EPSILON);//large
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
  double getCurDx(){
    return _cur_dx;
  }
  void setDx(double dx){
    _dx=dx;
  }
  void setCurDx(double dx){
    _cur_dx=dx;
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
    _cur_dx = b.getCurDx();
    _use_num = b.size();

  }
  void setUseNum(double size){
    _use_num = size;
  }
  std::list<VecPosition> * pos_array;

  VecPosition avg_pos;
  double _dx;//history best dx
  double _cur_dx;//newest dx
  double _use_num;
};
class pf_slam:public pfLocalize{
public:
 pf_slam();
 ~pf_slam();
 void callBackMappingStart(const std_msgs::Int16  if_start);
 void callBackMappingCancel(const std_msgs::Int16  if_cancel);
 void callBackMappingStop(const std_msgs::String fname);
 /**
  * @brief the rfs global localization thread
  * use the dead recking pos and the newest measured rfs to cal the loc pos
  * the rfs measured data will be delayed when we used so need to compensate according the delta bet the time rfs measured and the time be used
  */
 virtual void calGlobalPosThread();
 //每间隔一段时间，更新view范围内　已在地图中的反光板的坐标方差数据
 void updateHistoryRfsThread( geometry_msgs::Pose2D scan_pos,std::vector<VecPosition> mea_rfs,std::vector<std::pair<int,int> > matched_mea_rfs);
 /**
  * @brief get new  rfs
  * @param loc_pos input loc pos
  * @param mea_rfs input measured rfs
  * @param mea_rfs return every rfs index which not in abs rfs map data!!!!
  */
 void getNewRfs(geometry_msgs::Pose2D scan_pos, std::vector<VecPosition>& mea_rfs);
 void addingNewRfs(geometry_msgs::Pose2D scan_pos, std::vector<VecPosition> new_rfs);
 /**
  * @brief update triangle template data and pair dist data by history rfs average data
  * @param avg_rfs_index   map rfs index
  * @param avg_rfs average history rfs cord
  */
 void updateRfsPara(int avg_rfs_index,VecPosition avg_rfs);

 void insertSortRfs(std::vector<VecPosition> new_rfs);
 void sortPairRfsDist();

private:
 double triangle_side_min_len,triangle_grouping_thread;
 geometry_msgs::Pose2D _global_pos;//scan pos cal by pf_localization
 std::map<std::set<int>,comparedata> _triangle_template;//measured rfs para( sum(side len) side dx (max side len)/(min side len) ) in map cord
 std::deque<VecPosition> _abs_map_rfs;//measured rfs in map cord
 std::map<std::set<int>,double> _pair_rfs_dist;//the pair rfs in scan view  range
 std::deque<order_pair_rfs_para> ordered_pair_rfs_dist;//all the pair rfs ordered by largest to smallest
 std::deque<rfs_history_cord*> history_rfs;//every map rfs history cord para data

 tf::StampedTransform tf_base2scan;

 double scan_range_max;
 double keep_to_otherrfs_dist,keep_to_otherrfs_angle_deg;//与其他反光板保持的最小距离，角度
 double new_rfs_dist,new_rfs_angle_deg;//是新的反光板的最小间隔距离和角度
 double history_rfs_reject_dist;//与平均值差值　大于此值，不纳入求历史反光板坐标
 int history_update_size_thread;//the cord update min history size
 int min_his_rfs_avg_size;//使反光板更新为平均值的　历史反光板数据最少数量
 double his_rfs_update_thread;
 ros::NodeHandle nh;
 ros::Timer timer_updatehistory ;

 ros::Publisher global_pos_pub;
 ros::Subscriber sub_mapping_task_start,sub_mapping_task_cancel,sub_mapping_task_stop;
 bool mapping_start,mapping_cancel,init_mapping;//firt to mapping
 boost::mutex addrfs_mut,temp_mut,cal_mut;
};
}

#endif
