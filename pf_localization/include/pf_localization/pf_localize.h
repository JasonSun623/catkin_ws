// Copyright (c) 2018, Pepperl+Fuchs Cal scan localization by rfs  @Houdar Company
// Copyright (c) 2018-11, @author CaiHuaQiang
// All rights reserved.
#ifndef PF_LOCALIZE_
#define PF_LOCALIZE_
#include <ros/ros.h>
#include <ros/rate.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <utility> //pair
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Vector3.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"

// given the three abs cord rfs and their dist to robot,cal the robot abs pos
#include "calthreearc.h"
#include "Geometry.h"
#include "CountTime.hpp"

#include <pf_filtercenter/filter_rfs_center.h>
using namespace filter_rfs_center_space;
namespace pf_localization_space {

/**
 * @struct comparedata
 * @brief record the triangle para such as the len of three sides,their variance,and the side ratio bet largest side and the least side
 */
struct comparedata{
  comparedata(){}
  comparedata(double l,double dx,double p){
    _len = l;
    _dx = dx;
    _p = p;
  }
  comparedata operator -(const comparedata& b){
    return comparedata(_len-b._len,_dx-b._dx,_p-b._p);
  }
  double getLen(void){return _len;}
  double getDx(void){return _dx;}
  double getP(void){return _p;}
  double _len;
  double _dx;
  double _p;
};

/**
 * @class pfLocalize
 * @brief reflectors localize proc.use the reflectors scan data to get every center of the rfs and according them to loc robot pos
 */
class pfLocalize{

public:
  /**
   * @brief Default constructor
   */
  pfLocalize();
  /**
   * @brief call this fun to active loc thread given condition that give rfs map
   */
  void localize();
  /**
   * @brief call this fun to active loc thread in no condition
   */
  void run(){
    _run_loc_thread = true;
  }
  /**
   * @brief call this fun to inactive loc thread in no condition
   */
  void stop(){
    _run_loc_thread = false;
    _re_deadrecking = true;
  }
  void compensateTimeStart(){
    compensate_timer.begin();
  }
  double compensateTimeStop(){
    compensate_timer.end();
    double dt = compensate_timer.getTime();
    return dt;
  }
  /**
   * @brief accoring the excited map,we record them in memory for matching laterly,and according this we will also pre construct the triangles bet them under limit dist lately
   */
  void loadRfsMap(std::string name);
  /**
   * @brief construct the triangles template para data under limit dist
   */
  void createTriangleTemplate();
  /**
   * @brief the first step to loc  ,given a better init pos to start mathching the rfs in map to the rfs measured
   * @param x,y,a init pos: x,y,theta
   */
  void setInitPos(double x,double y,double a){
    init_pos.x = x;
    init_pos.y = y;
    init_pos.theta = a;
    recking_pos = init_pos;
  }
  void setGlobalPos(geometry_msgs::Pose2D loc_pos){
    global_pos =loc_pos;
  }
  void setReReckingFlag(bool flag){
    _re_deadrecking = flag;
  }
  void UpdateCurMeaRfs(std::vector<VecPosition> mea_rfs){
    this->mea_rfs = mea_rfs;
  }
  /**
   * @brief upodate map rfs data if they are actively updating outsidely
   * @param rfs new map rfs
   */
  void setMapRfsData(const std::deque<VecPosition>& rfs){
    std::vector<VecPosition> temp_rfs;
    temp_rfs.assign(rfs.begin(),rfs.end() );
    map_rfs = temp_rfs;
  }
  /**
   * @brief upodate map Triangle Template Data if they are actively updating outsidely
   * @param tri_data Triangle Template Data
   */
  void setTriangleTemplateData(const std::map<set<int>,comparedata>& tri_data){
    _triangle_template = tri_data;
  }
  double normalizeRad(double angle_rad){
    return atan2(sin(angle_rad),cos(angle_rad));
  }
  double mapToMinusPIToPI( double angle )
  {
    double angle_overflow = static_cast<double>( static_cast<int>(angle / M_PI ) );

    if( angle_overflow > 0.0 )
    {
      //ceil函数。ceil(x)返回的是大于x的最小整数。
      angle_overflow = ceil( angle_overflow / 2.0 );
    }
    else
    {
      //floor(x)返回的是小于或等于x的最大整数。
      angle_overflow = floor( angle_overflow / 2.0 );
    }

    angle -= 2 * M_PI * angle_overflow;
    return angle;
  }

  /**
   * @brief given three cords ,cal the para about the triangle sides length,side variance,and the ratio bet max side len and min len
   * @param v_a,v_b,v_c,three cord
   */
  comparedata calCoefficient(VecPosition v_a,VecPosition v_b,VecPosition v_c);

 /**
 * @brief given the rfs maps cord info ,pre construct the triangles template bet them under limit dist lately
 */
  VecPosition calTrianglePoint(VecPosition a, VecPosition b,VecPosition c );
  /**
   * @brief the rfs global localization thread
   * use the dead recking pos and the newest measured rfs to cal the loc pos
   * the rfs measured data will be delayed when we used so need to compensate according the delta bet the time rfs measured and the time be used
   */
  virtual void calGlobalPosThread();
  /**
   * @brief odom dead recking thread .we think the  pos will be more accurate in short time (the time bet the global loc thread end and restart)
   * the start pos will be updated when the new cald global pos by calGlobalPosThread is coming
   */
  void deadReckingThread();
  /**
   * @brief return loc pos
   */
  geometry_msgs::Pose2D getLocPos(){
    return global_pos;
  }
  /**
   * @brief return recking pos
   */
  geometry_msgs::Pose2D getReckingPos(){
    return recking_pos;
  }
  /**
   * @brief return measured rfs in scan frmme
   */
  std::vector<VecPosition> getMeasuredRfs(){
    return _rfs;
  }
  std::vector<VecPosition> getCurMapRfs(){
    return map_rfs;
  }
  std::string getMapFrame(){
    return map_frame;
  }
  /**
   * @brief pub tf from map to cur pos frame
   */
  void pubMapToThisTF( geometry_msgs::Pose2D cur_pos );
  /**
   * @brief Compensate the delay time from the rfs info be used(in calGlobalPosThread) to the rfs be caled (in callBackScan)
   */
  void compensateRfsUsedDelay(std::vector<VecPosition>& mea_rfs,double dt);
  /**
   * @brief Compensate the diff time "A" bet every scan frame,and the same time *B* from  the scannging complate to the scan be received in call back func
   * A:0 deg will be compensate with (360-0)*(1/scan_fre),360 deg will be compsate with (360-3600)*(1/scan_fre)
   * B:all deg from 0 to 360 will be compensate by (cur scan received  time - last scan received time)-(1/scan_fre)
   */
  void compensateScanRecDelay(std::vector<VecPositionDir>& mea_rfs,double loop_time);
  /**
   * @brief For the matched triangles, find out include angles bet the lines constructed by robot pos and the triangle point  whose angles are most close to 120 degrees.
   */
  int getMinIndex( std::vector< std::vector<std::pair<int,int> > >& group);
  /**
   * @brief given the score limit,select the triangles which comparedata para is fullfill
   */
  double getScore( comparedata & comp);
  /**
   * @brief 评价匹配到的反光板，选出最优的三个反光板（三角形）,相对坐标系下的
   */
  int getOptimizeTriangle(geometry_msgs::Pose2D pos,std::vector<std::pair<int,int> > matched_rfs, std::vector<std::pair<int,int> >& best);
  /**
   * @brief 根据计算出来的坐标　和匹配到的三角形，测量三角形　计算车身方向
   */

  double getOptimizeAngle(std::vector<std::pair<int,int> > result,VecPosition cord_result);
  /**
   * @brief 将实测反光板与地图中的反光板进行一一匹配，匹配上的添加到匹配列表，内容包括匹配的地图反光板索引和对应的测量反光板在绝对坐标系下的位置
   * @param mea_rfs cur measured rfs
   * @param matched_mea_rfs pair type: var first map rfs index,second measured rfx index
  */
  void getMatchedMeaRfs(geometry_msgs::Pose2D pos,
                        std::vector<VecPosition> mea_rfs,
                        std::vector<std::pair<int,int> > & matched_mea_rfs);//loc pos(base_link) ,real measured rfs

  void callbackOdom(const nav_msgs::Odometry::ConstPtr& state_odata);
  //void callBackRelativeRfs(const geometry_msgs::PoseArray::ConstPtr& rfs_data);
  void callBackScan(const sensor_msgs::LaserScan & scan);
  std::string map_frame,base_frame,scan_frame,odom_frame;
  geometry_msgs::Pose2D cur_recking_pos,cur_scan_pos;
private:
  double m_cur_vec_x,m_cur_vec_y,m_cur_w;//the trans velocity and the rot velocity cal by odom data
  double lrange1,lrange2,lrange3,lrange4;
  double dxrange1,dxrange2,dxrange3,dxrange4;
  double prange1,prange2,prange3,prange4;
  double kl1,kl2,kl3,kl4;
  double kdx1,kdx2,kdx3,kdx4;
  double kp1,kp2,kp3,kp4;

  double triangle_side_min_len;
  double triangle_grouping_thread;
  double search_triangle_thread;
  double match_angle_thread,match_dist_thread;
  double score_thread;
  std::vector<VecPosition> map_rfs;
  std::vector<VecPosition> comp_rfs;

  std::vector<VecPosition> _rfs;
  std::vector<VecPosition> mea_rfs;//cur _rfs
  std::map<set<int>,comparedata> _triangle_template;
  bool _run_loc_thread;//start loc flag
  bool _re_deadrecking;//when the global pos is caled successfully,set _re_deadrecking be true to force the dead recking restart
  int scan_update_fre;//laser scan frequency(property data not set data)
  double compensate_time;//when cal global pos the delay time bet when the rfs be used and the rfs msg arrived
  CountTime compensate_timer;
  geometry_msgs::Pose2D recking_pos;//odom recking pos
  geometry_msgs::Pose2D init_pos;//
  geometry_msgs::Pose2D global_pos;//caled by calGlobalPosThread


  ros::NodeHandle nh;
  ros::Publisher global_pos_pub,global_recking_pos_pub;
  ros::Publisher marker_pos_pub;
  ros::Subscriber rfs_sub ;
  ros::Subscriber odom_sub ;
  ros::Subscriber scan_sub;
  ros::Timer timer,timer_cal ;
  boost::mutex odom_mut;
  boost::mutex cal_mut;
  boost::mutex mut_recking;
  boost::mutex rfs_mut;
  boost::mutex temp_mut;


  std::string map_name ;
  std::string pos_frame ;
  std::string pos_topic,recking_pos_topic,pub_rfs_topic,rfs_topic ;
  std::string scan_topic,odom_topic ;

  ///transform map to scan
  ///     // Use a child class to get access to tf2::Buffer class inside of tf_
  struct TransformListenerWrapper : public tf::TransformListener
  {
    inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
  };
  TransformListenerWrapper* tf_;

  bool sent_first_transform_;

  tf::Transform latest_tf_;
  tf::StampedTransform tf_base2scan;
  bool latest_tf_valid_;
   bool tf_broadcast_;
   tf::TransformBroadcaster* tfb_;
   //time for tolerance on the published transform,
   //basically defines how long a map->odom transform is good for
   double tmp_tol;
   ros::Duration transform_tolerance_;

  ///
  ///for filter rfs center---start
  ros::Publisher pub_rfs_center;
  boost::mutex scan_lock_;

  geometry_msgs::PoseArray rfs_pos_pub;
  std::vector<sensor_msgs::LaserScan> raw_scan;
  std::vector<VecPositionDir> v_optrfs_center;//the optimized rfs center data
  int _echo;
  int pub_rate;
  int _fiter_scan_num;
  bool if_pub_marker;
  bool _judge_by_dist;//直接通过距离判断反光板
  double _rf_radius;
  double _step;
  double _err;
  ///for filter rfs center---end
  FilterRfsCenter filter;
} ;
}
#endif
