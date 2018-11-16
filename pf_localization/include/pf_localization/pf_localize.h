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
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include "calthreearc.h"
#include "Geometry.h"
#include "CountTime.hpp"


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

class pfLocalize{

public:
  pfLocalize();
  void localize();
  void loadRfsMap(string name);
  void setInitPos(double x,double y,double a){
    init_pos.x = x;
    init_pos.y = y;
    init_pos.theta = a;
    recking_pos = init_pos;
  }
  comparedata calCoefficient(VecPosition v_a,VecPosition v_b,VecPosition v_c);
  void createTriangleTemplate();
  VecPosition calTrianglePoint(VecPosition a, VecPosition b,VecPosition c );
  void calGlobalPosThread();
  void deadReckingThread();
  int getMinIndex( std::vector< std::vector<std::pair<int,int> > >& group);
  double getScore( comparedata & comp);
  //评价匹配到的反光板，选出最优的三个反光板（三角形）,相对坐标系下的
  int getOptimizeTriangle(std::vector<std::pair<int,int> > matched_rfs, std::vector<std::pair<int,int> >& best);
  //根据计算出来的坐标　和匹配到的三角形，测量三角形　计算车身方向
  double getOptimizeAngle(std::vector<std::pair<int,int> > result,VecPosition cord_result);
  //将实测反光板与地图中的反光板进行一一匹配，匹配上的添加到匹配列表，内容包括匹配的地图反光板索引和对应的测量反光板在绝对坐标系下的位置
  void getMatchedMeaRfs(std::vector<std::pair<int,int> > & matched_mea_rfs);//loc pos(base_link) ,real measured rfs
  void callbackOdom(const nav_msgs::Odometry::ConstPtr& state_odata);
  void callBackRelativeRfs(const geometry_msgs::PoseArray::ConstPtr& rfs_data);
private:
  double m_cur_vec_x,m_cur_vec_y,m_cur_w;
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
  bool _re_deadrecking;
  geometry_msgs::Pose2D recking_pos;
  geometry_msgs::Pose2D init_pos;
  geometry_msgs::Pose2D global_pos;
  geometry_msgs::Pose2D cur_recking_pos;

  ros::NodeHandle nh;
  ros::Publisher global_pos_pub;
  ros::Publisher marker_pos_pub;
  ros::Subscriber rfs_sub ;
  ros::Subscriber odom_sub ;
   ros::Timer timer ;
  boost::mutex odom_mut;
  boost::mutex cal_mut;
  boost::mutex mut_recking;
  boost::mutex rfs_mut;

  std::string map_name ;
  std::string pos_frame ;
  std::string pos_topic ;
  std::string rfs_topic ;
   std::string odom_topic ;
} ;
#endif
