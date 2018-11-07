#ifndef TYPEVDOCK_H
#define TYPEVDOCK_H

#pragma once
#define MAX_ANGLE_ERROR    20.0        // m_angle - MAX_ANGLE_ERROR MUST >  90 deg
                                       // m_angle + MAX_ANGLE_ERROR MUST < 180 deg
///#define MIN_LENGTH_ERROR  0.07         // length error chq disable
#define MIN_LENGTH_ERROR  0.3         // length error
#include "point.h"
#include "laser2line.h"
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>
#include <string>
#include "Geometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include "MotionControlHeader.hpp"
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
class Docking
{
public:
  Docking(void);
  ~Docking(void);

  /// some flag
  int m_task_flag;
  /// task status: 0 --> off  1 --> on    -1 --> stop ...
  int m_task_status;
  int m_reme_status;

  bool stop_flag;

  double m_speedv, m_speedw;

  /// laser 2 line
  Laser2Line m_vdetec;
  std::vector<segments> m_line;

  /// some function


  enum ChargeTaskType{
    NoTask = 0,
    ChargeInTask = 1,
    ChargeOutTask = 2
  };
  enum ChargeStatus{
    ChargeStop = -1,
    ChargeFail = 0,
    ChargeBegin = 2,
    GoStraightLineMode = 3,
    ChargeInSuc = 4,
    ChargeOutSuc = 5,
    ModifyAngleOnly = 35,
    TypeVNotFound= 110,
    AtField1 = 999,
    AtField2 = 9999
  };
  void setInitTargetDist(double dist){
    init_target_dist = dist;
  }
  // Set module enable
  /// 随任务状态变化更新g_status( not 0 means autocharge go)
  void Set_Switch(bool sflag)
  {
    charge_status = sflag;
  }
  void calPos(void);
  void trackTarget(PointList &scan, Point target_point);
  void updateLaser(const sensor_msgs::LaserScan::ConstPtr& ldata);
  ////接受任务回调，１，充电，２，退出充电,update date to type typeV
  void callBackTask(const std_msgs::Int16  Docking_motion);
  void updateOdom(const nav_msgs::Odometry::ConstPtr& state_odata);
  void autoChargeThread(void);
  void motionPlanning();
  void setTask(int motion_task);
  void searchVType(const PointList& laser_data);//chq
  void setVTypeProperty(double l1, double ang1,double min_range_dist, double dis, double outdis);
  void Set_Frame(std::string od_f,std::string base_link_f,std::string laser_f,std::string map_f ="/map"){
    odom_frame = od_f;
    base_link_frame = base_link_f;
    laser_frame = laser_f;
    map_frame = map_f;
  }
  void PubMark(std::string frame,std::string ns, uint32_t shape,float scale,double *rgba,vector<VecPosition> points);
  //include angle range -pi/2~pi/2
  double getIncludeAngleRad(double ka,double kb){
    if(fabs(ka*kb+1)<0.000001)
      return  (ka-kb)*(ka*kb+1)>0?M_PI/2:-M_PI/2;
    else
      return atan((ka-kb)/(1+ka*kb));
  }
  double getIncludeAngleK(double ka,double kb){
    if(fabs(ka*kb+1)<0.000001)
      return  (ka-kb)*(ka*kb+1)<0?-MAXK:MAXK;
    else
      return ((ka-kb)/(1+ka*kb));
  }
  //角平分线所在的斜率
 double getAngularBisector_k(double ka,double kb){
#if 0
   double ka_deg =Rad2Deg(atan(ka));
   double kb_deg = Rad2Deg(atan(kb));
   double angle = 0.0;
   //angle夹角,强制为正值[0-180)
   if(fabs(ka*kb+1)<0.000001)
     angle = M_PI/2;
   else{
     angle = getIncludeAngleRad(ka, kb);
     if(angle < 0)
       angle+=M_PI;//计算角度是锐角时，计算其补角（xielv互为相反数）
   }
    double angle_deg =Rad2Deg(angle);
   if(ka*kb>=0&&ka>0 )//都在第1相限
     return tan(atan(min(ka,kb))+angle/2);
   else
   if(ka*kb>=0&&ka<0)//都在第2相限
     return tan(M_PI+atan(min(ka,kb))+angle/2);
   else {//在第1相限 + 在第2相限
     double ang = atan(max(ka,kb))+angle/2;
     double ang_deg = Rad2Deg(ang);
     int flag = (ang-M_PI/2)>0?-1:1;
     if(fabs(ang-M_PI/2)<0.00001)//如果平分线垂直
       return flag*MAXK;
     else
       return tan(ang);
   }
#endif
   //由于两条直线的角平分线有两条　故关键是确定是哪一条平分线
  if(fabs(ka+kb)<0.00001)
    return (ka+kb)<0?M_PI/2:-M_PI/2;
  else{
    double k1=( (ka*kb-1)+sqrt((1-ka*kb)*(1-ka*kb)+(ka+kb)*(ka+kb)) )/(ka+kb);
    double k2=( (ka*kb-1)-sqrt((1-ka*kb)*(1-ka*kb)+(ka+kb)*(ka+kb)) )/(ka+kb);
    double include_angle1_k = fabs(getIncludeAngleK(ka,k1));//force rui jiao
    double include_angle2_K = fabs(getIncludeAngleK(ka,k2));
    if(include_angle1_k > tan_halfmin_recharge && include_angle1_k < tan_halfmax_recharge)
    return k1;
    else
    return k2;
  }
 }
  double mapToMinusPIToPI( double angle )
  {
    double angle_overflow = static_cast<double>( static_cast<int>(angle / M_PI ) );

    if( angle_overflow > 0.0 )
    {
      angle_overflow = ceil( angle_overflow / 2.0 );
    }
    else
    {
      angle_overflow = floor( angle_overflow / 2.0 );
    }

    angle -= 2 * M_PI * angle_overflow;
    return angle;
  }

  Point getTarget();//chq
  geometry_msgs::Twist getSpeed();
  int getStatus();

protected:

private:
  ros::NodeHandle n;
  ros::Publisher items_marker_pub;
  ros::Publisher target_scan_pub;
  ros::Publisher track_targets_pub;
  ros::Subscriber scan_sub ;
  ros::Subscriber odo_sub ;
  ros::Subscriber autocharge_task_sub;
  ros::Publisher vel_pub_;

  std::string cmd_vel_topic ;
  enum m_status
  {
    AT_BEGIN,
    ALMOST_LAST,
    FINISH,
  };
  double turn_ratio;//转向率　０～１，1 沿着垂线直接转　最剧烈
  double init_target_dist ;
  double Vshape;   //the edge length  chq
  double MIN_R;  //连续激光点间距，作为线条的阈值 chq
  boost::mutex mut;
  boost::mutex mut_loop;
  boost::mutex odom_lock_;
  boost::mutex scan_lock_;
  std::string scan_topic ;
  std::string odom_topic ;
  bool charge_status;
  PointList obs_p;//for update in call back
  PointList obstacle_points_;//激光障碍物点数据
  double m_line_rechage;
  double m_line_roller;
  double m_angle_recharge;
  double m_angle_roller;
  double m_lastdis;
  double m_outdis;
  double Sum_Dist;
  double tan_min_roller;
  double tan_max_roller;
  double tan_min_recharge;
  double tan_max_recharge;
  double tan_halfmin_recharge;
  double tan_halfmax_recharge;
  double HX;
  double pre_targetx, pre_targety, preHX;
  VecPosition l_pos;//laser pos
  double laser_angle_deg;//laser angle
  bool odm_record_flag;
  bool has_typev_flag;
  bool last_go_ready_flag;
  std::vector<double> allx;
  std::vector<double> ally;
  std::vector<double> allhx;
  Point m_target, pre_robot2v;
  Point m_forecast_target;
  geometry_msgs::Twist m_speed;
  //chq added start
  std::string map_frame ;
  std::string laser_frame ;
  std::string base_link_frame;
  std::string odom_frame;
  //chq added end
  /// odom
  OrientedPoint m_leave_odm, m_cur_odom, m_pre_odom, m_delta_odm;
  static const  double ydist_err ;
  static const  double yarrived_thread ;
  static const  double xdist_near ;
  static const  double blind_move_thread ;
  static const  double angle_near ;
  static const  double v_fast  ;
  static const  double v_slow ;
  static const  double v_back_fast ;
  static const  double w_none ;
  /// some function
  bool judgeStop();
  void setSpeed(double _v, double _w);
  void setTaskStatus(int s);
  OrientedPoint recordOdom(bool switch_odm_record);

};
#endif
