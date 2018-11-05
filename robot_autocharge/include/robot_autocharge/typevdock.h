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
  void cal_pos(void);
  void track_target(PointList &scan, Point target_point);
  void updateLaser(const sensor_msgs::LaserScan::ConstPtr& ldata);
  ////接受任务回调，１，充电，２，退出充电,update date to type typeV
  void updateMotionFlags(const std_msgs::Int16  Docking_motion);
  void updateOdom(const nav_msgs::Odometry::ConstPtr& state_odata);
  void autoChargeThread(void);
  void motionPlanning();
  void Set_Task(int motion_task);
  void Set_Laser(const PointList& laser_data);//chq
  void Set_Vdata(double l1, double ang1,double min_range_dist, double dis, double outdis);
  void Set_Frame(std::string od_f,std::string base_link_f,std::string laser_f,std::string map_f ="/map"){
    odom_frame = od_f;
    base_link_frame = base_link_f;
    laser_frame = laser_f;
    map_frame = map_f;
  }

  Point get_target();//chq 
  geometry_msgs::Twist get_speed();
  int get_status();

protected:

private:
  ros::NodeHandle n;
  ros::Publisher items_marker_pub;
  ros::Publisher target_scan_pub;
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
  bool Judge_stop();
  void set_speed(double _v, double _w);
  void set_task_status(int s);
  OrientedPoint Record_odm(bool switch_odm_record);

};
#endif
