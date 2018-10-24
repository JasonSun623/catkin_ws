#ifndef TYPEVDOCK_H
#define TYPEVDOCK_H

#pragma once
#define MAX_ANGLE_ERROR    20.0        // m_angle - MAX_ANGLE_ERROR MUST >  90 deg
                                       // m_angle + MAX_ANGLE_ERROR MUST < 180 deg
#define MIN_LENGTH_ERROR  0.07         // length error

#include "point.h"
#include "laser2line.h"
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>

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
  void Run_Navigation();
  void Set_Task(int motion_task);
  void Set_Laser(PointList laser_data);//chq 
  void Set_Vdata(double l1, double ang1, double dis, double outdis);
  void Set_Odm(nav_msgs::Odometry odm_data);

  Point get_target();//chq 
  geometry_msgs::Twist get_speed();
  int get_status();

protected:

private:

  enum m_status
  {
    AT_BEGIN,
    ALMOST_LAST,
    FINISH,
  };
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
  bool odm_record_flag;
  bool has_typev_flag;
  bool last_go_ready_flag;
  std::vector<double> allx;
  std::vector<double> ally;
  std::vector<double> allhx;
  Point m_target, pre_robot2v;;
  Point m_forecast_target;
  geometry_msgs::Twist m_speed;
  /// odom
  OrientedPoint m_leave_odm, m_cur_odom, m_pre_odom, m_delta_odm;
  //mutex chq
  boost::mutex mut;

  /// some function
  bool Judge_stop();
  void set_speed(double _v, double _w);
  void set_task_status(int s);
  OrientedPoint Record_odm(bool switch_odm_record);

};
#endif
