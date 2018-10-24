#ifndef _SURO_MOTION_CONTROL_HEADER_taotao_2011_02_13_
#define _SURO_MOTION_CONTROL_HEADER_taotao_2011_02_13_

#include "Geometry.h"
#include <iostream>
#include <string.h>
#include <stdio.h>

template<typename T, int N>
class ObjectArray {
public:
  T data[N];
};

struct OrientPos {
  VecPosition pos;
  double angle;

  inline void std_out() {
    std::cout << "x= " << pos.getX() << " y= " << pos.getY() << " angle= "
        << angle << std::endl;
  }

  OrientPos(double x = 0.0, double y = 0.0, double a = 0.0) :
    pos(x, y), angle(a) {

  }
};

struct TargetPos {
  VecPosition pos;
  double angle;
  double speed;

  TargetPos(double x = 0.0, double y = 0.0, double a = 0.0, double v = 0.0) :
    pos(x, y), angle(a), speed(v) {
  }
};

struct RobotPose2D {
  OrientPos pose_; //锟芥储x锟斤拷y锟斤拷锟角讹拷锟斤拷维状态
  char map_name_[50];//锟芥储锟斤拷图锟斤拷

  inline void set_pose(const OrientPos &p) {
    pose_ = p;
  }

  inline OrientPos pose() {
    return pose_;
  }

  inline void set_map_name(const std::string &s) {
#ifdef _WIN32
    strcpy_s(map_name_ , s.c_str());
#else
    strcpy(map_name_, s.c_str());
#endif
  }

  inline std::string map_name() {
    return std::string(map_name_);
  }
};

/*
 * 矢锟斤拷锟斤拷式锟斤拷示锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷图锟叫碉拷位锟斤拷
 */
class UPL {
public:
  UPL(int sid = -1, int eid = -1, double t = 0.0f, bool forwards = true,
      std::string map = "") :
    start_id_(sid), end_id_(eid), percent_(t), forward_(forwards)
  /*map_name_(map)*/{
    ;
  }
  ~UPL() {
    ;
  }
  int start_id_;
  int end_id_;
  bool forward_; // 锟斤拷时锟斤拷锟矫ｏ拷锟斤拷start_id_ -> end_id_锟斤拷示锟斤拷锟斤拷
  double percent_;
  //  std::string map_name_; // 锟皆猴拷锟斤拷展锟斤拷
};

struct ControlParams {
  ControlParams() :
    max_vx_(-1.0), max_vy_(-1.0), max_w_(-1.0), max_av_(-1.0),
        max_aw_(-1.0) {
    ;
  }
  ControlParams(double m_vx, double m_vy, double m_w, double m_av,
      double m_aw) :
    max_vx_(m_vx), max_vy_(m_vy), max_w_(m_w), max_av_(m_av), max_aw_(m_aw) {
    ;
  }
  bool IsValid() {
    if (max_vx_ <= 0 || max_vy_ <= 0 || max_w_ <= 0 || max_av_ <= 0
        || max_aw_ <= 0) {
      return false;
    } else
      return true;
  }
  std::string DebugString() {
    char buf[128];
    int j = sprintf(buf, "[v, w, va, wa] = %f, %f, %f, %f ", max_vx_,
        max_w_, max_av_, max_aw_);
    std::string s;
    if (j > 0)
      s.assign(buf);
    return s;
  }
  double max_vx_, max_vy_, max_w_; // 小锟斤拷/锟斤拷锟斤拷0 锟斤拷使锟斤拷锟斤拷锟斤拷锟侥硷拷锟叫碉拷锟斤拷锟矫ｏ拷锟斤拷锟斤拷使锟矫该诧拷锟斤拷锟斤拷锟斤拷锟剿讹拷锟斤拷锟斤拷
  double max_av_, max_aw_; // 锟斤拷锟劫度匡拷锟狡ｏ拷 锟斤拷位 m/s, rad/s , m/s^2, rad/s^2
};

struct OrientTarget {
#ifdef _WIN32
  static const int kNavigationType_AvoidObstacle = 1;
  static const int kNavigationType_AccurateTune = 2;
  static const int kNavigationType_Straight = 3;
  static const int kNavigationType_BiDirectionStraight = 4;
  static const int kNavigationType_BezierEdge = 5;

  static const int kEndConditionTypeDistOnly = 1; //锟斤拷止锟斤拷锟斤拷锟叫讹拷锟斤拷锟酵讹拷锟藉：锟斤拷锟叫断伙拷锟斤拷锟斤拷位锟矫猴拷某锟教讹拷位锟矫的撅拷锟斤拷锟角凤拷小锟斤拷一锟斤拷值
  static const int kEndConditionTypeDistAndAngle = 2; //锟斤拷止锟斤拷锟斤拷锟叫讹拷锟斤拷锟酵讹拷锟藉：锟斤拷锟斤拷锟叫断撅拷锟斤拷锟角凤拷锟斤拷锟姐，锟斤拷锟斤拷锟叫断伙拷锟斤拷锟剿筹拷锟斤拷锟角凤拷锟斤拷锟斤拷锟斤拷锟斤拷
#else
  static const int kNavigationType_AvoidObstacle;
  static const int kNavigationType_AccurateTune;
  static const int kNavigationType_Straight;
  static const int kNavigationType_BiDirectionStraight;
  static const int kNavigationType_BezierEdge;

  static const int kEndConditionTypeDistOnly; //锟斤拷止锟斤拷锟斤拷锟叫讹拷锟斤拷锟酵讹拷锟藉：锟斤拷锟叫断伙拷锟斤拷锟斤拷位锟矫猴拷某锟教讹拷位锟矫的撅拷锟斤拷锟角凤拷小锟斤拷一锟斤拷值
  static const int kEndConditionTypeDistAndAngle; //锟斤拷止锟斤拷锟斤拷锟叫讹拷锟斤拷锟酵讹拷锟藉：锟斤拷锟斤拷锟叫断撅拷锟斤拷锟角凤拷锟斤拷锟姐，锟斤拷锟斤拷锟叫断伙拷锟斤拷锟剿筹拷锟斤拷锟角凤拷锟斤拷锟斤拷锟斤拷锟斤拷
#endif

  OrientPos target_;
  UPL tar_; // kNavigationType_BezierEdge使锟斤拷锟斤拷锟斤拷锟斤拷
  //   double    max_vx_, max_vy_, max_w_;  // 小锟斤拷/锟斤拷锟斤拷0 锟斤拷使锟斤拷锟斤拷锟斤拷锟侥硷拷锟叫碉拷锟斤拷锟矫ｏ拷锟斤拷锟斤拷使锟矫该诧拷锟斤拷锟斤拷锟斤拷锟剿讹拷锟斤拷锟斤拷
  //   double    max_av_, max_aw_;  // 锟斤拷锟劫度匡拷锟狡ｏ拷 锟斤拷位 m/s, rad/s , m/s^2, rad/s^2
  ControlParams con_params_; // 20130913 锟侥筹拷一锟斤拷锟结构锟藉，锟皆猴拷锟斤拷锟斤拷锟斤拷锟斤拷删锟斤拷
  int navigation_type_;
  int end_condition_type_;
  double end_condition_dist_thres_;
  double end_condition_angle_thres_;
  bool b_consider_orientation_;
  bool b_precise_stop_; // 20130902 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷欠锟斤拷锟矫撅拷确锟斤拷锟斤拷锟斤拷品锟绞?  // true - 锟斤拷确锟斤拷锟斤拷
  // false - 锟矫达拷统锟斤拷模式锟斤拷默锟较ｏ拷
};

struct LazyTask {
  bool go_; // true - forward search
  // false - back off search
  int con_; // select branch topology way
  // 0 - no branch
  // 1 .. n - one or more branch, anti clock wise
};

#endif
