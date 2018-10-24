#include <ros/ros.h>
#include <ros/duration.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <math.h>
//#include <afxmt.h> //cmutex
#include <boost/thread.hpp>

#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_broadcaster.h>
#include "../include/robot_autocharge/MotionControlHeader.hpp"
#include "../include/robot_autocharge/point.h"
#include "../include/robot_autocharge/typevdock.h"

#define Vshape  0.5
#define MIN_R  0.03


//need to modify.it is not the real RobotShapeRect class !!!!!!!!!!!! chq ---->
//in this program it is not really used,so it is not very necessary
class RobotShapeRect{
public:
	RobotShapeRect():_front(0),_back(0),_left(0),_right(0){}
    RobotShapeRect(double front, double back, double left, double right ):_front(front),_back(back),_left(left),_right(right){};
private:
    double _front,_back,_left,_right;
};
//need to modify.it is not the real RobotShapeRect class !!!!!!!!!!!! chq-----< 

/// class dock
Docking V_dock;
RobotShapeRect safe_shape;

ros::Publisher vel_pub_;

boost::mutex mut;
boost::mutex odom_lock_;
boost::mutex scan_lock_;


bool g_status(false);
bool g_laser(false);

OrientedPoint g_leave_odm, g_cur_odom, g_pre_odom, g_delta_odm;

PointList obstacle_points_;//激光障碍物点数据



/// Set module enable
void Set_Switch(bool sflag)
{
  boost::mutex::scoped_lock l(mut);
  g_status = sflag;
}

//raw scan data transfer to vector laser scan 
PointList transLaser(const sensor_msgs::LaserScan &scan){
boost::mutex::scoped_lock l(mut);

//！这里默许前提是激光数据已经换算成到定位中心点的距离数据
  obstacle_points_.clear();
  
  for (int i = 0 ; i < scan.ranges.size() ; i += 1) //
  {
     //need to be modified
    double dist = scan.ranges[i];
    double angle = scan.angle_min + i * scan.angle_increment;
   //wangqiang to do 激光数据重新换算到两个激光头发出来的原始数据，过滤无效激光-->
    if ( dist < 0.010 ) //激光距离小于到定位中心车头过滤
      continue;
   //wangqiang to do 激光数据重新换算到两个激光头发出来的原始数据，过滤无效激光<--
    
    VecPosition obs(dist , angle , POLAR);
    Point obj(obs.getX(),obs.getY());
    obstacle_points_.push_back( obj );
  }
}

/// laser track target
PointList track_target(PointList scan, Point target_point)
{
  ROS_INFO("track_target.Track area x:%.4f,y:%.4f,With Cov:%.4f",target_point.x,target_point.y,Vshape);
  double dx,dy,dx1,dy1;

  for (int i = 0; i < scan.size(); ++i)
  {
     dx =  (scan[i].x - target_point.x);
    
     dy =  (scan[i].y - target_point.y);
    if (dx * dx + dy * dy > Vshape*Vshape)
    {
      scan.erase(scan.begin() + i);
      i--;
    }
  }
  int tmp_size = scan.size() - 1;
  for (int i = 1; i < tmp_size; ++i)
  {
    
    dx = (scan[i].x - scan[i - 1].x);
    dy = (scan[i].y - scan[i - 1].y);

    dx1 = (scan[i].x - scan[i + 1].x);
    dy1 = (scan[i].y - scan[i + 1].y);
    
    
    if (dx * dx + dy * dy  > MIN_R * MIN_R && dx1 * dx1 + dy1 * dy1 > MIN_R * MIN_R)
    {
      scan.erase(scan.begin() + i);
      i--;
    }
  }
  return scan;
}
/// update odometer 
void updateOdom(const nav_msgs::Odometry& state_odata)
{


  boost::mutex::scoped_lock l(odom_lock_);
  nav_msgs::Odometry s_odo = state_odata;
  V_dock.Set_Odm(s_odo);
  
}

/// update laser
void updateLaser(const sensor_msgs::LaserScan& ldata)
{
  boost::mutex::scoped_lock l(scan_lock_);
  if (g_status)
  {
    obstacle_points_ = transLaser(ldata);
    obstacle_points_ = track_target(obstacle_points_, V_dock.get_target());
    V_dock.Set_Laser(obstacle_points_);
  }
}

void updateMotionFlags(const std_msgs::Int16 &        Docking_motion)
{
  boost::mutex::scoped_lock l(mut);

  int tmp_reflag(0);
  ros::Rate r(1);
  ROS_INFO("updateMotionFlags.get Navi_task is:%d ",Docking_motion.data);
  const int dock_m = Docking_motion.data;
  switch (dock_m)
  {

    case 1:    tmp_reflag = 1;  ROS_INFO("updateMotionFlags.Get Charge IN task");    break;
    case 2:    tmp_reflag = 2;  ROS_INFO("updateMotionFlags.Get Charge OUT task");    break;
    default:  tmp_reflag = 0;    break;
  }
  if (tmp_reflag != 0)
  {
    r.sleep();
    Set_Switch(true);
    V_dock.Set_Task(tmp_reflag);
  }
}

void AtuoCharge_Navi(void ){
	while (true)
		  {
			  ros::Duration(0.01).sleep();
			  if (g_status)
			  {
				  V_dock.Run_Navigation();
				  /// pub speed
				  vel_pub_.publish(V_dock.get_speed());
				  /// update task status
				  int status_flag = V_dock.get_status();
				  if (status_flag == 4)
				  {
					  ROS_INFO("robot_autocharge_node.Finish update task status, In success~ go sleeping...");
					  Set_Switch(false);
				  }
				  if (status_flag == 5)
				  {
					  /// pub common data
					  ROS_INFO("robot_autocharge_node.Finish update task status, Out success~ go sleeping...");
					  Set_Switch(false);
				  }
				  if (status_flag == 999)
				  {
					  ROS_INFO("robot_autocharge_node.Finish update task status, field~ go sleeping...");
					  Set_Switch(false);
				  }
				  if (status_flag == 110)
				  {
					 ROS_INFO("robot_autocharge_node.Can Not Fine Type V Dock");
				  }
				  if (status_flag == 9999)
				  {
					  ROS_INFO("robot_autocharge_node.Finish update task status, field~ go sleeping...");
					  Set_Switch(false);
				  }
			  }
		  }

}




int main(int argc, char ** argv){

  ros::init(argc, argv, "robot_autocharge_node");
  ros::NodeHandle nh;
  ros::Rate r(1);
  ros::Subscriber scan_sub = nh.subscribe("scan",1,updateLaser);
  ros::Subscriber odo_sub = nh.subscribe("odom",1,updateOdom);
  ros::Subscriber autocharge_task_sub = nh.subscribe("autocharge_task",1,updateMotionFlags);
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  ROS_INFO("robot_autocharge_node init done.");
  ros::Duration(0.5).sleep();
  ROS_INFO("robot_autocharge_node Everything is ok, now go...");

  double lendth_recharge(0.2), angle_recharge(150), last_dis(0), out_dis(0.7);
  string tmp_shape;
  /*
  READ_PARAM(lendth_recharge); // chq charge object length?
  READ_PARAM(angle_recharge);  // chq charge object ang 
  READ_PARAM(last_dis);        // chq charge in task 累计调整距离最大值  
  READ_PARAM(out_dis);         // chq charge out task 累计调整距离最大值 
  READ_PARAM(tmp_shape);       // chq robot本体 车身尺寸 
  */
  std::vector<double> safe;

  // front back left right
  safe.push_back(0.7);
  safe.push_back(0.7);
  safe.push_back(0.48);
  safe.push_back(0.48);
  
  safe_shape = RobotShapeRect(safe[0], safe[1], safe[2], safe[3]);
  /// Set v data
  V_dock.Set_Vdata(lendth_recharge,  angle_recharge, last_dis, out_dis);

  /*chq auto charge main loop*/
  boost::thread t_navicharge(AtuoCharge_Navi);

  ros::spin();
}