//#include "opencv2/opencv.hpp"
//#include "cvtools/cv_plot.h"
#include <ros/ros.h>
#include "typevdock.h"
 const  double Docking::ydist_err = 0.7;
 const  double Docking::yarrived_thread = 0.05;
 const  double Docking::xdist_near = 0.8;
 const  double Docking::blind_move_thread = 0.2;
 const  double Docking::angle_near = 0.5;
 const  double Docking::v_fast = 0.05;
 const  double Docking::v_slow = 0.03;
 const  double Docking::v_back_fast = -0.1;
 const  double Docking::w_none = 0.0;
Docking::Docking(void)
{
  charge_status = false;
  ros::NodeHandle private_nh("~");
  odom_topic = "wheel_diff_controller/odom";
  scan_topic = "scan";
  cmd_vel_topic = "wheel_diff_controller/cmd_vel";

  map_frame = "map" ;
  laser_frame = "hokuyo_link";
  base_link_frame = "base_link";
  odom_frame = "odom";
  init_target_dist = 1.5;
  private_nh.param("scan_topic",scan_topic,scan_topic);
  private_nh.param("odom_topic",odom_topic,odom_topic);
  private_nh.param("cmd_vel_topic",cmd_vel_topic,cmd_vel_topic);

  private_nh.param("map_frame",map_frame,map_frame);
  private_nh.param("laser_frame",laser_frame,laser_frame);
  private_nh.param("base_link_frame",base_link_frame,base_link_frame);
  private_nh.param("odom_frame",odom_frame,odom_frame);
  private_nh.param("init_target_dist",init_target_dist,init_target_dist);

  scan_sub = n.subscribe(scan_topic,1,&Docking::updateLaser,this);
  odo_sub = n.subscribe(odom_topic,1,&Docking::updateOdom,this);
  autocharge_task_sub = n.subscribe("autocharge_task",1,&Docking::updateMotionFlags,this);
  items_marker_pub = n.advertise<visualization_msgs::Marker>("dock_items_marker",1,true);

  vel_pub_ = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
  target_scan_pub = n.advertise<visualization_msgs::Marker>("dock_targets_marker",1,true);
}


Docking::~Docking(void)
{
}
//record odom velocity and w
void Docking::updateOdom(const nav_msgs::Odometry& state_odata)
{
  boost::mutex::scoped_lock l(odom_lock_);
  nav_msgs::Odometry s_odo = state_odata;
  //m_cur_odom.x = s_odo.twist.twist.linear.x;
  //m_cur_odom.y = s_odo.twist.twist.linear.x;
  //m_cur_odom.theta = s_odo.twist.twist.angular.z;

  m_cur_odom.x = state_odata.pose.pose.position.x;
  m_cur_odom.y = state_odata.pose.pose.position.y;
  geometry_msgs::Quaternion quat = state_odata.pose.pose.orientation;
  double yaw = tf::getYaw(quat);
  m_cur_odom.theta = yaw;
  ROS_INFO("Docking::updateOdom.(x,y,angle):(%.6f,%.6f,%.6f)",m_cur_odom.x,m_cur_odom.y,m_cur_odom.theta );
}
/// 筛选激光
void Docking::track_target(PointList &scan, Point target_point)
{
  ROS_INFO("track_target.Track area x:%.4f,y:%.4f,With Cov:%.4f",target_point.x,target_point.y,Vshape);
  double dx,dy,dx1,dy1;
  double var,var1,var_limit;
  for (int i = 0; i < scan.size(); ++i)
  {
     dx = (scan[i].x - target_point.x);
     dy = (scan[i].y - target_point.y);
     var = dx * dx + dy * dy;
    if (var > Vshape*Vshape)
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
    var = dx * dx + dy * dy;
    var1 = dx1 * dx1 + dy1 * dy1;
    var_limit = MIN_R * MIN_R;
    //如果连续的相邻激光点超过阈值，认为不是直线
    if (var  > var_limit && var1 > var_limit)
    {
      scan.erase(scan.begin() + i);
      i--;
    }
  }
 static visualization_msgs::Marker items_maker;
 items_maker.points.clear();
 static uint32_t shape = visualization_msgs::Marker::POINTS;
 ///common info for item
 items_maker.header.frame_id="odom";
 items_maker.type = shape;
 items_maker.ns = "odom_dock_space";
 items_maker.action = visualization_msgs::Marker::ADD;
 // Set the scale of the marker -- 1x1x1 here means 1m on a side
 items_maker.scale.x = 0.05;
 items_maker.scale.y = 0.05;
 items_maker.scale.z = 0.05;
 // Set the color -- be sure to set alpha to something non-zero!
 //DarkOrchid	153 50 204
 items_maker.color.r = 255;
 items_maker.color.g = 255;
 items_maker.color.b = 0;
 items_maker.color.a = 1.0;
 items_maker.lifetime = ros::Duration(1000000);
 static tf::TransformListener tf_listenter;
 tf::StampedTransform transform;
 try{
   tf_listenter.waitForTransform(odom_frame,laser_frame,ros::Time(0),ros::Duration(10.0));
    tf_listenter.lookupTransform(odom_frame,laser_frame,ros::Time(0),transform);
 }
 catch (tf::TransformException ex){
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
 }

 double laser_pos_x = transform.getOrigin().x();
 double laser_pos_y = transform.getOrigin().y();
 tf::Quaternion quat = transform.getRotation();
 double roll, pitch, yaw;
 tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
 double laser_angle_deg = Rad2Deg(yaw);
 VecPosition l_pos(laser_pos_x,laser_pos_y);

  for(int i = 0; i < scan.size(); i++){
    items_maker.id = i;
    items_maker.header.stamp = ros::Time::now();
    VecPosition start(scan[i].x,scan[i].y);
    VecPosition rot = start.rotate(laser_angle_deg);
    VecPosition pos = rot + l_pos;
    geometry_msgs::Point p;
    p.x = pos.getX();
    p.y = pos.getY();
    p.z = 0;
    items_maker.points.push_back(p);
  }
  target_scan_pub.publish(items_maker);
}
/// 记录最新激光数据
void Docking::updateLaser(const sensor_msgs::LaserScan& ldata)
{
  boost::mutex::scoped_lock l(scan_lock_);
  if (charge_status)
  {
    obstacle_points_.clear();
    for (int i = 0 ; i < ldata.ranges.size() ; i += 1) //
    {
       //need to be modified
      double dist = ldata.ranges[i];
      double angle = ldata.angle_min + i * ldata.angle_increment;
     //wangqiang to do 激光数据重新换算到两个激光头发出来的原始数据，过滤无效激光-->
      if ( dist < 0.010 ) //激光距离小于到定位中心车头过滤
        continue;
     //wangqiang to do 激光数据重新换算到两个激光头发出来的原始数据，过滤无效激光<--
      VecPosition obs(dist , Rad2Deg(angle), POLAR);//unit must be deg
      Point obj(obs.getX(),obs.getY());
      obstacle_points_.push_back( obj );
    }
    track_target(obstacle_points_, get_target());
    Set_Laser(obstacle_points_);
  }
}
///接收新任务
void Docking::updateMotionFlags(const std_msgs::Int16  Docking_motion)
{
  boost::mutex::scoped_lock l(mut);
  int tmp_reflag(0);
  ros::Rate r(1);
  ROS_INFO("updateMotionFlags.get Navi_task is:%d ",Docking_motion.data);
  const int dock_m = Docking_motion.data;
  switch (dock_m)
  {

    case 1:
      tmp_reflag = 1;
      ROS_INFO("updateMotionFlags.Get Charge IN task");
      break;
    case 2:
      tmp_reflag = 2;
      ROS_INFO("updateMotionFlags.Get Charge OUT task");
      break;
    default:
      tmp_reflag = 0;
      break;
  }
  if (tmp_reflag != 0)
  {
    r.sleep();///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    Set_Switch(true);
    Set_Task(tmp_reflag);
  }
}
/// judge stop
bool Docking::Judge_stop()
{
  m_task_status =ChargeStop;    // stopped
	return false;
}

/// 配置Vtype属性
void Docking::Set_Vdata(double l1, double ang1,double min_range_dist, double dis, double outdis)
{
  m_line_rechage = l1;          //V Type length for one edge chq
  m_angle_recharge = ang1;	    //V Type Angle(deg) chq
  m_lastdis = dis;              //the dist bet robot and v type when starting charging task chq
  m_outdis = outdis;            //the dist bet robot and v type when start outting task chq
  MIN_R = min_range_dist;
  tan_min_recharge = tan(Deg2Rad(m_angle_recharge - MAX_ANGLE_ERROR));
  tan_max_recharge = tan(Deg2Rad(m_angle_recharge + MAX_ANGLE_ERROR));
  odm_record_flag = false;      //do not have recorded odom data
  has_typev_flag = false;       //do not have found typev
	
  ROS_INFO("Docking::Set V Data.tan_min_recharge:%.4f, tan_max_recharge:%.4f, odm_record_flag:%d, has_typev_flag:%d",
    tan_min_recharge,tan_max_recharge, odm_record_flag, has_typev_flag);
}

/// 处理目标点数据　计算偏转角
void Docking::Set_Laser(PointList laser_data)
{
	has_typev_flag = false;
  double include_angle_k,include_angle_k_modify, include_angle_test;
	m_vdetec.setLaserData(laser_data);
	m_vdetec.split_and_merge();
	m_line = m_vdetec.get_line();
  static tf::TransformListener tf_listenter;
  tf::StampedTransform transform;
  try{
    tf_listenter.waitForTransform(odom_frame,laser_frame,ros::Time(0),ros::Duration(10.0));
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  tf_listenter.lookupTransform(odom_frame,laser_frame,ros::Time(0),transform);
  double laser_pos_x = transform.getOrigin().x();
  double laser_pos_y = transform.getOrigin().y();
  tf::Quaternion quat = transform.getRotation();
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  double laser_angle_deg = Rad2Deg(yaw);
  VecPosition l_pos(laser_pos_x,laser_pos_y);
	ROS_INFO("Docking::Set_Laser.laser_data size:%d, line number:%d ",laser_data.size(), m_line.size() );
  static visualization_msgs::Marker items_maker;

  items_maker.points.clear();
  items_maker.header.frame_id=odom_frame;
  items_maker.type = visualization_msgs::Marker::LINE_STRIP;
  items_maker.ns = "map_dock_space";
  items_maker.action = visualization_msgs::Marker::ADD;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  items_maker.scale.x = 0.01;
  items_maker.scale.y = 0.01;
  items_maker.scale.z = 0.01;
  // Set the color -- be sure to set alpha to something non-zero!
  //DarkOrchid	153 50 204
  items_maker.color.r = 153;
  items_maker.color.g = 50;
  items_maker.color.b = 204;
  items_maker.color.a = 1.0;
  geometry_msgs::Point p;
	for (int i = 0; !has_typev_flag && m_line.size() > 1 &&  i < m_line.size() - 1; ++i)
	{
    double robot2vx, robot2vy;
    ///chq
    items_maker.header.stamp = ros::Time::now();
    items_maker.id = i;
    VecPosition start(m_line[i].lbegin_x,m_line[i].lbegin_y);
    VecPosition st_pos = start.rotate(laser_angle_deg)  + l_pos;
    VecPosition end(m_line[i].lend_x,m_line[i].lend_y);
    VecPosition end_pos = end.rotate(laser_angle_deg)  + l_pos;
    p.x = st_pos.getX();
    p.y = st_pos.getY();
    items_maker.points.push_back(p);
    p.x = end_pos.getX();
    p.y = end_pos.getY();
    items_maker.points.push_back(p);
    double deg_a_k,deg_a_modify;
    double test_min_err = 180+Rad2Deg( atan(tan_min_recharge));
    double test_max_err = 180+Rad2Deg( atan(tan_max_recharge));
    if (1.0 != m_line[i].line_k*m_line[i + 1].line_k)//如果不互余
    {
      include_angle_k = (m_line[i + 1].line_k - m_line[i].line_k) / (1.0 - m_line[i].line_k*m_line[i + 1].line_k);
      deg_a_k =  180+Rad2Deg( atan(include_angle_k));

    }
    if (-1.0 != m_line[i].line_k*m_line[i + 1].line_k && m_line[i].line_k*m_line[i + 1].line_k<0)//如果不vertical
    {
      include_angle_k_modify = (m_line[i + 1].line_k - m_line[i].line_k) / (1.0 + m_line[i].line_k*m_line[i + 1].line_k);
      if(include_angle_k_modify>0)include_angle_k_modify=-include_angle_k_modify;
       deg_a_modify =  180+Rad2Deg( atan(include_angle_k_modify));
    }

    if (
        //fabs(m_line[i].line_length - m_line_rechage) < MIN_LENGTH_ERROR &&   //chq disabled temp
        //fabs(m_line[i + 1].line_length - m_line_rechage) < MIN_LENGTH_ERROR && //chq disabled temp
        //include_angle_k > tan_min_recharge && include_angle_k < tan_max_recharge
        include_angle_k_modify > tan_min_recharge && include_angle_k_modify < tan_max_recharge
        )//如果两条直线的夹角与ｖ板夹角不超过阈值
    {
      /// Todo: get the point
      /// 两线交点
      double a1 = (m_line[i].lbegin_x - m_line[i].lend_x) / (m_line[i].lbegin_y - m_line[i].lend_y);
      double b1 = m_line[i].lbegin_x - a1 * m_line[i].lbegin_y;
      double a2 = (m_line[i+1].lbegin_x - m_line[i+1].lend_x) / (m_line[i+1].lbegin_y - m_line[i+1].lend_y);
      double b2 = m_line[i + 1].lbegin_x - a2 * m_line[i + 1].lbegin_y;
      robot2vy = (b1 - b2) / (a2 - a1);
      robot2vx = a1 * robot2vy + b1;

      //////////////////////////////////////////////////////////////////////////
      if (pre_robot2v.x != init_target_dist)
      {
        if (fabs(robot2vx - pre_robot2v.x) > 0.07)
          robot2vx = pre_robot2v.x;
        if (fabs(robot2vy - pre_robot2v.y) > 0.07)
          robot2vy = pre_robot2v.y;
      }

      /// todo some 0 judge
      /// tanα=(k-k1)/(1+kk1)=(k2-k)/(1+kk2)
      double k1 = MAXK;
      double a11 = atan((m_line[i].line_k - k1)/(1 + m_line[i].line_k*k1));//求k1与垂线的夹角
      a11 = M_PI / 2.0 - a11;
      //double jia = atan(include_angle_k);
      //if (jia < 0)	jia = M_PI + jia;
      double jia = atan(m_line[i].line_k) - atan(m_line[i + 1].line_k);
      HX = 1.5 * M_PI - jia / 2.0 - a11;
      ROS_INFO_STREAM("Docking::Set_Laser."
                      << "\n patential v type.angle bet two lines(deg):" << RAD2DEG(atan(include_angle_k))
                      << "\n cross pos : (" << robot2vx << "," << robot2vy << ")."
                      << "\n ang bet l1 and vertical line(deg): " << RAD2DEG(a11)
                      << "\n turn angle(deg): " <<  VecPosition::normalizeAngle( RAD2DEG(a11) )
                      );

      m_target.x = robot2vx * sin(HX) - robot2vy * cos(HX);
      m_target.y = robot2vx * cos(HX) + robot2vy * sin(HX);
      ROS_INFO_STREAM("Docking::Set_Laser.The patential v type end.the target in motion control cord: (" << m_target.x << "," << m_target.y << ")");
      //最新数据做比较，选较大的一个
      allx.push_back(m_target.x);
      if (allx.size() > 1)
        m_target.x = fabs(allx[allx.size() - 1]) < fabs(allx[allx.size() - 2]) ? fabs(allx[allx.size() - 2]) : fabs(allx[allx.size() - 1]);
      /// a < b ? b : a

      ally.push_back(m_target.y);
      if (ally.size() > 2)
        m_target.y = (ally[ally.size() - 1] + ally[ally.size() - 2] + ally[ally.size() - 3]) / 3.0;

      allhx.push_back(HX);
      if (allhx.size() > 2)
        HX = (allhx[allhx.size() - 1] + allhx[allhx.size() - 2] + allhx[allhx.size() - 3]) / 3.0;
            
  //    LOG_COUT_INFO(m_vdock_logger, "robot2v: " << robot2vx << "," << robot2vy << "," << HX / M_PI*180.0);
  //    LOG_COUT_INFO(m_vdock_logger, "target: " << m_target.x << "," << m_target.y << " HX: " << HX / M_PI*180.0);

      m_forecast_target.x = 0.0;
      m_forecast_target.y = 0.0;

      pre_targetx = m_target.x;
      pre_targety = m_target.y;
      preHX = HX;

      pre_robot2v.x = robot2vx;
      pre_robot2v.y = robot2vy;

      has_typev_flag = true;
      if (m_task_status == TypeVNotFound)//chq Can Not Fine Type V Dock
      {
        set_task_status(m_reme_status);//reset task
        ROS_INFO_STREAM("Find tye v_dock again, reset task_status to " << m_reme_status);
      }
    }
    else
    {
      double forecast_t[2];
      forecast_t[0] = pre_targetx - m_forecast_target.x;
      forecast_t[1] = pre_targety + m_forecast_target.y;

      ROS_INFO_STREAM("Docking::Set_Laser.forecast: " << pre_targetx << "-" << m_forecast_target.x << " , " << pre_targety << "+" << m_forecast_target.y);

      m_target.x = forecast_t[0];
      m_target.y = forecast_t[1];
      HX = preHX;
    }
    ROS_INFO_STREAM("Docking::Set_Laser End. target(x,y):(" << m_target.x << "," << m_target.y << ") and pre target(x,y): " << pre_targetx << "," << pre_targety);
	}
  items_marker_pub.publish(items_maker);
}

/// handle task order
void Docking::Set_Task(int motion_task)
{
	odm_record_flag = false;
	has_typev_flag = false;
	last_go_ready_flag = false;
	
  pre_robot2v.x = init_target_dist;
	pre_robot2v.y = 0.0;
  m_target.x = pre_robot2v.x;
  m_target.y = pre_robot2v.y;
	
  m_forecast_target.x = 0.0;
  m_forecast_target.y = 0.0;
  pre_targetx = pre_robot2v.x;
  pre_targety = pre_robot2v.y;

  preHX = M_PI / 2.0;
	m_task_flag = motion_task;
  m_task_status = ChargeBegin;
	allx.clear();
	ally.clear();
	allhx.clear();
  ROS_INFO("Docking::Set_Task.Set TaskChargeBegin");
}

/// set task status
int Docking::get_status()
{
	return m_task_status;
}

/// handle output speed
geometry_msgs::Twist Docking::get_speed()
{
	  geometry_msgs::Twist cmd_vel;
	
	  cmd_vel.linear.x = m_speedv;
	  cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
	  cmd_vel.angular.z = m_speedw;

	return cmd_vel;
}

/// get target point
Point Docking::get_target()
{
	//if (m_task_flag == 0)
	//{
  //	m_target.x = init_target_dist;
	//	m_target.y = 0.0;
	//}
	//if (fabs(m_target.x) > 2.0)	m_target.x = 2.0;
	//if (fabs(m_target.y) > 1.0) m_target.y = m_target.y / fabs(m_target.y);

	return pre_robot2v;
}


/// record odm
OrientedPoint Docking::Record_odm(bool switch_odm_record)
{
	if (switch_odm_record)
  {
    ROS_INFO("Docking::Record_odm.m_cur_odom(x,y,angle):(%.6f,%.6f,%.6f)",m_cur_odom.x,m_cur_odom.y,m_cur_odom.theta );
    ROS_INFO("Docking::Record_odm.m_pre_odom(x,y,angle):(%.6f,%.6f,%.6f)",m_pre_odom.x,m_pre_odom.y,m_pre_odom.theta );
    m_delta_odm = absoluteDifference(m_cur_odom, m_pre_odom);
    m_leave_odm = absoluteSum(m_leave_odm, m_delta_odm);
    m_pre_odom = m_cur_odom;
    return m_leave_odm;
	}
	else
    return OrientedPoint(0,0,0);
}

/// set speed
void Docking::set_speed(double _v, double _w)
{
	m_speedv = _v;	m_speedw = _w;
}
/// set task status
void Docking::set_task_status(int s)
{
	m_task_status = s;
}
void Docking::autoChargeThread(void){
  while (n.ok())
      {
        ros::Duration(0.01).sleep();
        //if in charge in/out process chq
        if (charge_status)
        {
          motionPlanning();
          /// pub speed
          vel_pub_.publish(get_speed());
          /// update task status
          int status_flag = get_status();
          if (status_flag == ChargeInSuc)
          {
            ROS_INFO("robot_autocharge_node.Finish update task status, In success~ go sleeping...");
            Set_Switch(false);
          }
          if (status_flag == ChargeOutSuc)
          {
            /// pub common data
            ROS_INFO("robot_autocharge_node.Finish update task status, Out success~ go sleeping...");
            Set_Switch(false);
          }
          if (status_flag == AtField1)
          {
            ROS_INFO("robot_autocharge_node.Finish update task status, field~ go sleeping...");
            Set_Switch(false);
          }
          if (status_flag == TypeVNotFound)
          {
           ROS_INFO("robot_autocharge_node.Can Not Fine Type V Dock");
          }
          if (status_flag == AtField2)
          {
            ROS_INFO("robot_autocharge_node.Finish update task status, field~ go sleeping...");
            Set_Switch(false);
          }
        }
        ros::spinOnce();
      }
}
void Docking::motionPlanning()
{
	double _speedv(0.0);
	double _speedw(0.0);

  //ROS_INFO("Docking::motionPlanning for Charge Move Task");

	// chq m_task_status = 0 意味着exit navi loop
  if (m_task_status != NoTask )
	{
    //if (!has_typev_flag && m_task_status != 2 && m_task_status != 3)
    //{
    //	if (m_task_status != 110)
    //    m_reme_status = m_task_status;
    //	set_task_status(110);
    //	LOG_COUT_INFO(m_vdock_logger, "Can not find type v_dock set task_status 110, and remember now task_status " << m_reme_status);
    //}
    // chq 进入充电位 任务
    if (m_task_flag == ChargeInTask)
    {
      /// to do some filter
      //chq =2 新任务开始状态
      if (m_task_status == ChargeBegin)// chq when we get a new task, the m_task_status equal to 2
      {
        ROS_INFO("Docking::Run_Navigation.docking target x:%f, y:%f, HX:%f ",m_target.x, m_target.y, HX / M_PI*180.0);
        //chq 如果发现充电桩
        if (has_typev_flag)
        {
            //chq 若发生y方向偏差过大现象，失败
          if (fabs(m_target.y) > ydist_err)
          {
            m_task_flag = NoTask;
            ROS_ERROR("Docking::Run_Navigation.has_typev_flag = true.but m_target.y:%f, ERROR TOO LARGE！！！", m_target.y);
            set_task_status(AtField2);
          }
          //chq 否则，固定线速度，角速度通过实时的偏差Hx与target.y计算
          else
          {
            _speedv = v_fast;
            _speedw = HX - M_PI / 2.0 + m_target.y * 2.0;
            //	_speedw = 1.0 * m_target.y;
            //	_speedv = 0;
            //	_speedw = 0;

            //chq 如果充电模式下，x方向偏差小于0.8m 进入修正调整角度模式
            if (m_target.x < xdist_near)
            {
              odm_record_flag = false;
              set_task_status(NearMode);
              ROS_ERROR("Docking::Run_Navigation.has_typev_flag = true.m_target.x :%f < 0.8 Enter Modified HX Status", m_target.x );
            }
          }
          odm_record_flag = false;
        }
        //chq 如果未发现充电桩
        else
        {
            //chq 重置里程计计数，重新计算里程计累积量
          if (!odm_record_flag)
          {
            ROS_INFO("Docking::motionPlanning.no found type v.last not record odom.now record!");
            m_pre_odom = m_cur_odom;
            m_leave_odm.x = 0.0;
            m_leave_odm.y = 0.0;
            m_leave_odm.theta = 0.0;
            odm_record_flag = true;
          }
          double record_dis = Record_odm(odm_record_flag).mod();
          //chq 如果累计调整超过0.2m 还未找到charge,stop
          if (record_dis > blind_move_thread)	/// if has no typevdock above 20cm  stop it
          {
            _speedv = 0.0;
            _speedw = 0.0;
            set_task_status(AtField1);
            ROS_ERROR("Docking::Run_Navigation.has_typev_flag = false.record_dis %f > 0.2.Long Dis Can Not Find Type V Dock", record_dis);
          }
          //chq 否则，恒速前进，计算危险距离
          else
          {
            m_forecast_target.y = Record_odm(odm_record_flag).x * cos(HX) + Record_odm(odm_record_flag).y * sin(HX);
            m_forecast_target.x = Record_odm(odm_record_flag).x * sin(HX) - Record_odm(odm_record_flag).y * cos(HX);
            _speedv = v_fast;
            _speedw = 0.0;
            ROS_ERROR("Docking::Run_Navigation.record_dis: %f, m_forecast_target.x:%f, m_forecast_target.y:%f.Danger~ ", record_dis, m_forecast_target.x, m_forecast_target.y);
          }
          //chq 如果x方向偏差<0.8，进入修正模式(by blind)
          if (m_target.x < xdist_near)
          {
            odm_record_flag = false;
            set_task_status(NearMode);
            ROS_INFO("Docking::Run_Navigation.Enter Modified HX Status BY Blind Moving...");
          }
        }
      }
      //chq x方向偏差已较小，进入角度修正模式
      if (m_task_status == NearMode) // modified
      {
        ROS_INFO("Docking::Run_Navigation.m_task_status == 35.target x:%f, y:%f, Hx:%f ", m_target.x, m_target.y, HX / M_PI*180.0);
        _speedv = 0.0;
        _speedw = HX - M_PI / 2.0;
        if (fabs(Rad2Deg(HX - M_PI/2)) < angle_near)//若距离较小，且角度偏差已经很小，角速度设为0
        {
          _speedw = 0.0;
          set_task_status(NearDistMode);
          ROS_INFO("Docking::Run_Navigation.Enter Last_Dist Status");
        }
      }
      //chq x距离偏差已较小且角度调整到位,进入 直行到充电桩 模式
      if (m_task_status == NearDistMode)	// last dis
      {
        //chq 如果y方向发生偏差较大情况，失败
        if (!last_go_ready_flag && fabs(m_target.y) > yarrived_thread)
        {
          m_task_flag = NoTask;
          ROS_ERROR("Docking::Run_Navigation.m_task_status == 3.LAST Y ERROR:%f > 0.05, IS TOO LARGE",fabs(m_target.y));
          set_task_status(AtField1);
        }
        last_go_ready_flag = true;
         //chq 重置并累计计算里程计距离值
        if (!odm_record_flag)
        {
           ROS_INFO("Docking::motionPlanning.go ointo NearDistMode.last not record odom.now record!");
          m_pre_odom = m_cur_odom;
          m_leave_odm.x = 0.0;
          m_leave_odm.y = 0.0;
          m_leave_odm.theta = 0.0;
          odm_record_flag = true;
        }
        double record_dis = Record_odm(odm_record_flag).mod();
        ROS_INFO("Docking::Run_Navigation.Record odm dist:%f ", record_dis);
        //chq 如果小于设定的限制行驶距离，固定速度前进
        if (record_dis < m_lastdis)
        {
          _speedv = v_slow;
          _speedw = 0.0;
        }
        //chq 否则，停止并置任务为完成状态
        else
        {
          m_task_flag = NoTask;
          odm_record_flag = false;
          _speedv = 0.0;
          _speedw = 0.0;
          set_task_status(ChargeInSuc);	// in finish
          ROS_INFO("Docking::Run_Navigation.Charge In Success!");
        }
      }
    }
    // chq 离开充电位 任务
    if (m_task_flag == ChargeOutTask)
    {   //chq 重置并累计记录里程计距离值
      if (!odm_record_flag)
      {
        ROS_INFO("Docking::motionPlanning.charge out task begin.last not record odom.now record!");
        m_pre_odom = m_cur_odom;
        m_leave_odm.x = 0.0;
        m_leave_odm.y = 0.0;
        m_leave_odm.theta = 0.0;
        odm_record_flag = true;
      }
      //chq 小于限定行驶距离值，恒速倒退
      if (Record_odm(odm_record_flag).mod() < m_outdis)
      {
        _speedv = v_back_fast;
        _speedw = -0.0;
      }
      //chq 否则，停止，置为退出成功状态
      else
      {
        m_task_flag = NoTask;
        odm_record_flag = false;
        _speedv = 0.0;
        _speedw = 0.0;
        set_task_status(ChargeOutSuc);	// out finish
        ROS_INFO("Docking::Run_Navigation.Charge Out Success!");
      }
    }
	}

//	if (Judge_stop())
//	{
//    set_speed(0.0, 0.0);
//    LOG_COUT_INFO(m_vdock_logger, "Laser Stop");
//	}
//	else
//	{
//    set_speed(_speedv, _speedw);
//	}
    //chq 下发速度给控制器
	set_speed(_speedv, _speedw);
	ros::Duration(0.03).sleep();
}

