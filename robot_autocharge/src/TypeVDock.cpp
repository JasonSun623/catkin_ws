//#include "opencv2/opencv.hpp"
//#include "cvtools/cv_plot.h"
#include <ros/ros.h>
#include "typevdock.h"
 const  double Docking::ydist_err = 0.7;
 const  double Docking::yarrived_thread = 0.05;
 const  double Docking::xdist_near = 0.8;
 const  double Docking::blind_move_thread = 0.8;
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

  scan_sub = n.subscribe(scan_topic,1000,&Docking::updateLaser,this);
  odo_sub = n.subscribe(odom_topic,1000,&Docking::updateOdom,this);
  autocharge_task_sub = n.subscribe("autocharge_task",1,&Docking::callBackTask,this);
  items_marker_pub = n.advertise<visualization_msgs::Marker>("dock_items_marker",1,true);

  vel_pub_ = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
  target_scan_pub = n.advertise<visualization_msgs::Marker>("dock_targets_marker",1,true);
}


Docking::~Docking(void)
{
}
//record odom velocity and w
void Docking::updateOdom(const nav_msgs::Odometry::ConstPtr& state_odata)
{
  boost::mutex::scoped_lock l(odom_lock_);
  const nav_msgs::Odometry odo = (*state_odata);
  m_cur_odom.x = odo.pose.pose.position.x;
  m_cur_odom.y = odo.pose.pose.position.y;
  double yaw = tf::getYaw(odo.pose.pose.orientation);
  m_cur_odom.theta = yaw;
  //ROS_INFO("Docking::updateOdom.(x,y,angle):(%.6f,%.6f,%.6f)",m_cur_odom.x,m_cur_odom.y,m_cur_odom.theta );
}
/// 筛选激光
void Docking::trackTarget(PointList &scan, Point target_point)
{

  double dx,dy,dx1,dy1;
  double var,var1,var_limit;
  //与预定目标点距离超过Vｓｈａｐｅ的　过滤
  for (int i = 0; i < scan.size(); ++i)
  {
    /*disable by chq tmp
    //think 1 consided by tar cord
     //dx = (scan[i].x - target_point.x);
     //dy = (scan[i].y - target_point.y);
    //.if (var > Vshape*Vshape)
    */
    //consided by robot cord
    dx = ( scan[i].x );
    dy = ( scan[i].y );
    var = hypot(dx,dy);
    //if (var > Vshape*Vshape)//disable by chq tmp
    double var_tar = hypot(pre_robot2v.x+Vshape ,pre_robot2v.y+Vshape);
    if (var > var_tar)
    {
      scan.erase(scan.begin() + i);
      i--;
    }
  }
   ROS_INFO_STREAM(" Docking::trackTarget.after filter by tar p:(" <<  target_point.x <<"," <<  target_point.y <<").the scan size:" << scan.size());
  //激光点间距超过指定阈值的 过滤
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
    if (var  > var_limit && var1 > var_limit)
    {
      scan.erase(scan.begin() + i);
      i--;
    }
  }
 ROS_INFO_STREAM(" Docking::trackTarget.after filter by point dentisity,the scan size:" << scan.size());
 static visualization_msgs::Marker items_maker;
 items_maker.points.clear();
 static uint32_t shape = visualization_msgs::Marker::POINTS;
 ///common info for item
 items_maker.header.frame_id="odom";
 items_maker.type = shape;
 items_maker.ns = "odom_dock_space";
 items_maker.action = visualization_msgs::Marker::ADD;
 // Set the scale of the marker -- 1x1x1 here means 1m on a side
 items_maker.scale.x = 0.02;
 items_maker.scale.y = 0.02;
 items_maker.scale.z = 0.02;
 // Set the color -- be sure to set alpha to something non-zero!
 //DarkOrchid  153 50 204
 items_maker.color.r = 255;
 items_maker.color.g = 255;
 items_maker.color.b = 0;
 items_maker.color.a = 1.0;
 items_maker.lifetime = ros::Duration(1000000);
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
void Docking::updateLaser(const sensor_msgs::LaserScan::ConstPtr& ldata)
{
  boost::mutex::scoped_lock l(scan_lock_);
  const sensor_msgs::LaserScan scan = (*ldata);
  double dist;
  double angle;

  if (charge_status)
  {
    obs_p.clear();
    for (int i = 0 ; i < scan.ranges.size() ; i += 1) //
    {
       //need to be modified
      dist = scan.ranges[i];
      angle = scan.angle_min + i * scan.angle_increment;
     //wangqiang to do 激光数据重新换算到两个激光头发出来的原始数据，过滤无效激光-->
      if ( dist < 0.010 ) //激光距离小于到定位中心车头过滤
        continue;
     //wangqiang to do 激光数据重新换算到两个激光头发出来的原始数据，过滤无效激光<--
      VecPosition obs(dist , Rad2Deg(angle), POLAR);//unit must be deg
      //Point obj(obs.getX(),obs.getY());
      obs_p.push_back( Point(obs.getX(),obs.getY()) );
    }


  }
}
void Docking::calPos(void){
  static tf::TransformListener tf_listenter;
  tf::StampedTransform transform;
  try{
    tf_listenter.waitForTransform(odom_frame,laser_frame,ros::Time(0),ros::Duration(1.0));
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
  laser_angle_deg = Rad2Deg(yaw);
  l_pos.setX(laser_pos_x);
  l_pos.setY(laser_pos_y);
}
///接收新任务
void Docking::callBackTask(const std_msgs::Int16  Docking_motion)
{
  boost::mutex::scoped_lock l(mut);
  int tmp_reflag(0);
  ros::Rate r(1);
  ROS_INFO("Docking::callBackTask.get dock type arg is:%d ",Docking_motion.data);
  const int dock_m = Docking_motion.data;
  switch (dock_m)
  {

    case 1:
      tmp_reflag = 1;
      ROS_INFO("Docking::callBackTask.Get Charge IN task");
      break;
    case 2:
      tmp_reflag = 2;
      ROS_INFO("Docking::callBackTask.Get Charge OUT task");
      break;
    default:
      tmp_reflag = 0;
      break;
  }
  if (tmp_reflag != 0)
  {
    r.sleep();///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    Set_Switch(true);
    setTask(tmp_reflag);
  }
}
/// judge stop
bool Docking::judgeStop()
{
  m_task_status =ChargeStop;    // stopped
  return false;
}

/// 配置VType属性
void Docking::setVTypeProperty(double l1, double ang1,double min_range_dist, double dis, double outdis)
{
  m_line_rechage = l1;          //V Type length for one edge chq
  m_angle_recharge = ang1;      //V Type Angle(deg) chq
  m_lastdis = dis;              //the dist bet robot and v type when starting charging task chq
  m_outdis = outdis;            //the dist bet robot and v type when start outting task chq
  MIN_R = min_range_dist;
  tan_min_recharge = tan(Deg2Rad(m_angle_recharge - MAX_ANGLE_ERROR));
  tan_max_recharge = tan(Deg2Rad(m_angle_recharge + MAX_ANGLE_ERROR));
  odm_record_flag = false;      //do not have recorded odom data
  has_typev_flag = false;       //do not have found typev

  ROS_INFO("Docking::Set VType Property.tan_min_recharge:%.4f, tan_max_recharge:%.4f, odm_record_flag:%d, has_typev_flag:%d",
    tan_min_recharge,tan_max_recharge, odm_record_flag, has_typev_flag);
}

/// 处理目标点数据　计算偏转角
void Docking::searchVType(const PointList & laser_data)
{
  has_typev_flag = false;
  double include_angle_k,include_angle_k_modify, include_angle_test;
  m_vdetec.setLaserData(laser_data);
  m_vdetec.split_and_merge();
  m_line = m_vdetec.get_line();
   ROS_INFO("Docking::searchVType.laser_data size:%d, line number:%d ",laser_data.size(), m_line.size() );
  for( int i = 0; i < m_line.size(); i++){
    ROS_INFO("Docking::searchVType.line[%d]'s length:%.4f,k:%.4f",i, m_line[i].line_length,m_line[i].line_k);
  }
  static visualization_msgs::Marker items_maker;

  items_maker.points.clear();
  items_maker.header.frame_id=odom_frame;
  items_maker.type = visualization_msgs::Marker::LINE_STRIP;
  items_maker.ns = "map_dock_space";
  items_maker.action = visualization_msgs::Marker::ADD;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  items_maker.scale.x = 0.02;
  items_maker.scale.y = 0.02;
  items_maker.scale.z = 0.02;
  // Set the color -- be sure to set alpha to something non-zero!
  //DarkOrchid  153 50 204
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
    double test_min_err = 180+Rad2Deg(atan(tan_min_recharge));
    double test_max_err = 180+Rad2Deg(atan(tan_max_recharge));
    if (1.0 != m_line[i].line_k*m_line[i + 1].line_k)//如果不互余
    {
      include_angle_k = (m_line[i + 1].line_k - m_line[i].line_k) / (1.0 - m_line[i].line_k*m_line[i + 1].line_k);
      deg_a_k =  180+Rad2Deg( atan(include_angle_k));//atanx, return in the interval [-pi/2,+pi/2] radians.

    }
    if (-1.0 != m_line[i].line_k*m_line[i + 1].line_k /*&& m_line[i].line_k*m_line[i + 1].line_k<0*/)//如果不vertical
    {
      include_angle_k_modify = (m_line[i + 1].line_k - m_line[i].line_k) / (1.0 + m_line[i].line_k*m_line[i + 1].line_k);
      double include_angle_rad = getIncludeAngleRad(m_line[i + 1].line_k, m_line[i].line_k);
      if(include_angle_k_modify > 0)
        include_angle_k_modify=-include_angle_k_modify;//计算角度是锐角时，计算其补角（xielv互为相反数）
       deg_a_modify =  180+Rad2Deg( atan(include_angle_k_modify));

    }
    else{
      ROS_INFO("searchVType.line[%d] vertical to line[%d]!",i,i+1);
    }
    ROS_INFO("searchVType.cur cal line[%d] to line[%d] include_angle_deg:%.2f",i,i+1,deg_a_modify);
    if (
        //fabs(m_line[i].line_length - m_line_rechage) < MIN_LENGTH_ERROR &&   //chq disabled temp
        //fabs(m_line[i + 1].line_length - m_line_rechage) < MIN_LENGTH_ERROR && //chq disabled temp
        //include_angle_k > tan_min_recharge && include_angle_k < tan_max_recharge
        include_angle_k_modify > tan_min_recharge && include_angle_k_modify < tan_max_recharge
        )//如果两条直线的夹角与ｖ板夹角不超过阈值
    {
      /// Todo: get the point
      /// 两线交点
     // double a1 = (m_line[i].lbegin_x - m_line[i].lend_x) / (m_line[i].lbegin_y - m_line[i].lend_y);
      //double b1 = m_line[i].lbegin_x - a1 * m_line[i].lbegin_y;
      //double a2 = (m_line[i+1].lbegin_x - m_line[i+1].lend_x) / (m_line[i+1].lbegin_y - m_line[i+1].lend_y);
      //double b2 = m_line[i + 1].lbegin_x - a2 * m_line[i + 1].lbegin_y;
      //robot2vy = (b1 - b2) / (a2 - a1);
      //robot2vx = a1 * robot2vy + b1;
      Line a(m_line[i].line_k,-1,m_line[i].line_b);
      Line b(m_line[i+1].line_k,-1,m_line[i+1].line_b);
      VecPosition cross = a.getIntersection(b);
      robot2vx= cross.getX();
      robot2vy= cross.getY();
      double l1tocar_rad = fabs(atan(m_line[i].line_k));// limit to 0~pi/2
      double half_angular_bisector = fabs((atan(m_line[i].line_k) - atan(m_line[i + 1].line_k))/2.0);// limit to 0~pi/2
      double turn_w = l1tocar_rad - half_angular_bisector;
      turn_w *= (robot2vy>0?1:-1);//车在左侧　y>0 ,w取正　左转　否则　取负
      //////////////////////////////////////////////////////////////////////////
      //if (pre_robot2v.x != init_target_dist)//disable by chq temp
      /*if ( fabs(pre_robot2v.x - init_target_dist) > 0.0001 )
      {
        //如果新计算得到的目标与上一次目标相差过大，则以上一次记录目标为目标
        if (fabs(robot2vx - pre_robot2v.x) > 0.07){
          robot2vx = pre_robot2v.x;
          ROS_INFO_STREAM("searchVType.prevr2v not init.cur r2v.x - pre r2v > 0.07 set r2v.y=pre r2v.y");
        }
        if (fabs(robot2vy - pre_robot2v.y) > 0.07){
          robot2vy = pre_robot2v.y;
          ROS_INFO_STREAM("searchVType.prevr2v not init.cur r2v.y - pre r2v > 0.07 set r2v.y=pre r2v.y");
        }
      }*/

      /// todo some 0 judge
      /// tanα=(k-k1)/(1+kk1)=(k2-k)/(1+kk2)
      double k1 = MAXK;
      double a11 = atan((m_line[i].line_k - k1)/(1 + m_line[i].line_k*k1));//求k1与垂线(y)的夹角

      a11 = M_PI / 2.0 - a11;
      double a11_deg_look = Rad2Deg(a11);
      //enable temp -s
      double jia_1 = atan(include_angle_k);
      if (jia_1 < 0)  jia_1 = M_PI + jia_1;
      //enable temp -e
      double jia = atan(m_line[i].line_k) - atan(m_line[i + 1].line_k);//atana +/- atan b = atan( a +/- b /(1 -/+ ab) )
      double jia_test = atan((m_line[i].line_k-m_line[i+1].line_k)/(1+m_line[i].line_k*m_line[i+1].line_k));
      double jia_deg_look  = Rad2Deg(jia);

      double turn_angle_rad = fabs(a11+M_PI/2) - fabs(jia / 2.0);
      //HX - M_PI/2 车子与角平分线的夹角 已经证明 chq
      HX = 1.5 * M_PI - jia / 2.0 - a11;

      double turn_angle_this = HX - M_PI/2;
      double HX_deg_look = Rad2Deg(HX);
      ROS_INFO_STREAM("Docking::searchVType."
                      << "Found VType.The angle bet two lines(deg):" << deg_a_modify
                      << " cross pos : (" << robot2vx << "," << robot2vy << ")."
                      << " turn angle(deg): " <<  VecPosition::normalizeAngle( RAD2DEG(a11) )
                      );

      m_target.x = robot2vx * sin(HX) - robot2vy * cos(HX);
      m_target.y = robot2vx * cos(HX) + robot2vy * sin(HX);
      ROS_INFO_STREAM("Docking::searchVType.The VType target in motion control cord: (" << m_target.x << "," << m_target.y << ")");
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
        setTaskStatus(m_reme_status);//reset task
        ROS_INFO_STREAM("Find tye v_dock again, reset task_status to " << m_reme_status);
      }
    }
    else
    {
      double forecast_t[2];
      forecast_t[0] = pre_targetx - m_forecast_target.x;
      forecast_t[1] = pre_targety + m_forecast_target.y;

      ROS_INFO_STREAM("Docking::searchVType.cur inculde_angle_k:" << include_angle_k_modify
                      << " do not fit angle condition "
                      <<"do tar equal to pre_targetx - m_forecast_target and HX = pre HX. ");

      m_target.x = forecast_t[0];
      m_target.y = forecast_t[1];
      HX = preHX;
    }
    ROS_INFO_STREAM("Docking::searchVType End. target(x,y):(" << m_target.x << "," << m_target.y << ")" );
  }
  items_marker_pub.publish(items_maker);
}

/// handle task order
void Docking::setTask(int motion_task)
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
  ROS_INFO("Docking::setTask.Set TaskChargeBegin");
}

/// set task status
int Docking::getStatus()
{
  return m_task_status;
}

/// handle output speed
geometry_msgs::Twist Docking::getSpeed()
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
Point Docking::getTarget()
{
  //if (m_task_flag == 0)
  //{
  //  m_target.x = init_target_dist;
  //  m_target.y = 0.0;
  //}
  //if (fabs(m_target.x) > 2.0)  m_target.x = 2.0;
  //if (fabs(m_target.y) > 1.0) m_target.y = m_target.y / fabs(m_target.y);

  return pre_robot2v;
}


/// record odm
OrientedPoint Docking::recordOdom(bool switch_odm_record)
{
  if (switch_odm_record)
  {
    boost::mutex::scoped_lock l(mut);//we use the global var m_cur_odom,if will chage in callback fun :update odom ,so we must add a lock
    //ROS_INFO("Docking::recordOdom.m_cur_odom(x,y,angle):(%.6f,%.6f,%.6f)",m_cur_odom.x,m_cur_odom.y,m_cur_odom.theta );
    //ROS_INFO("Docking::recordOdom.m_pre_odom(x,y,angle):(%.6f,%.6f,%.6f)",m_pre_odom.x,m_pre_odom.y,m_pre_odom.theta );
    m_delta_odm = absoluteDifference(m_cur_odom, m_pre_odom);
    //ROS_INFO("Docking::recordOdom.m_delta_odm(in pre cord)(x,y,angle):(%.6f,%.6f,%.6f)",m_delta_odm.x,m_delta_odm.y,m_delta_odm.theta );
    m_leave_odm = absoluteSum(m_leave_odm, m_delta_odm);
    //ROS_INFO("Docking::recordOdom.m_leave_odm(in last leave cord)(x,y,angle):(%.6f,%.6f,%.6f)",m_leave_odm.x,m_leave_odm.y,m_leave_odm.theta );
    m_pre_odom = m_cur_odom;
    return m_leave_odm;
  }
  else
    return OrientedPoint(0,0,0);
}

/// set speed
void Docking::setSpeed(double _v, double _w)
{
  m_speedv = _v;  m_speedw = _w;
}
/// set task status
void Docking::setTaskStatus(int s)
{
  m_task_status = s;
}
void Docking::autoChargeThread(void){
  while (n.ok())
      {
        boost::mutex::scoped_lock l(mut_loop);
        ros::Duration(0.01).sleep();
        if (charge_status)
        {
          ROS_INFO("autoChargeThread.-----------------------------------start");
          //if in charge in/out process chq
          calPos();
          obstacle_points_ = obs_p;//与订阅更新数据隔离
          trackTarget(obstacle_points_, getTarget());
          searchVType(obstacle_points_);
          motionPlanning();
          /// pub speed
          vel_pub_.publish(getSpeed());
          /// update task status
          int status_flag = getStatus();
          if (status_flag == ChargeInSuc)
          {
            ROS_INFO("Docking::autoChargeThread.Finish update task status, In success~ go sleeping...");
            Set_Switch(false);
          }
          if (status_flag == ChargeOutSuc)
          {
            /// pub common data
            ROS_INFO("Docking::autoChargeThread.Finish update task status, Out success~ go sleeping...");
            Set_Switch(false);
          }
          if (status_flag == AtField1)
          {
            ROS_INFO("Docking::autoChargeThread.Finish update task status, field~ go sleeping...");
            Set_Switch(false);
          }
          if (status_flag == TypeVNotFound)
          {
           ROS_INFO("Docking::autoChargeThread.Can Not Fine Type V Dock");
          }
          if (status_flag == AtField2)
          {
            ROS_INFO("Docking::autoChargeThread.Finish update task status, field~ go sleeping...");
            Set_Switch(false);
          }
          ROS_INFO("autoChargeThread.-------------------------------------end");
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
    //  if (m_task_status != 110)
    //    m_reme_status = m_task_status;
    //  setTaskStatus(110);
    //  LOG_COUT_INFO(m_vdock_logger, "Can not find type v_dock set task_status 110, and remember now task_status " << m_reme_status);
    //}
    // chq 进入充电位 任务
    if (m_task_flag == ChargeInTask)
      {
        /// to do some filter
        //chq =2 新任务开始状态
        if (m_task_status == ChargeBegin)// chq when we get a new task, the m_task_status equal to 2
        {
          ROS_INFO("Docking::motionPlanning.charge in tar x:%f, y:%f, HX:%f,has_typev_flag:%d",m_target.x, m_target.y, HX / M_PI*180.0,has_typev_flag);
          //chq 如果发现充电桩
          if (has_typev_flag)
          {
            //chq 若发生y方向偏差过大现象，失败
            //if (fabs(m_target.y) > ydist_err)
            //{
              //m_task_flag = NoTask;
              //ROS_ERROR("Docking::motionPlanning.found VType.but m_target.y:%f, ERROR TOO LARGE.setTaskStatus(AtField2)！！！;", m_target.y);
             // setTaskStatus(AtField2);
            //}
            //chq 否则，固定线速度，角速度通过实时的偏差Hx与target.y计算
            //else
            {
              _speedv = v_fast;
              _speedw = mapToMinusPIToPI(HX - M_PI / 2.0) + m_target.y * 2.0;//mod by chq
              //  _speedw = 1.0 * m_target.y;
              //  _speedv = 0;
              //  _speedw = 0;
              ROS_INFO_STREAM("Docking::motionPlanning.caled v:" << _speedv << " w:" << _speedw);
              //chq 如果充电模式下，x方向偏差小于0.8m 进入修正调整角度模式
              if (m_target.x < xdist_near)
              {
                odm_record_flag = false;
                setTaskStatus(  ModifyAngleOnly);
                ROS_INFO("Docking::motionPlanning.found VType.m_target.x :%f < 0.8 Enter Modified HX Status.setTaskStatus(  ModifyAngleOnly);", m_target.x );
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
              ROS_INFO("Docking::motionPlanning.this time no found typev.last not record odom.now record!");
              m_pre_odom = m_cur_odom;
              m_leave_odm.x = 0.0;
              m_leave_odm.y = 0.0;
              m_leave_odm.theta = 0.0;
              odm_record_flag = true;
            }
            double record_dis = recordOdom(odm_record_flag).mod();
            //chq 如果累计调整超过0.2m 还未找到charge,stop
            if (record_dis > blind_move_thread)  /// if has no typevdock above 20cm  stop it
            {
              _speedv = 0.0;
              _speedw = 0.0;
              setTaskStatus(AtField1);
              ROS_ERROR("Docking::motionPlanning.this time no found typev.record_dis %.4f > blind_move_thread %.4f.Long Dis Can Not Find Type V Dock", record_dis,blind_move_thread);
            }
            //chq 否则，恒速前进，计算危险距离
            else
            {
              m_forecast_target.y = recordOdom(odm_record_flag).x * cos(HX) + recordOdom(odm_record_flag).y * sin(HX);
              m_forecast_target.x = recordOdom(odm_record_flag).x * sin(HX) - recordOdom(odm_record_flag).y * cos(HX);
              _speedv = v_slow;
              _speedw = 0.0;
             ROS_INFO("Docking::motionPlanning.no found typev.record_dis: %.4ff,below to blind_move_thread:%.4f, pub v:%.4f,w = 0", record_dis, blind_move_thread, _speedv);
            }
            //chq 如果x方向偏差<0.8，进入修正模式(by blind)
            if (m_target.x < xdist_near)
            {
              odm_record_flag = false;
              setTaskStatus(ModifyAngleOnly);
              ROS_INFO("Docking::motionPlanning.no found typev.m_target.x:%.4f below to xdist_near :%.4f.Enter Modified angle only BY Blind Moving. setTaskStatus(  ModifyAngleOnly);",m_target.x,xdist_near);
            }
          }
        }
        //chq x方向偏差已较小，进入角度修正模式
        if (m_task_status == ModifyAngleOnly) // modified
        {
          _speedv = 0.0;
          _speedw = mapToMinusPIToPI(HX - M_PI / 2.0);
          ROS_INFO("Docking::motionPlanning.modify angle only.target x:%f, y:%f, Hx(deg):%.4f.v:0,w(rad):%.4f,w(deg):%.4f", m_target.x, m_target.y, HX / M_PI*180.0,_speedw,Rad2Deg(_speedw));
          if (fabs(Rad2Deg(_speedw)) < angle_near)//若距离较小，且角度偏差已经很小，角速度设为0
          {
            ROS_INFO("Docking::motionPlanning.turn angle(deg):%.4f below to angle_near:%.4f,set w=0.setTaskStatus(GoStraightLineMode(GoStraightLineMode));",fabs(_speedw),angle_near);
            _speedw = 0.0;
            setTaskStatus(GoStraightLineMode);
          }
        }
        //chq x距离偏差已较小且角度调整到位,进入 直行到充电桩 模式
        if (m_task_status == GoStraightLineMode)  // last dis
        {
          //chq 如果y方向发生偏差较大情况，失败
          //chq disable temp
         /* if (!last_go_ready_flag && fabs(m_target.y) > yarrived_thread)
          {
            m_task_flag = NoTask;
            ROS_ERROR("Docking::motionPlanningfirst into GoStraightLineMode.but the Y :%f > yarrived_thread, IS TOO LARGE.set m_task_flag = NoTask; .setTaskStatus(AtField1);",fabs(m_target.y));
            setTaskStatus(AtField1);
          }
          */
          ROS_INFO("Docking::motionPlanning.judge suc lastly..set last_go_ready_flag = true;");
          last_go_ready_flag = true;
           //chq 重置并累计计算里程计距离值
          if (!odm_record_flag)
          {
             ROS_INFO("Docking::motionPlanning.go ahead mode.last not record odom.now record!");
            m_pre_odom = m_cur_odom;
            m_leave_odm.x = 0.0;
            m_leave_odm.y = 0.0;
            m_leave_odm.theta = 0.0;
            odm_record_flag = true;
          }
          double record_dis = recordOdom(odm_record_flag).mod();

          //chq 如果小于设定的限制行驶距离，固定速度前进
          if (record_dis < m_lastdis)
          {
            _speedv = v_slow;
            _speedw = 0.0;
             ROS_INFO("Docking::motionPlanning.go ahead mode.Record odm dist:%f < m_lastdis:%.4f.set _speedv = v_slow. ", record_dis,m_lastdis);
          }
          //chq 否则，停止并置任务为完成状态
          else
          {
            m_task_flag = NoTask;
            odm_record_flag = false;
            _speedv = 0.0;
            _speedw = 0.0;
            setTaskStatus(ChargeInSuc);  // in finish
            ROS_INFO("Docking::motionPlanning.go ahead mode.Record odm dist:%f > m_lastdis:%.4f.set _speedv = v_slow.Charge in suc! ", record_dis,m_lastdis);
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
      if (recordOdom(odm_record_flag).mod() < m_outdis)
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
        setTaskStatus(ChargeOutSuc);  // out finish
        ROS_INFO("Docking::Run_Navigation.Charge Out Success!");
      }
    }
  }

//  if (judgeStop())
//  {
//    setSpeed(0.0, 0.0);
//    LOG_COUT_INFO(m_vdock_logger, "Laser Stop");
//  }
//  else
//  {
//    setSpeed(_speedv, _speedw);
//  }
    //chq 下发速度给控制器
  setSpeed(_speedv, _speedw);
  ros::Duration(0.03).sleep();
}

