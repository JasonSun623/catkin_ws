#include <ros/ros.h>
#include <string>
#include <sstream>
#include "filter_rfs_center.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "CountTime.hpp"
int filter_num = 1;

using namespace filter_rfs_center_space;
class Rfs_Filter{

public :
  Rfs_Filter();
  void getPubPos(const std::vector<VecPositionDir>& v_optrfs_center,
              geometry_msgs::PoseArray rfs_pos,
              visualization_msgs::MarkerArray& items_MarkerArray);
  void compensateScanMoveDist(std::vector<VecPositionDir>& mea_rfs,double loop_time);
  void callBackScan(const sensor_msgs::LaserScan & scan);
  void callbackOdom(const nav_msgs::Odometry::ConstPtr& state_odata);
 private:
  string scan_topic;
  std::string odom_topic ;
  std::string scan_frame ;
  std::string pub_rfs_topic;
  visualization_msgs::Marker items_maker;
  visualization_msgs::Marker text_marker;
  visualization_msgs::MarkerArray items_MarkerArray;
  bool if_pub_marker;
  int scan_update_fre;
  double m_cur_vec_x,m_cur_vec_y,m_cur_w;

  boost::mutex scan_lock_;
  boost::mutex odom_mut;

  ros::Publisher pub_ref_center;
  ros::Publisher items_array_pub;

  int _fiter_scan_num;
  int pub_rate;
  geometry_msgs::PoseArray rfs_pos;
  std::vector<sensor_msgs::LaserScan> raw_scan;
  std::vector<VecPositionDir> v_optrfs_center;

  bool _judge_by_dist;//直接通过距离判断反光板
  int _echo;
  double _rf_radius;
  double _step;
  double _err;

  FilterRfsCenter filter;

  ros::Subscriber scan_sub;
  ros::Subscriber odom_sub ;
  ros::NodeHandle nh;
};
Rfs_Filter::Rfs_Filter(){

  nh.param<std::string>("odom_topic", odom_topic, "/wheel_diff_controller/odom");
  nh.param<std::string>("scan_topic", scan_topic, "filter_scan");
  nh.param<std::string>("scan_frame", scan_frame, "hokuyo_link");
  nh.param<std::string>("pub_rfs_topic", pub_rfs_topic, "pf_reflectors");

  nh.param<int>("pub_rate",pub_rate,20);
  nh.param<int>("scan_update_fre",scan_update_fre,30);
  nh.param<int>("fiter_scan_num", _fiter_scan_num, 1);
  nh.param<bool>("if_pub_marker",if_pub_marker,true);
  nh.param<bool>("judge_by_dist", _judge_by_dist, true);//直接通过距离判断反光板
  nh.param<int>("echo_thread", _echo, 100);
  nh.param<double>("reflector_radius",_rf_radius,0.025);
  nh.param<double>("step",_step,0.0005);
  nh.param<double>("err_thread",_err,0.0011);

  //construct the filter scan object
  filter(FilterRfsCenter(_judge_by_dist,_rf_radius,_echo,_step,_err));

  odom_sub = nh.subscribe(odom_topic,1,&Rfs_Filter::callbackOdom,this);
  scan_sub = nh.subscribe(scan_topic,1,&Rfs_Filter::callBackScan,this);
  pub_ref_center = nh.advertise<geometry_msgs::PoseArray>(pub_rfs_topic,1,true);
  //pub the visualization of the rfs center data
  items_array_pub = nh.advertise<visualization_msgs::MarkerArray>("pub_rfs_marker_array",1,true);



}
  void Rfs_Filter::getPubPos(const std::vector<VecPositionDir>& v_optrfs_center,
              geometry_msgs::PoseArray rfs_pos,
              visualization_msgs::MarkerArray& items_MarkerArray){
    int sum = 0;
    rfs_pos.poses.clear();
    items_MarkerArray.markers.clear();
    items_maker.points.clear();

    items_maker.header.stamp = ros::Time::now();
    items_maker.header.frame_id = scan_frame;
    rfs_pos.header.frame_id = scan_frame;
    items_maker.type = visualization_msgs::Marker::POINTS;
    items_maker.ns = "rfs_nmspace";
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
    items_maker.color.a = 0.3;
    items_maker.lifetime = ros::Duration(1000);

    if(v_optrfs_center.size() ){
      for(int i=0;i < v_optrfs_center.size();i++){
        sum+=1;
        items_maker.id = sum;
        rfs_pos.header.stamp = items_maker.header.stamp;
        geometry_msgs::Point p;
        //在相对于激光头的坐标空间
        p.x = v_optrfs_center[i].getX();
        p.y = v_optrfs_center[i].getY();
        p.z = 0;
        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0;
        ////fort text type,Only scale.z is used. scale.z specifies the height of an uppercase "A".
        text_marker = items_maker;
        text_marker.ns = "rfs_text_space";
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.color.r = 255;
        text_marker.color.g = 0;
        text_marker.color.b = 0;
        text_marker.color.a = 1;
        text_marker.scale.z = 0.1;
        text_marker.id = sum;
        stringstream ss ;
        ss << sum;
        string str;
        ss >> str;
        text_marker.text = str ;
        text_marker.pose = pose;
        items_MarkerArray.markers.push_back(text_marker);
        items_maker.points.push_back(p);
        rfs_pos.poses.push_back(pose);
      }
    }
    items_MarkerArray.markers.push_back(items_maker);

    if(if_pub_marker)items_array_pub.publish(items_MarkerArray);

    pub_ref_center.publish(rfs_pos);

  }
  void Rfs_Filter::compensateScanMoveDist(std::vector<VecPositionDir>& mea_rfs,double loop_time){

    static double every_angle_waster_t = (1.0/(double)scan_update_fre)/(2*M_PI);
    static double min_scan_period = (1.0/(double)scan_update_fre);
    if(loop_time < min_scan_period)return ;//below to scan period we no not compensate

    std::vector<VecPositionDir> temp_rfs;
    for(int i =0 ;i < mea_rfs.size();i++){
      double cur_angle = Rad2Deg(mea_rfs[i].getDirection());
      if( cur_angle < 0 ) cur_angle+=2*M_PI;
      double dt = (loop_time - min_scan_period)+(2*M_PI-cur_angle)*every_angle_waster_t;

      double theta =0;
      double delta_theta = m_cur_w * dt;//因为里程计更新速度足够快，我们认为里程计是准的,忽略了延迟
      double cur_theta = delta_theta+theta ;
      double delta_x,delta_y;

      if ( fabs( m_cur_w ) < EPSILON )
      {// fit the condition that there is no rotate speed
        delta_x = (m_cur_vec_x * cos(theta) - m_cur_vec_y * sin(theta) ) * dt;
        delta_y = (m_cur_vec_x * sin(theta) + m_cur_vec_y * cos(theta) ) * dt;
      }
      else
      {
        double r_x = m_cur_vec_x/m_cur_w;
        double r_y = m_cur_vec_y/m_cur_w;

        delta_x = (  r_x * ( sin( cur_theta ) - sin( theta ) ) + r_y * ( cos( cur_theta ) - cos( theta ) ) ) ;
        delta_y = ( -r_x * ( cos( cur_theta ) - cos( theta ) ) + r_y * ( sin( cur_theta ) - sin( theta ) ) ) ;//注意，y方向上，前后偏移差值再取负，因为投影方向是向上为正
      }
      VecPosition scan_pre=mea_rfs[i];
      VecPosition new_origin(delta_x,delta_y);
      VecPosition scan_new = scan_pre.globalToRelative(new_origin,Rad2Deg( cur_theta) );
      temp_rfs.push_back(VecPositionDir(scan_new,mea_rfs[i].angle()));
    }
    mea_rfs = temp_rfs;
  }
  //选取filter_num个连续激光束　用于平均
  void  Rfs_Filter::callBackScan(const sensor_msgs::LaserScan & scan){
    boost::mutex::scoped_lock l(scan_lock_);
    static CountTime timer;
    static int push_num = 0;

    //timer.end();

    //double dt = timer.getTime()/1000.0;


    if( push_num < _fiter_scan_num ){
      raw_scan.push_back(scan);
      push_num++;
    }
    else{
      raw_scan.pop_back();
      raw_scan.push_back(scan);
    }

    //cal the cneter of rfs
    timer.begin();
    filter.getReflectorsCenter(raw_scan,v_optrfs_center);
    timer.end();
    double dt = timer.getTime()/1000.0;

   // compensateScanMoveDist(v_optrfs_center,dt);
    //getPubPos(v_optrfs_center,rfs_pos,items_MarkerArray);
    ROS_INFO("pf_filter_center.rate:%.3f(ms).rfs num:%d",dt*1000,v_optrfs_center.size());
    //ROS_INFO_STREAM("pf_filter.push scan size:" << raw_scan.size() );
    //timer.begin();
  }

  void  Rfs_Filter::callbackOdom(const nav_msgs::Odometry::ConstPtr& state_odata){
    boost::mutex::scoped_lock l(odom_mut);
    const nav_msgs::Odometry odo = (*state_odata);
    m_cur_vec_x = odo.twist.twist.linear.x;
    m_cur_vec_y = odo.twist.twist.linear.y;
    m_cur_w = odo.twist.twist.angular.z;
    ROS_INFO("filterRfsCenter.call odom.vx:%.6f,rot(rad):%.6f",m_cur_vec_x,m_cur_w);
  }









int main(int argc, char **argv)
{
   ros::init(argc, argv, "pepper_fuchs_filter_center");
   try
    {
      Rfs_Filter rfs;
      ros::spin();
    }
    catch(std::runtime_error& e)
    {
      ROS_ERROR("pepper_fuchs_filter_center exception: %s", e.what());
      return -1;
    }
    return 0;

}


