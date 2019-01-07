#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace imu_odom_fusion_space {

class sensorFusion{
public:
  sensorFusion();
  ~sensorFusion();

  void callBackOdom(const nav_msgs::Odometry::ConstPtr& odom);
  void callBackImu(const sensor_msgs::Imu::ConstPtr& imu);
  void sensorFusing();
  void publishOdom();
  void publishOdomTF();
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

private:
  geometry_msgs::PoseWithCovarianceStamped pose;
  nav_msgs::Odometry odom_msg;
  sensor_msgs::Imu imu_msg;
  //tf
  geometry_msgs::TransformStamped odom_trans;
  tf::TransformBroadcaster tb;
  double g_x,g_y,g_angle;//odom pos info
  boost::mutex odom_mut,imu_mut,mutex_;
  std::string odom_comb_frame,base_link_frame;
  std::string imu_topic,odom_topic,odom_combine_topic;

  ros::Subscriber sub_odom;
  ros::Subscriber sub_imu;
  ros::NodeHandle nh;
  bool publish_tf;
  bool rec_odom_msg,rec_imu_msg;
  ros::Timer time_combine;
  ros::Publisher pub_combine_odom;


};

}
