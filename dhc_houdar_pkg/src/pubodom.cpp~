#include <ros/ros.h>
#include <ros/duration.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread/thread.hpp>

ros::Subscriber sub_od;
ros::Publisher pub_od;

 nav_msgs::Odometry odom;
  static geometry_msgs::TransformStamped stamped;
  static geometry_msgs::Quaternion qu;
  static double global_x,global_y,global_z,global_theta;
 double up_z = 0.5;//车体相对于base_link高度
 double up_laser = 0.2; //激光相对于车体高度
void RecGazeboOdom(const nav_msgs::Odometry& od){ 
  //如果使用gazebo差速控制器，发布的odom消息与robot没有坐标关系，这里将他们关系链接起来
  static int i = 0;
  tf::TransformBroadcaster tf_broad;
 if( i++ > 100 ){
     ROS_INFO("pubodom.into RecGazeboOdom...."); 
     ROS_INFO_STREAM("pubodom.RecGazeboOdom.rec frame id: " << od.header.frame_id << " child frame:" <<od.child_frame_id);
     i = 0;
}
  stamped.header.frame_id = "real_odom";
  stamped.child_frame_id = "base_link";
  ros::Time cur_time = ros::Time::now();
  stamped.header.stamp = cur_time;
  global_x = od.pose.pose.position.x;
  global_y = od.pose.pose.position.y;
  global_z = od.pose.pose.position.z + up_laser - up_z ;
 // global_theta += tf::getYaw(od.pose.pose.orientation);

  stamped.transform.translation.x = global_x;
  stamped.transform.translation.y = global_y;
  stamped.transform.translation.z = global_z;//与urdf/R2D2.simple.xacro模型文件对应
 // qu = tf::createQuaternionMsgFromYaw(global_theta);
  qu =  od.pose.pose.orientation;
  stamped.transform.rotation = qu;
  //tf_broad.sendTransform(stamped);
  
  odom.header.frame_id = stamped.header.frame_id;
  odom.child_frame_id = stamped.child_frame_id;

  odom.header.stamp = stamped.header.stamp;
  odom.pose.pose.position.x = stamped.transform.translation.x;
  odom.pose.pose.position.y = stamped.transform.translation.y;
  odom.pose.pose.position.z = stamped.transform.translation.z;
  odom.pose.pose.orientation = stamped.transform.rotation;
  odom.pose.covariance = od.pose.covariance;
  
  odom.twist.twist.linear.x = od.twist.twist.linear.x;
  odom.twist.twist.linear.y = od.twist.twist.linear.y;
  odom.twist.twist.linear.z = od.twist.twist.linear.z;

  odom.twist.twist.angular.x = od.twist.twist.angular.x;
  odom.twist.twist.angular.y = od.twist.twist.angular.y;
  odom.twist.twist.angular.z = od.twist.twist.angular.z;
  odom.twist.covariance = od.twist.covariance;
 // pub_od.publish(odom);

  tf_broad.sendTransform(stamped);
  pub_od.publish(odom);
}
void pub_tf_odommsg_thread(){
  ros::Rate r(100);
 tf::TransformBroadcaster tf_broad;
//注意，tf之类的变量最好声明成局部变量，不然会报错，
//因为其多使用了NodeHandle类型变量参数，但是Nodehandle需要在ros::init后面
  while(1){
  tf_broad.sendTransform(stamped);
  pub_od.publish(odom);
  r.sleep();
  }
}

int main (int argc, char ** argv)
{
 
  ros::init(argc, argv, "pubodom_node");
  ros::NodeHandle nh;
  ROS_INFO("pubodom init done...");                       
  sub_od = nh.subscribe("/wheel_diff_controller/odom",1,RecGazeboOdom);
  pub_od = nh.advertise<nav_msgs::Odometry>("real_odom",1,true);
  //boost::thread th(pub_tf_odommsg_thread);
  ros::spin();
  

}
