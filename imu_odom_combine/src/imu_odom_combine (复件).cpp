#include <imu_odom_combine/imu_odom_combine.h>
#include <imu_odom_combine/CountTime.hpp>
namespace imu_odom_fusion_space {
sensorFusion::sensorFusion(){
  nh.param<std::string>("odom_comb_frame",odom_comb_frame,"/odom");
  nh.param<std::string>("base_link_frame",base_link_frame,"/base_link");
  nh.param<std::string>("odom_combine_topic",odom_combine_topic,"odom_comb");
  nh.param<std::string>("imu_topic",imu_topic,"/imu_data");
  nh.param<std::string>("odom_topic",odom_topic,"/wheel_diff_controller/odom");
  nh.param<bool>("publish_tf",publish_tf,true);
  rec_imu_msg = false;
  rec_odom_msg = true;
  sub_imu = nh.subscribe(imu_topic,1,&sensorFusion::callBackImu,this);
  sub_odom = nh.subscribe(odom_topic,1,&sensorFusion::callBackOdom,this);
  time_combine = nh.createTimer(ros::Duration(0.02), boost::bind(&sensorFusion::sensorFusing,this));
  pub_combine_odom = nh.advertise<nav_msgs::Odometry>(odom_combine_topic,1000);

  odom_trans.header.frame_id = odom_comb_frame;
  odom_trans.child_frame_id = base_link_frame;

}
sensorFusion::~sensorFusion(){

}

void sensorFusion::callBackOdom(const nav_msgs::Odometry::ConstPtr& odom){
  boost::mutex::scoped_lock l(odom_mut);
  if(!rec_odom_msg)rec_odom_msg = true;
  odom_msg = (*odom);
}
void sensorFusion::callBackImu(const sensor_msgs::Imu::ConstPtr& imu){
  boost::mutex::scoped_lock l(imu_mut);
  if(!rec_imu_msg)rec_imu_msg = true;
  imu_msg = (*imu);
}
void sensorFusion::sensorFusing(){
 boost::mutex::scoped_lock l(mutex_);
 if(!rec_imu_msg || !rec_odom_msg){
   ROS_ERROR("we must start comb after odom or imu data.return");
   return;//we must start comb after odom or imu data
 }

 static bool first_time = true;
 static sensor_msgs::Imu last_imu_msg;

 if(first_time){
   last_imu_msg = imu_msg;
   g_x = odom_msg.pose.pose.position.x;
   g_y = odom_msg.pose.pose.position.y;
   geometry_msgs::Quaternion cur_qua =  odom_msg.pose.pose.orientation;
   if( fabs(cur_qua.w) <1e-4 && fabs(cur_qua.x) <1e-4 && fabs(cur_qua.y) <1e-4&& fabs(cur_qua.z) <1e-4 )
     g_angle = 0;
   else{
     if(std::isnan(tf::getYaw(cur_qua)))
       g_angle = 0;
     else
       g_angle = tf::getYaw(cur_qua);
   }
   first_time = false;
 }

 geometry_msgs::Quaternion cur_imu_qua = imu_msg.orientation;
 geometry_msgs::Quaternion last_imu_qua = last_imu_msg.orientation;
 double cur_imu_heading = tf::getYaw(cur_imu_qua);
 double last_imu_heading = tf::getYaw(last_imu_qua);
 double delta_angle = cur_imu_heading - last_imu_heading;
 static CountTime t;
 t.end();
 double dt = t.getTime()/1000;
 t.begin();
 if( dt < 1e-4 )return;
 double w = delta_angle/dt;
 double delta_x = 0.0,delta_y =0.0;
 double v = odom_msg.twist.twist.linear.x;

 if( fabs(w) < 1e-4 ){
   delta_x =  v * dt * cos(g_angle);
   delta_y =  v * dt * sin(g_angle);
   g_x += delta_x;
   g_y += delta_y;
 }
 else{
   double R_rotate = v / w;
   delta_x = R_rotate * ( sin(g_angle  + delta_angle ) - sin( g_angle ));
   delta_y = R_rotate * ( -cos(g_angle + delta_angle ) + cos( g_angle ));
   g_x += delta_x;
   g_y += delta_y;
 }
 g_angle += delta_angle;
 g_angle = mapToMinusPIToPI(g_angle);
 last_imu_msg = imu_msg;
 publishOdom();
 publishOdomTF();
}

void sensorFusion::publishOdom(){
  static boost::shared_ptr<nav_msgs::Odometry> odom_new_msg(new nav_msgs::Odometry());
  odom_new_msg->header.frame_id = odom_comb_frame;
  odom_new_msg->header.stamp = ros::Time::now();
  odom_new_msg->child_frame_id = base_link_frame;
  // Position
  odom_new_msg->pose.pose.position.x =g_x;
  odom_new_msg->pose.pose.position.y =g_y;

  odom_new_msg->pose.pose.position.z =odom_msg.pose.pose.position.z;

  geometry_msgs::Quaternion cur_qua = tf::createQuaternionMsgFromYaw(g_angle);
  // Orientation
  odom_new_msg->pose.pose.orientation.x = cur_qua.x;
  odom_new_msg->pose.pose.orientation.y = cur_qua.y;
  odom_new_msg->pose.pose.orientation.z = cur_qua.z;
  odom_new_msg->pose.pose.orientation.w = cur_qua.w;
  // Pose covariance
  odom_new_msg->pose.covariance[0]  = 0.00001;//x*x r2d2_conf_ekf
  odom_new_msg->pose.covariance[7]  = 0.00001;//y*y mir r2d2_conf_ekf
  //odom_new_msg->pose.covariance[35] = pow(0.01 * M_PI / 180,2);//yaw*yaw jy901
  odom_new_msg->pose.covariance[35] = pow(0.00017,2);//imu gazebo

  odom_new_msg->pose.covariance[14] = DBL_MAX; // set a non-zero covariance on unused//z*z
  odom_new_msg->pose.covariance[21] = DBL_MAX; // dimensions (z, pitch and roll); this//roll*roll
  odom_new_msg->pose.covariance[28] = DBL_MAX; // is a requirement of robot_pose_ekf//pitch*pitch
  // Velocity
  odom_new_msg->twist.twist.linear.x = odom_msg.twist.twist.linear.x;
  odom_new_msg->twist.twist.linear.y = odom_msg.twist.twist.linear.y;

  odom_new_msg->twist.twist.angular.z = imu_msg.angular_velocity.z;
  pub_combine_odom.publish(odom_new_msg);
}

void sensorFusion::publishOdomTF(){
  if (publish_tf == false)
    return;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = g_x;
  odom_trans.transform.translation.y = g_y;
  odom_trans.transform.translation.z = odom_msg.pose.pose.position.z;
  geometry_msgs::Quaternion cur_qua = tf::createQuaternionMsgFromYaw(g_angle);
  odom_trans.transform.rotation = cur_qua;
  tb.sendTransform(odom_trans);
}

}


