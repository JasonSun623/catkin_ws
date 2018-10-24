
/*
 * sicknav350_node.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: Punith Mallesha
 *
 * Based on the sicklms.cpp and sickld.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 *
 * Released under BSD license.
 */

#include <iostream>
#include <sicktoolbox/SickNAV350.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <deque>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define DEG2RAD M_PI/180.0
#define RAD2DEG 180/M_PI
using namespace std;
using namespace SickToolbox;
// odometry call back from the robot
double vx,vy,vth;

void publish_scan(
        ros::Publisher *pub,
        double *range_values,
        uint32_t n_range_values,
         int *intensity_values,
        uint32_t n_intensity_values,
        ros::Time start,
        double scan_time,
        bool inverted,
        float angle_min,
       float angle_max,
        std::string frame_id,
        unsigned int sector_start_timestamp)
{
  static int scan_count = 0;
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = frame_id;
  scan_count++;
  if (inverted) {
    scan_msg.angle_min = angle_max*DEG2RAD;
    scan_msg.angle_max = angle_min*DEG2RAD;
  } else {
    scan_msg.angle_min = angle_min*DEG2RAD;
    scan_msg.angle_max = angle_max*DEG2RAD;
  }
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(n_range_values-1);
  scan_msg.scan_time = 0.125;//scan_time 125ms;
  scan_msg.time_increment = scan_msg.scan_time / n_range_values;
  scan_msg.range_min = 0.1;
  scan_msg.range_max = 250.;
  scan_msg.ranges.resize(n_range_values);
  scan_msg.intensities.resize(n_range_values);
  scan_msg.header.stamp = start;

   //output scan data to screen
 // std::cout << "scan data intensity(ang,intensity):\n ";
  for (size_t i = 0; i < n_range_values; i++) {
    scan_msg.ranges[i] = (float)range_values[i]/1000;
    scan_msg.intensities[i] = intensity_values[i];//chq
    double a = RAD2DEG *(scan_msg.angle_min + i * scan_msg.angle_increment);
    //printf(" (%.1f,%.0lf) ",a,  scan_msg.intensities[i] );
   //if( i%4==0)std::cout << "\n ";
  }
  std::cout << "\n ";

  pub->publish(scan_msg);

}

//发布激光头到把base 转换
void PublishLaserTransform(tf::TransformBroadcaster laser_broadcaster,std::string header_frame_id,std::string child_frame_id)
{

        laser_broadcaster.sendTransform(
                tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.2374)),
                      ros::Time::now(),header_frame_id, child_frame_id)); // distance from the focal point of the scanner to its base (199.4mm) + offset from the mount (38mm)

} //you can also define a customized urdf model using the nav350 meshes given


//necessary for sensor fusion using robot_localization package
void PublishLaserOdometry(double x,double y,double th,ros::Publisher *pub,std::string frame_id,std::string child_frame_id)
{
    //publish the odometry message over ROS
    ros::Time current_time;
    current_time=ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    //1\set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;
    //2\set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    pub->publish(odom);

}

// pos 到 绝对坐标系的TF变换
void PublishPositionTransform(double x,double y,double th,tf::TransformBroadcaster odom_broadcaster,std::string header_frame_id,std::string child_frame_id)
{

    ros::Time current_time;
    current_time=ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id =header_frame_id;// "odom_frame"
    odom_trans.child_frame_id = child_frame_id;// "base_link_frame"

    odom_trans.transform.translation.x = x;//global x coordinate
    odom_trans.transform.translation.y = y; //global y coordinate
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
}


void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    vx=msg->twist.twist.linear.x;
    vy=msg->twist.twist.linear.y;
    vth=msg->twist.twist.angular.z;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sicknav350");
    int port;
    std::string ipaddress;
    std::string frame_id, fixed_frame_id;
    std::string odometry;
    std::string scan;
    bool inverted;
    bool publish_tf_,publish_odom_,publish_scan_;
    int sick_motor_speed = 8;//10; // Hz
    double sick_step_angle = 0.25;//0.5;//0.25;
    double active_sector_start_angle = 0;
    double active_sector_stop_angle = 360;;
    std::string laser_frame_id,laser_child_frame_id,odom_frame_id;
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");
    nh_ns.param<std::string>("scan", scan, "scan");
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 100);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("nav350laser/odom", 10);

    nh_ns.param<bool>("publish_tf", publish_tf_, true);
    nh_ns.param<bool>("publish_odom_", publish_odom_, true);
    nh_ns.param<bool>("publish_scan", publish_scan_, true);

    nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
    nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
    nh_ns.param("inverted", inverted, false);
    nh_ns.param<std::string>("frame_id", frame_id, "front_laser_frame"); //laser frame
    nh_ns.param<std::string>("fixed_frame_id", fixed_frame_id, "base_link_frame"); //robot base frame

    nh_ns.param<std::string>("laser_frame_id", laser_frame_id, "odom_frame"); //global cooridnate frame measurement for navigation and position based on reflectors
    nh_ns.param<std::string>("laser_child_frame_id", laser_child_frame_id, "base_link_frame");// a fixed frame eg: odom or base or reflector

    ros::Subscriber sub = nh.subscribe("odometry/filtered", 10, OdometryCallback); // data from sensor fusion or wheel odometry of jackal robot

    nh_ns.param("resolution", sick_step_angle, 1.0);
    nh_ns.param("start_angle", active_sector_start_angle, 0.);
    nh_ns.param("stop_angle", active_sector_stop_angle, 360.);
    nh_ns.param("scan_rate", sick_motor_speed, 5);

    /* Define buffers for return values */
    double range_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
     int intensity_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};

     int cord_mode, rfcnt;
     double rfpar1[SICK_MAX_NUM_REFLECTORS] ={0};
    double rfpar2[SICK_MAX_NUM_REFLECTORS] ={0};

    /* Define buffers to hold sector specific data */
    unsigned int num_measurements = {0};
    unsigned int sector_start_timestamp = {0};
    unsigned int sector_stop_timestamp = {0};
    double sector_step_angle = {0};
    double sector_start_angle = {0};
    double sector_stop_angle = {0};
    /* Instantiate the object */
    SickNav350 sick_nav350(ipaddress.c_str(),port);
  //  ros::Duration(50).sleep(); //timedelay for jackal robot startup jobs
  double last_time_stamp=0;
  ///-------------------------------- Init the NAV350 parameter-------------------------------------------------////
    try {
        /* Initialize the device */
        sick_nav350.Initialize();

        try {
          //sick_nav350.SetOperatingMode(1);//0 POWERDOWN / 1 standby  2 ,MAPPING(if we want change the mode from power down to navigation or landmark detection ,we must change the mode to standby firstly!!)
            sick_nav350.SetOperatingMode(1);//1 standby
            sick_nav350.SetOperatingMode(3);//3 landmark detecting, 4 navigation

          //set output data format
          sick_nav350.SetPositioningDataFormat(1,1);//chq set data output format
          sick_nav350.SetLandMarkDataFormat(1,1,1);//chq set data output format 0 cartesian cord,1 enable show opt param ,1 Select transmission of the detected  data in positioning mode withNPOSGetData
          sick_nav350.SetScanDataFormat(1,1);//chq set scan data output format 1 dist only ,1 enable output remission data

          uint8_t mode,show,cord_frame,landmarkfilter;
          //check setting
          sick_nav350.GetPositioningDataFormat(mode,show);//chq get positioning data output format
          sick_nav350.GetLandMarkDataFormat(cord_frame,show,landmarkfilter);//chq get landmark data output format
          sick_nav350.GetScanDataFormat(mode,show);//chq get scan data output format
        } catch (...) {
            ROS_ERROR("Configuration error");
            return -1;
        }

    ros::Time last_start_scan_time;
    unsigned int last_sector_stop_timestamp = 0;
    ros::Rate loop_rate(8);
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster laser_broadcaster;
///---------------------------get newest data from 350-----------------------------------------------------------------////
        while (ros::ok()) {
            /* Get the scan and landmark measurements */
          sick_nav350.GetDataLandMark(1,1);//---
         // sick_nav350.GetDataPosition(1);//chq get position data (only)
         // sick_nav350.GetDataNavigation(1,2);// 0 instantly 1 next / 0 pos 1 pos+scan 2 p + S + rf
          sick_nav350.GetSickMeasurements(
                                        range_values,
                                         intensity_values,//chq
                                        &num_measurements,
                                        &sector_step_angle,
                                        &sector_start_angle,
                                        &sector_stop_angle,
                                        &sector_start_timestamp,
                                        &sector_stop_timestamp
                                        );
         /* sick_nav350.GetLandmarkMeasurements(
                                        rfcnt,
                                        cord_mode,//chq
                                       rfpar1,
                                       rfpar2
                                        );
         */

#if 0
    double x1=(double) sick_nav350.PoseData_.x;
    double y1=(double) sick_nav350.PoseData_.y;
    double phi1=(double )sick_nav350.PoseData_.phi;

    double x2 = x1/1000;
    double y2 = y1/1000;

    double phi2 = phi1 /1000 ;
    if(phi2>=180)phi2 -= 360;
    phi2*=DEG2RAD;
    //double phi2=phi1-180000-1250-300;
    //phi2=phi2/1000*3.14159/180;
   // x2=x1-/*300*/529*cos(phi2);
    //y2=y1-/*300*/529*sin(phi2);
   // x2=x2/1000;
   // y2=y2/1000;
#endif
    if(publish_tf_)
    {
        PublishPositionTransform(x2,y2,phi2,odom_broadcaster,laser_frame_id,laser_child_frame_id); //publish position data as transform in map frame
        //PublishLaserTransform(laser_broadcaster,fixed_frame_id,frame_id); // publish laser transform with respect to base frame
    }
    if(publish_odom_)
      {
        PublishLaserOdometry(x2,y2,phi2,&odom_pub,laser_frame_id,laser_child_frame_id); // publish odometry data from nav350 for sensor fusion
      }

    if ( sector_start_timestamp < last_time_stamp )
    {
        loop_rate.sleep();
        ros::spinOnce();
        continue;
    }
        last_time_stamp=sector_start_timestamp;
        ros::Time end_scan_time = ros::Time::now();

        double scan_duration = 0.125;

        ros::Time start_scan_time = end_scan_time - ros::Duration(scan_duration);
       // sector_start_angle-=180;
        //sector_stop_angle-=180;
       // sector_start_angle = 0;
        //sector_stop_angle = 360;
        if(publish_scan_)
       {
            //ROS_INFO("nav350.chq.main.needed to pub scan...");
            publish_scan(
                        &scan_pub,
                        range_values,
                        num_measurements,
                        intensity_values,
                        num_measurements,
                        start_scan_time,
                        scan_duration,
                        inverted,
                        (float)sector_start_angle,
                        (float)sector_stop_angle,
                        frame_id,
                        sector_start_timestamp);
        }

         last_start_scan_time = start_scan_time;
         last_sector_stop_timestamp = sector_stop_timestamp;

          // sick_nav350.SetSpeed(vx,vy,vth,sector_start_timestamp,0);

            loop_rate.sleep();
            ros::spinOnce();

        }
        /* Uninitialize the device */
       sick_nav350.Uninitialize();
    }
    catch(...) {
        ROS_ERROR("Error");
        return -1;
    }
    return 0;
}
