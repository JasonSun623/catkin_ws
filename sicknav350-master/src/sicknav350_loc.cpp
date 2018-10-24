
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
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "sicknav350/Geometry.h"

#define DEG2RAD M_PI/180.0
#define RAD2DEG 180/M_PI
using namespace std;
using namespace SickToolbox;
// odometry call back from the robot
double vx,vy,vth;
tf::TransformListener* global_listener;
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
  //std::cout << "\n ";

  pub->publish(scan_msg);

}

// loc pos 到 Map的TF变换
void PublishPositionTransform(double x,double y,double th,tf::TransformBroadcaster broadcaster,std::string fixed_frame_id,std::string base_frame_id,std::string laser_frame_id)
{
    //NAV350输出的是 sick to map 我们需要转换到 base_link to map
    //这里不是简单的加减运算，涉及到旋转变换
    tf::StampedTransform trans;
    try{
        global_listener->lookupTransform( base_frame_id, laser_frame_id ,ros::Time(0),trans);
    }
    catch(...){
        ROS_ERROR("Listen the tf from %s to %s fail.Fail to pub pos.return!", laser_frame_id.c_str(), base_frame_id.c_str() );
        return;
    }
    ros::Time current_time;
    current_time=ros::Time::now();

    geometry_msgs::TransformStamped trans_broader;
    trans_broader.header.stamp = current_time;
    trans_broader.header.frame_id =fixed_frame_id;// "map"
    trans_broader.child_frame_id = base_frame_id;// "base_link"
    //trans_broader.child_frame_id = laser_frame_id;// "laser_frame"

    ///scan 到base_link的四元素坐标转换为RPY坐标
    geometry_msgs::Quaternion temp;
    tf::quaternionTFToMsg(trans.getRotation(), temp) ;
    double yaw= tf::getYaw(temp);
    std::cout << "TF from <laserscan to base_link> (x,y,yaw):( " << trans.getOrigin().x() << "," << trans.getOrigin().y() << "," << yaw <<" )" <<std::endl;

    ///将base_link相对scan的相对坐标值转到世界坐标系下的位置。
    /// 相对坐标系原点在世界坐标scan的位置(global)
    /// 相对坐标系与世界坐标系的夹角就是激光的朝向角
    VecPosition global_scan(x,y);//scan 绝对坐标系
    VecPosition relative_base_link(-trans.getOrigin().x(),-trans.getOrigin().y());//base_link 相对与scan 坐标系下的位置
            double relative_ang_deg = Rad2Deg(th);
    VecPosition global_base_link = relative_base_link.relativeToGlobal(global_scan,relative_ang_deg);


    trans_broader.transform.translation.x = global_base_link.getX();//global x coordinate
    trans_broader.transform.translation.y = global_base_link.getY(); //global y coordinate
    trans_broader.transform.translation.z = 0;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(th-yaw);
    trans_broader.transform.rotation = quat;

    //send the transform
    broadcaster.sendTransform(trans_broader);
    std::cout
        << " scan pos (x,y,yaw):( " << x << "," << y << "," << th <<" )"
        << "\nloc pos (x,y,yaw) :( " <<trans_broader.transform.translation.x << "," << trans_broader.transform.translation.y << "," << th-yaw <<" )"
        << "\ndelta_angle: " << atan2(y-trans_broader.transform.translation.y, x-trans_broader.transform.translation.x)
        << "\ndelta_angle: " << atan2(x-trans_broader.transform.translation.x, trans_broader.transform.translation.y-y)
        << std::endl;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sicknav350_loc");
    int port;
    std::string ipaddress;
    std::string laser_frame_id,base_frame_id, fixed_frame_id;
    std::string scan;
    bool inverted;
    bool publish_tf_,publish_scan_;
    int sick_motor_speed = 8;//10; // Hz
    double sick_step_angle = 0.25;//0.5;//0.25;
    double active_sector_start_angle = 0;
    double active_sector_stop_angle = 360;;
    double scan_duration = 0.125;
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");
    nh_ns.param<std::string>("scan", scan, "scan");
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 100);
    nh_ns.param<bool>("publish_tf", publish_tf_, true);
    nh_ns.param<bool>("publish_scan", publish_scan_, true);

    nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
    nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
    nh_ns.param("inverted", inverted, false);

    nh_ns.param<std::string>("fixed_frame_id", fixed_frame_id, "odom_frame"); //global cooridnate frame measurement for navigation and position based on reflectors
    nh_ns.param<std::string>("base_frame_id", base_frame_id, "base_link_frame");// a fixed frame eg: odom or base or reflector
    nh_ns.param<std::string>("laser_frame_id", laser_frame_id, "front_laser_frame"); //laser frame

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
   double cur_x,cur_y,cur_theta;
   tf::TransformListener listener;
   global_listener =&listener;
  ///-------------------------------- Init the NAV350 parameter-------------------------------------------------////
    try {
        /* Initialize the device */
        sick_nav350.Initialize();

        try {
          //sick_nav350.SetOperatingMode(1);//0 POWERDOWN / 1 standby  2 ,MAPPING(if we want change the mode from power down to navigation or landmark detection ,we must change the mode to standby firstly!!)
            sick_nav350.SetOperatingMode(1);//1 standby
            sick_nav350.SetOperatingMode(4);//3 landmark detecting, 4 navigation

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

    sleep(5);//wait for init pos suc
    for(;;){
      ROS_INFO("------------------------Getting Init Nav Pos...-------------------------------");
      sick_nav350.GetDataNavigation(1,1);// 0 instantly 1 next , 0 pos 1 pos+scan 2 p + S + rf
      const int s = sick_nav350.GetNavStatus();
      if( s == 0 ){
         sick_nav350.GetPosMeasurements(cur_x,cur_y,cur_theta);
         ROS_INFO("Get Nav Pos Suc.(x,y,theta):(%.0lf,%.0lf,%.0lf).Change to mapping....",cur_x,cur_y,cur_theta);
         break;
     }
     else{
       ROS_INFO("ERROR!!!Get Nav Pos Failed,status:%d.do wait to get pos.get pos status:%d.....",s);
       sleep(scan_duration);
     }
   }
///---------------------------get newest data from 350-----------------------------------------------------------------////
        while (ros::ok()) {
          /* Get the scan and pos measurements */
         sick_nav350.GetDataNavigation(1,1);// 0 instantly 1 next / 0 pos 1 pos+scan 2 p + S + rf
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


    double x1=(double) sick_nav350.PoseData_.x;
    double y1=(double) sick_nav350.PoseData_.y;
    double phi1=(double )sick_nav350.PoseData_.phi;

    double x2 = x1/1000;
    double y2 = y1/1000;
    double phi2 = phi1 /1000 ;
    if(phi2>=180)phi2 -= 360;
    phi2*=DEG2RAD;

    if(publish_tf_)
    {
        PublishPositionTransform(x2,y2,phi2,odom_broadcaster,fixed_frame_id,base_frame_id,laser_frame_id); //publish position data as transform in map frame
    }

    if ( sector_start_timestamp < last_time_stamp )
    {
        loop_rate.sleep();
        ros::spinOnce();
        continue;
    }
       double temp_stamp =last_time_stamp ;
        last_time_stamp=sector_start_timestamp;
        ros::Time end_scan_time = ros::Time::now();

        double scan_duration = 0.125;

        ros::Time start_scan_time = end_scan_time - ros::Duration(scan_duration);
        if(publish_scan_)
       {
            printf("nav350.chq.main.  publishing scan...scan timestamp:%.0lf,delta time(ms):%.0lf\n",last_time_stamp,last_time_stamp-temp_stamp);
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
                        laser_frame_id,
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

