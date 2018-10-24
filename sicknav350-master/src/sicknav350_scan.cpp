
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
#include <iomanip>
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


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sicknav350_scan");
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
    double scan_duration = 0.125;
    std::string laser_frame_id,laser_child_frame_id,odom_frame_id;
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");
    nh_ns.param<std::string>("scan", scan, "scan");
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 100);

    nh_ns.param<bool>("publish_scan", publish_scan_, true);

    nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
    nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
    nh_ns.param("inverted", inverted, false);
    nh_ns.param<std::string>("frame_id", frame_id, "front_laser_frame"); //laser frame
    nh_ns.param<std::string>("fixed_frame_id", fixed_frame_id, "base_link_frame"); //robot base frame

    nh_ns.param<std::string>("laser_frame_id", laser_frame_id, "odom_frame"); //global cooridnate frame measurement for navigation and position based on reflectors
    nh_ns.param<std::string>("laser_child_frame_id", laser_child_frame_id, "base_link_frame");// a fixed frame eg: odom or base or reflector

    nh_ns.param("resolution", sick_step_angle, 1.0);
    nh_ns.param("start_angle", active_sector_start_angle, 0.);
    nh_ns.param("stop_angle", active_sector_stop_angle, 360.);

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
  //double  test_float = 123.45;
  string test_str="1234567\n\t\\\' " ;

  //std:cout << "test_float: " << setiosflags(ios::fixed) << setprecision(3)<< test_float <<std::endl;
  std::cout
    << "test str: "
    << setiosflags(ios::showpos) << test_str
    << " cap: " << test_str.capacity()
    << " size: " << test_str.size()
      << " len: " << test_str.length()
    << std::endl;
  ///-------------------------------- Init the NAV350 parameter-------------------------------------------------////
    try {
        /* Initialize the device */
        sick_nav350.Initialize();

        try {
          //sick_nav350.SetOperatingMode(1);//0 POWERDOWN / 1 standby  2 ,MAPPING(if we want change the mode from power down to navigation or landmark detection ,we must change the mode to standby firstly!!)

            sick_nav350.SetOperatingMode(1);//1 standby
            sick_nav350.SetOperatingMode(3);//3 landmark detecting, 4 navigation

            sick_nav350.SetLandMarkDataFormat(1,1,1);//chq set data output format 0 cartesian cord,1 enable show opt param ,1 Select transmission of the detected  data in positioning mode withNPOSGetData
            sick_nav350.SetScanDataFormat(1,1);//chq set scan data output format 1 dist only ,1 enable output remission data

            uint8_t mode,show,cord_frame,landmarkfilter;
            //check setting
            
            sick_nav350.GetLandMarkDataFormat(cord_frame,show,landmarkfilter);//chq get landmark data output format
            sick_nav350.GetScanDataFormat(mode,show);//chq get scan data output format
        } catch (...) {
            ROS_ERROR("Configuration error");
            return -1;
        }

    ros::Rate loop_rate(10);
    ///---------------------------get newest data from 350-----------------------------------------------------------------////
     while (ros::ok()) {
            /* Get the scan and landmark measurements */
          sick_nav350.GetDataLandMark(0,1);//---0 last calculated scan 1 new cal scan , 0 rf 1 rf+scan
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


          if ( sector_start_timestamp <= last_time_stamp )
          {
              printf("main.cur get sick newest timestamp: %.0lf <= last record time stamp:%.0lf .continue\n",sector_start_timestamp,last_time_stamp);
              loop_rate.sleep();
              ros::spinOnce();
              continue;
          }
          double temp_stamp =last_time_stamp ;
        last_time_stamp=sector_start_timestamp;
        ros::Time end_scan_time = ros::Time::now();
        ros::Time start_scan_time = end_scan_time - ros::Duration(scan_duration);
        if(publish_scan_)
       {
            printf("nav350.chq.main.  publishing scan...scan timestamp:%.0lf,delta time(ms):%.0lf\n",last_time_stamp,last_time_stamp-temp_stamp);
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
        //record newest sick cal scan time
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

