/*!
 * \file main.cc
 * \brief A simple application using the Sick LMS 1xx driver.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2009, Jason C. Derenick and Christopher R. Mansley
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#include <stdlib.h>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include "sicktoolbox/SickLMS1xx.hh"
#include "sicktoolbox/SickLD.hh"
#include <sensor_msgs/LaserScan.h>
#include <math.h>
//#define M_PI 3.1415927
#define DEG2RAD M_PI/180.0
#define RAD2DEG 180/M_PI

using namespace std;
using namespace SickToolbox;

void publish_scan(
        ros::Publisher *pub,
        unsigned int *range_values,
        uint32_t n_range_values,
        float scan_period,
        ros::Time start_measure_time,
        bool inverted,
        float angle_start,
        float angle_resloution,
        std::string frame_id)
{
  static int scan_count = 0;
  sensor_msgs::LaserScan scan_msg;
  scan_count++;
  //转换到ROS 坐标(将激光自定义的 坐标视角转换为 ros坐标视角)
  //减去90度
  angle_start -= M_PI/2;
  float angle_min = angle_start;//rad
  float angle_max = angle_start + angle_resloution * (n_range_values-1);//rad
  scan_msg.angle_min = angle_min;
  scan_msg.angle_max = angle_max;

  std::cout << "pub scan.scan_msg.angle_min(deg):  " << RAD2DEG * scan_msg.angle_min<< " max(deg): " << RAD2DEG * scan_msg.angle_max << std::endl;
  scan_msg.range_min = 0.1;
  scan_msg.range_max = 20.;

  scan_msg.scan_time = scan_period;//scan_time 125ms;
  scan_msg.angle_increment = angle_resloution;
  std::cout << "pub scan scan_msg.angle_increment(deg): " << RAD2DEG * scan_msg.angle_increment <<std::endl;
  scan_msg.time_increment = scan_msg.scan_time / n_range_values;
  scan_msg.ranges.resize(n_range_values);
  scan_msg.intensities.resize(n_range_values);
  scan_msg.header.stamp = start_measure_time;

   //output scan data to screen
 // std::cout << "scan data(rad，deg,dist) :\n ";
  for (size_t i = 0; i < n_range_values; i++) {
      if(!inverted){
        scan_msg.ranges[i] = ( (float)range_values[i] ) / 1000.0;
       // if( i == 10 )std::cout << "pub scan.angle(deg): "<< (float) i * 0.5<< " dist: " << range_values[i] << " " <<std::endl;
      }
      else{//倒置
       scan_msg.ranges[i] = ( (float)range_values[n_range_values-i-1] ) / 1000.0;
      // if( i == 10 )std::cout << "pub scan.angle(deg): "<< (float) (n_range_values-i-1) * 0.5<< " dist: " << range_values[i]  << " " <<std::endl;
      }

    //double a = RAD2DEG * (scan_msg.angle_min + i * scan_msg.angle_increment);
    //printf(" (%.4f,%.4lf ) ",a, scan_msg.ranges[i]);
    //if( i%2 == 0)std::cout << "\n ";
  }
 // std::cout << "\n ";
  scan_msg.header.frame_id = frame_id;
  pub->publish(scan_msg);

}

int main(int argc, char* argv[])
{
  
  /*
   * Instantiate an instance
   */

  ros::init(argc,argv,"sick_lms111");
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");
  int port=2112;
  std::string ipaddress="192.168.0.9";
  std::string frame_id, fixed_frame_id;
  std::string scan;
  ros::Rate r(50);
  bool inverted;
  bool publish_scan_;
  nh_ns.param<std::string>("scan", scan, "scan");
  nh_ns.param<bool>("publish_scan", publish_scan_, true);
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 20);

  //nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
  //nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
  //是否倒置
  nh_ns.param("inverted", inverted, true);
  nh_ns.param<std::string>("frame_id", frame_id, "scan"); //laser frame
  /*
   * Initialize the Sick LMS 2xx
   */
  SickLMS1xx sick_lms_1xx(ipaddress,port);
  try {
    sick_lms_1xx.Initialize();
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct IP address?" << endl;
    return -1;
  }

  try {
    unsigned int status = 1;
    unsigned int scan_freq = 0;
    unsigned int num_measurements = 0;
             int start_angle;
    unsigned int angle_resloution;
    unsigned int measure_start_time = 0;
    unsigned int range_1_vals[SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS];
    unsigned int range_2_vals[SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS];
    //sick_lms_1xx.SetSickScanFreqAndRes(SickLMS1xx::SICK_LMS_1XX_SCAN_FREQ_25,SickLMS1xx::SICK_LMS_1XX_SCAN_RES_25);
    sick_lms_1xx.SetSickScanDataFormat(SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_NONE);
    //sick_lms_1xx.SetSickScanDataFormat(SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_16BIT);
    int i = 0;

 while( nh.ok() ){

     sick_lms_1xx.GetSickMeasurements(range_1_vals,
                                      NULL,
                                      NULL,
                                      NULL,
                                      num_measurements,
                                      scan_freq,
                                      start_angle,
                                      angle_resloution,
                                      measure_start_time);
     //sick_lms_1xx.GetSickMeasurements(range_1_vals,NULL,NULL,NULL,num_measurements,&status);

      double start_angle_rad = (double)start_angle / 10000.0 * DEG2RAD;//convert to rad
      double angle_resloution_rad = (double)angle_resloution / 10000.0 * DEG2RAD;

      unsigned int measure_time_sec = measure_start_time / ( 1000 * 1000 ) ;//sec
      unsigned int measure_time_nsec = ( measure_start_time - measure_time_sec * 1000*1000  ) * 1000;//nsec
      ros::Time ros_start_measure_time(measure_time_sec,measure_time_nsec);
      double scan_period = 0.02;
      if( scan_freq ){
        scan_period = 1.0/(double)(scan_freq/100);
      }
     // std::cout << "\n send tele num: "<< i++ << ": " << num_measurements << " " << status
      //          << "\n scan_period(s): " << scan_period
      //          << "\n scan_start_angle(deg): " << (double)start_angle / 10000
      //          << "\n angle_resloution(deg): " << (double)angle_resloution / 10000.0
      //          << "\n measure_start_time(us): " << measure_start_time
      //          << std::endl;

      if(publish_scan_)
          publish_scan(&scan_pub,
                       range_1_vals,
                       num_measurements,
                       scan_period,
                       ros_start_measure_time,
                       inverted,
                       start_angle_rad,//rad
                       angle_resloution_rad,//rad
                       frame_id
                      );
      //if(i++ > 1) break;
      r.sleep();
      ros::spinOnce();

  }
  }
  
  catch(SickConfigException sick_exception) {
    std::cout << sick_exception.what() << std::endl;
  }

  catch(SickIOException sick_exception) {
    std::cout << sick_exception.what() << std::endl;
  }

  catch(SickTimeoutException sick_exception) {
    std::cout << sick_exception.what() << std::endl;
  }
  
  catch(...) {
    cerr << "An Error Occurred!" << endl;
    return -1;
  }
  
  
  /*
   * Uninitialize the device
   */
  try {
    sick_lms_1xx.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }

  ros::spin();
  /* Success! */
  return 0;

}
    
