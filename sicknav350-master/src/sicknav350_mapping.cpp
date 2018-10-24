
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
#include <signal.h> // signal functions
#define DEG2RAD M_PI/180.0
#define RAD2DEG 180/M_PI
using namespace std;
using namespace SickToolbox;
SickNav350 *global_nav350;
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
    double init_x,init_y,init_theta;
    int nclosestrfs,replaceorappend;//n closest reflectors , 0 replace first when mapping ,1 append
    int wait_mapping_interval;//wait for move the car to detect new lmks
    int mapping_layout;//specified mapping layout
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

    nh_ns.param("cord_x", init_x, 0.0);
    nh_ns.param("cord_y", init_y, 0.0);
    //0 -2*pi
    nh_ns.param("cord_theta", init_theta, 0.0);

    nh_ns.param("NClosestRFCnt", nclosestrfs, 0);
    nh_ns.param("ReplaceORAppendFirst", replaceorappend, 0);//0 replace 1 app
    nh_ns.param("mappinglayout", mapping_layout, 0);//0 layout0 ...
    nh_ns.param("wait_mapping_interval",wait_mapping_interval,120);

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
   global_nav350 = &sick_nav350 ;
    //  ros::Duration(50).sleep(); //timedelay for jackal robot startup jobs
    double last_time_stamp=0;
  ///-------------------------------- Init the NAV350 parameter-------------------------------------------------////
    try {
       ROS_INFO("Try to init Device ...");
        /* Initialize the device */
        sick_nav350.Initialize();
        try {
           //set operation mode
            ROS_INFO("-----------------------Init.SetOperatingMode-StandBy-------------------------------");
            sick_nav350.SetOperatingMode(1);//0 POWERDOWN / 1 standby  2 ,MAPPING(if we want change the mode from power down to navigation or landmark detection ,we must change the mode to standby firstly!!)
            //set data check format
            ROS_INFO("-----------------------Init.Set Data Format----------------------------------------");
            sick_nav350.SetPositioningDataFormat(1,0);//chq set data output format, 0 get last caled pos 1 get new pos , 0 show opt param data
            //set for getting lmk data in mapping
            sick_nav350.SetLandMarkDataFormat(1,1,1);//chq set data output format 0 cartesian cord,1 enable show opt param ,1 Select transmission of the detected  data in positioning mode withNPOSGetData
            sick_nav350.SetScanDataFormat(1,1);//chq set scan data output format 1 dist only ,1 enable output remission data
            //check setting
           uint8_t mode,show,cord_frame,landmarkfilter;
           //sick_nav350.GetPositioningDataFormat(mode,show);//chq get positioning data output format
           //sick_nav350.GetScanDataFormat(mode,show);//chq get scan data output format
           //set specified layer for mapping
          ROS_INFO("------------------------Init.Set Cur Layout:%d----------------------------------------",mapping_layout);
          sick_nav350.SetCurrentLayer(mapping_layout);
          ROS_INFO("------------------------Init.Set N Closest RF:%d--------------------------------------",nclosestrfs);
          sick_nav350.SetNClosestReflectors(nclosestrfs);
          ///firstly we erase the layout in device if the ReplaceORAppendFirst (new map)
          if( !replaceorappend ){
              ROS_INFO("------------------------Init.Before mapping ,Set replace map mode--------------------");
              sick_nav350.SetEraseLayout(0);
              }
          else
             ROS_INFO("------------------------Init.Before mapping ,Set appending map mode-----------------");
        } catch (...) {
            ROS_ERROR("SickNav350_Mapping Configuration error");
            return -1;
        }
    ROS_INFO("------------------------Init Device Done! Now Mapping Start--------------------------");
    ros::Rate loop_rate(10);
    int count = 0;
    double cur_x,cur_y,cur_theta;
    /// ----------------------------------------------
    // signal(SIGINT, save_exit); //相应ctrl+函数 （保存数据用）
   /// ----------------------------------------------
     while (ros::ok()) {

           /* Get the scan and landmark measurements */
         if( count < 1 ){
             count++;
             cur_x  = (int)(init_x) * 1000;
             cur_y  = (int)(init_y) * 1000;
             cur_theta  = (int)(init_theta * RAD2DEG) * 1000;
         }
         ROS_INFO("-----------------------1.Conf mapping-------------------------------------");
         std::cout << "Conf mapping .cur count: " << count << " (cur_x,cur_y,cur_theta) ( " << cur_x   << " , " << cur_y << " ," << cur_theta << " ) " <<std::endl;
         /// -------------------------------Conf mapping -----------------------------------------------------------------------------
         sick_nav350.ConfigureMapping(50,1,cur_x,cur_y,cur_theta);// mean , negtive map(0 pos 1 neg),x y theta pos(mm)
        ///---------------------------------Do Mapping-------------------------------------------------------------------------------
         ROS_INFO("-----------------------2.Do mapping---------------------------------------");
         sick_nav350.SetOperatingMode(2);
         sick_nav350.DoMapping();
         const int s = sick_nav350.GetMappingStatus();
         if(  s!=0 ){
              ROS_ERROR("ERROR.Init mapping error!!!  return status:%d.<-------------DO NOT MOVE CAR----------->redo init mapping",s);
              count = 0;
              sleep(scan_duration);
               sick_nav350.SetOperatingMode(1);//return to normal mode
              continue;
         }
         //else next
         /// ------------------------------Save rf data in layout-------------------------------------------------------
         int rf_cnt = sick_nav350.ReflectorData_.num_reflector;
          if(rf_cnt)
              ROS_INFO("------------------------3.Adding rf into device.all cnt:%d.( x, y ):-------",rf_cnt);
          else
              ROS_INFO("------------------------3.No New RFs.all cnt:%d-----------------------------",rf_cnt);
         for( int i=0; i < rf_cnt; i++ ){
             sick_nav350.AddLandmark(1,//every num = 1
                                     sick_nav350.ReflectorData_.x[i],
                                     sick_nav350.ReflectorData_.y[i],
                                     sick_nav350.ReflectorData_.type[i],
                                     sick_nav350.ReflectorData_.subtype[i],
                                     sick_nav350.ReflectorData_.size[i],
                                     1,//layout1
                                     mapping_layout);
          ROS_INFO("( %.0lf, %.0lf )",sick_nav350.ReflectorData_.x[i],sick_nav350.ReflectorData_.y[i]);
         }
         ///------------------------------- Save layout into device---------------------------------------------------
         if(rf_cnt)
             {
                 ROS_INFO("------------------------3.1Store new lmk into device------------------------");
                 sick_nav350.SetStoreCurrentLayer();

             }

         ///---------------------------------Get nav pos------------------------------------------------------------------

         ROS_INFO("------------------------4.<Don't Move!!!>.Change to nav.waitting 5s...--");
         sleep(5);//wait for stopping
         sick_nav350.SetOperatingMode(4);
         sleep(5);//wait for init pos suc
         for(;;){
           ROS_INFO("------------------------5.Getting Init Nav Pos...-------------------------------");
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
        ROS_INFO("------------------------<-----------Suc in Nav. Now Can Move---------------> The Car To New LMKs.SLEEP:%d s-----",wait_mapping_interval);
        sleep(wait_mapping_interval);
        ROS_INFO("-----------------------<------------From Now Don't Move---------------------->-------------------------------------");
        sleep(5);
        ///get new pos for next mapping
        for(;;){
          ROS_INFO("------------------------6.Getting Newest Nav Pos...-------------------------------");
          sick_nav350.GetDataNavigation(1,1);// 0 instantly 1 next , 0 pos 1 pos+scan 2 p + S + rf
          const int s = sick_nav350.GetNavStatus();
          if( s == 0 ){
             sick_nav350.GetPosMeasurements(cur_x,cur_y,cur_theta);
             ROS_INFO("Get Newest Nav Pos Suc.(x,y,theta):(%.0lf,%.0lf,%.0lf).Change to mapping....",cur_x,cur_y,cur_theta);
             break;
         }
         else{
           ROS_INFO("ERROR!!!Get Nav Pos Failed,status:%d.do wait to get pos.get pos status:%d.....",s);
           sleep(scan_duration);
         }
       }

        if(publish_scan_)
       {
            double temp_stamp;
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
           temp_stamp =last_time_stamp ;
           last_time_stamp=sector_start_timestamp;
           ros::Time end_scan_time = ros::Time::now();
           ros::Time start_scan_time = end_scan_time - ros::Duration(scan_duration);
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
    ROS_INFO("Error return 0");
    return 0;
}


