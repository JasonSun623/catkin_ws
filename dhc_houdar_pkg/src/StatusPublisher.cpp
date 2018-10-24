#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <memory.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

#define DISABLE 0
#define ENABLE 1
#define R   0.1015     //轮子半径
#define L1_L2   0.595  //L1 + L2


namespace xqserial_server
{

StatusPublisher::StatusPublisher()
{
    mbUpdated=false;

    car_status.w1 = 0.0;
    car_status.w2 = 0.0;
    car_status.w3 = 0.0;
    car_status.w4 = 0.0;

    car_status.vx = 0.0;
    car_status.vy = 0.0;
    car_status.w  = 0.0;

    car_status.x = 0.0;
    car_status.y = 0.0;
    car_status.theta = 0.0;

    car_status.electric_voltage = 0.0;

    current_time = ros::Time::now();
    last_time    = current_time; 
    current_getNum = 0;

   mOdomPub = mNH.advertise<nav_msgs::Odometry>("odom", 1, true);
   mVoltagePub = mNH.advertise<std_msgs::Float32>("voltage", 1, true);
}

void StatusPublisher::Update(const byte data[], unsigned int len)
{
    boost::mutex::scoped_lock lock(mMutex);

    static byte last_str= 0x00;
    byte current_str=0x00;
    //static byte new_packed_ctr=DISABLE;//ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；

    for(int i=0;i<len;i++)
    {
        current_str=data[i];
        //判断是否有新包头
        if(last_str==2&&current_str==3) //包头 02 03
        {
          cout << "new data"  << endl;
          last_str=current_str;
          current_getNum = 0;
          continue;
        }
        last_str=current_str;

         read_data[current_getNum] = current_str;
         current_getNum++;
         
         if(current_getNum==data_num) mbUpdated = true;
    }

    return;
}


void StatusPublisher::Refresh()
{
     boost::mutex::scoped_lock lock(mMutex);
     
    if(mbUpdated)
    {
          //for(int i=0; i< 16; i++)
            //  printf("read_data:  %x    ", read_data[i]);

          if(  (read_data[3] | 0x7f) == 0xff )
          {
             car_status.w1 = 0xffffffff - ( read_data[0]  | (read_data[1]<<8) &0xff00 | (read_data[2]<<16)&0xff0000 |  (read_data[3]<<24)&0xff000000 ) + 1 ;  
             car_status.w1 = 0 - car_status.w1;
          }
          else 
          {
             car_status.w1 = read_data[0]  | (read_data[1]<<8) &0xff00 | (read_data[2]<<16)&0xff0000 |  (read_data[3]<<24)&0xff000000  ;
          }
          if(  (read_data[7] | 0x7f) == 0xff )
          {
             car_status.w2 = 0xffffffff - ( read_data[4]  | (read_data[5]<<8) &0xff00 | (read_data[6]<<16)&0xff0000 |  (read_data[7]<<24)&0xff000000 ) + 1 ;  
             car_status.w2 = 0 - car_status.w2;
          }
          else 
          {
             car_status.w2 = read_data[4]  | (read_data[5]<<8) &0xff00 | (read_data[6]<<16)&0xff0000 |  (read_data[7]<<24)&0xff000000  ;
          }
          if(  (read_data[11] | 0x7f) == 0xff )
          {
             car_status.w3 = 0xffffffff - ( read_data[8]  | (read_data[9]<<8) &0xff00 | (read_data[10]<<16)&0xff0000 |  (read_data[11]<<24)&0xff000000 ) + 1 ;  
             car_status.w3 = 0 - car_status.w3;
          }
          else 
          {
             car_status.w3 = read_data[8]  | (read_data[9]<<8) &0xff00 | (read_data[10]<<16)&0xff0000 |  (read_data[11]<<24)&0xff000000  ;
          }
          if(  (read_data[15] | 0x7f) == 0xff )
          {
             car_status.w4 = 0xffffffff - ( read_data[12]  | (read_data[13]<<8) &0xff00 | (read_data[14]<<16)&0xff0000 |  (read_data[15]<<24)&0xff000000 ) + 1 ;  
             car_status.w4 = 0 - car_status.w4;
          }
          else 
          {
             car_status.w4 = read_data[12]  | (read_data[13]<<8) &0xff00 | (read_data[14]<<16)&0xff0000 |  (read_data[15]<<24)&0xff000000  ;
          }
           
          printf("read_data[16]   %x     read_data[17]    %x  \n",   read_data[16],   read_data[17]  );
          car_status.electric_voltage = ( (read_data[16]<<8) & 0xff00 | read_data[17]  )  / 10.0;
          std_msgs::Float32   ele_vol;
          ele_vol.data = car_status.electric_voltage;
          mVoltagePub.publish(ele_vol);

          car_status.w1 *= (PI/2500000);
          car_status.w2 *= (PI/2500000);
          car_status.w3 *= (PI/2500000);
          car_status.w4 *= (PI/2500000);

          car_status.vx = R/4.0 * (car_status.w1 + car_status.w2 + car_status.w3 + car_status.w4);
          car_status.vy = R/4.0 * (-car_status.w1 + car_status.w2 + car_status.w3 - car_status.w4);
          car_status.w  = R/4.0/L1_L2 * (-car_status.w1 + car_status.w2 - car_status.w3 + car_status.w4);


      // Time
      current_time = ros::Time::now();
      double delta_time = (current_time - last_time).toSec();
      last_time = current_time;

      car_status = get_robot_state(car_status, delta_time);
      
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.frame_id = "/odom";
        odom_trans.child_frame_id  = "/base_link";
        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf::createQuaternionMsgFromYaw(car_status.theta);
        //update transform
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = car_status.x;
        odom_trans.transform.translation.y = car_status.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //publishing the new tf
        static tf::TransformBroadcaster br;
        br.sendTransform(odom_trans);
        //filling the odometry
        CarOdom.header.stamp = current_time;
        CarOdom.header.frame_id = "/odom";
        CarOdom.child_frame_id = "/base_link";
        //position
        CarOdom.pose.pose.position.x = car_status.x;
        CarOdom.pose.pose.position.y = car_status.y;
        CarOdom.pose.pose.position.z = 0.0;
        CarOdom.pose.pose.orientation = odom_quat;
        CarOdom.pose.covariance =  boost::assign::list_of(0.001) (0) (0)  (0)  (0)  (0)
                                                              (0) (0.001)  (0)  (0)  (0)  (0)
                                                              (0)   (0)  (999) (0)  (0)  (0)
                                                              (0)   (0)   (0) (999) (0)  (0)
                                                              (0)   (0)   (0)  (0) (999) (0)
                                                              (0)   (0)   (0)  (0)  (0)  (0.001) ;
        //velocity
        CarOdom.twist.twist.linear.x = car_status.vx;
        CarOdom.twist.twist.linear.y = car_status.vy;
        CarOdom.twist.twist.linear.z = 0.0;
        CarOdom.twist.twist.angular.x = 0.0;
        CarOdom.twist.twist.angular.y = 0.0;
        CarOdom.twist.twist.angular.z = car_status.w;
        CarOdom.twist.covariance =  boost::assign::list_of(0.001) (0) (0)  (0)  (0)  (0)
                                                              (0) (0.001)  (0)  (0)  (0)  (0)
                                                              (0)   (0)  (999) (0)  (0)  (0)
                                                              (0)   (0)   (0) (999) (0)  (0)
                                                              (0)   (0)   (0)  (0) (999) (0)
                                                              (0)   (0)   (0)  (0)  (0)  (0.001) ;
        mOdomPub.publish(CarOdom);
        mbUpdated = false;
        ros::spinOnce();

        
    }
}

UPLOAD_STATUS StatusPublisher::get_robot_state(UPLOAD_STATUS last_state, double delta_t)
{
  UPLOAD_STATUS current_state = last_state;
  current_state.x = ( last_state.vx * cos(last_state.theta) - last_state.vy * sin(last_state.theta) ) * delta_t * 1.0148 + last_state.x;
  current_state.y = ( last_state.vx * sin(last_state.theta) + last_state.vy * cos(last_state.theta) ) * delta_t + last_state.y;
  current_state.theta = last_state.w * delta_t * 0.98222  + last_state.theta;
  return current_state;
}

} //namespace xqserial_server
