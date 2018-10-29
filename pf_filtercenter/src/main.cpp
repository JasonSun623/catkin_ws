#include <ros/ros.h>
#include <string>
#include <sstream>
#include "filterscan.h"
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

int filter_num = 1;
boost::mutex scan_lock_;
std::vector<sensor_msgs::LaserScan> raw_scan;

//选取filter_num个连续激光束　用于平均
void CallBackScan(const sensor_msgs::LaserScan & scan){
  boost::mutex::scoped_lock l(scan_lock_);
  static int push_num = 0;
  if( push_num < filter_num ){
    raw_scan.push_back(scan);
    push_num++;
  }
  else{
    raw_scan.pop_back();
    raw_scan.push_back(scan);
  }
  //ROS_INFO_STREAM("pf_filter.push scan size:" << raw_scan.size() );
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pepper_fuchs_filter_center");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    //20 hz <--->50 ms
    ros::Rate r(20);

    int echo_thread = 101;
    double reflector_radius = 0.0251;
    double step = 0.0005;//0.5mm
    double err_thread = 0.0011;
    std::string scan_topic = "/r2000_node/scan";
    std::string pub_rfs_topic = "pf_reflectors";

    nh_.param("scan_topic", scan_topic, scan_topic);
    nh_.param("echo_thread", echo_thread, echo_thread);
    nh_.param("reflector_radius",reflector_radius,reflector_radius);
    nh_.param("step",step,step);
    nh_.param("err_thread",err_thread,err_thread);
    ROS_INFO_STREAM("pf_filter.main.init done!"<< " scan_topic sub: " << scan_topic);

    //subscribe scan topic
    ros::Subscriber scan_rec = nh.subscribe(scan_topic,1,CallBackScan);
    //pub the rfs center data
    ros::Publisher pub_ref_center = nh.advertise<geometry_msgs::PoseArray>(pub_rfs_topic,1);
    //pub the visualization of the rfs center data
    ros::Publisher items_array_pub = nh.advertise<visualization_msgs::MarkerArray>("pub_rfs_marker_array",1,true);

    visualization_msgs::Marker items_maker;
    visualization_msgs::Marker text_marker;
    visualization_msgs::MarkerArray items_MarkerArray;
    ros::Duration(0.5).sleep();

    //construct the filter scan object
    filterScan filter(reflector_radius,echo_thread,step,err_thread);

    while(nh.ok()){
      //std::cout  << "pf_filter.main.loop\n";
      if(raw_scan.size() >= filter_num){
        //std::cout  << "pf_filter.main.grouping the scan data\n";
        std::vector<VecPositionDir> v_optrfs_center;
        ros::Time t = ros::Time::now();
        //cal the cneter of rfs
        filter.getReflectorsCenter(raw_scan,v_optrfs_center);

        ros::Duration waster = (ros::Time::now() - t)*1000;
        ROS_INFO_STREAM("cal the opt rfs waste(ms):" << waster << "rfs num: " << v_optrfs_center.size());

        ///common info for item
        items_MarkerArray.markers.clear();
        items_maker.points.clear();
        items_maker.header.frame_id="scan";
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
        items_maker.lifetime = ros::Duration(1000000);
         ///common info for text
        int sum =0;
        //发布可视化反光板中心　和　文字
        if(v_optrfs_center.size() ){
          for(int i=0;i < v_optrfs_center.size();i++){
            sum+=1;
            items_maker.id = sum;
            items_maker.header.stamp = ros::Time::now();
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
          }
        }
        items_MarkerArray.markers.push_back(items_maker);
        items_array_pub.publish(items_MarkerArray);
      }
      r.sleep();
      //attention this ros::spinOnce(); shoud put in cur loop otherwise we would not rec ros topoc
      ros::spinOnce();;
    }

    return 0;

}


