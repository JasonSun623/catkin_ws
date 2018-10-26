#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "filterscan.h"
//#include "pf_filtercenter/GALGO_Lib/Galgo.hpp"
#include <string>
#include <visualization_msgs/Marker.h>
int filter_num = 1;
boost::mutex mut;
boost::mutex odom_lock_;
boost::mutex scan_lock_;
std::vector<sensor_msgs::LaserScan> raw_scan;
void CallBackScan(const sensor_msgs::LaserScan & scan){
  boost::mutex::scoped_lock l(scan_lock_);
  static int push_num = 0;

  if(push_num < filter_num){
    raw_scan.push_back(scan);
     push_num++;
  }
  else{
      raw_scan.pop_back();
      raw_scan.push_back(scan);
  }
   std::cout << "pf_filter.push scan size:" << raw_scan.size()<<std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pepper_fuchs_filtercenter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate r(20);
    double reflector_radius = 0.0251;
    int echo_thread = 101;
    double step = 0.0005;//0.5mm
    double err_thread = 0.0051;
    std::string scan_topic = "/r2000_node/scan";
    std::string pub_rfs_topic = "pf_reflectors";
    ros::Subscriber scan_rec = nh.subscribe(scan_topic,1,CallBackScan);
    ros::Publisher pub_ref_center = nh.advertise<geometry_msgs::PoseArray>(pub_rfs_topic,1);
    ros::Publisher pub_marker_rfs = nh.advertise<visualization_msgs::Marker>("pub_rfs_marker",1);
    visualization_msgs::Marker items_maker;

    nh_.param("scan_topic", scan_topic, scan_topic);
    nh_.param("echo_thread", echo_thread, echo_thread);
    nh_.param("reflector_radius",reflector_radius,reflector_radius);
    nh_.param("step",step,step);
    nh_.param("err_thread",err_thread,err_thread);
    std::cout << "pf_filter.main.init done!"<< " scan_topic sub: " << scan_topic << std::endl ;
    ros::Duration(0.5).sleep();
    filterScan filter(reflector_radius,echo_thread,step,err_thread);
    pub_marker_rfs.publish(items_maker);

    while(nh.ok()){
       std::cout  << "pf_filter.main.loop\n";
      if(raw_scan.size()>=filter_num){
        std::cout  << "pf_filter.main.grouping the scan data\n";
        std::vector<std::list<scanCluster> >  rf_group;
        std::vector<std::list<VecPositionDir> >  relative_pointclounds;
        std::vector<VecPositionDir> v_rfs_center;
        std::vector<VecPositionDir> v_optrfs_center;
        filter.groupingScan(raw_scan,rf_group);
        filter.getInitCenter(rf_group,v_rfs_center);
        filter.getRelativePointClouds(rf_group,v_rfs_center,relative_pointclounds);
        filter.getOptimizedCenter(v_rfs_center,relative_pointclounds,v_optrfs_center);
        ROS_INFO("pf_filtercenter.pub rfs_points_ size:%d",v_optrfs_center.size() );
        ///common info for item
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
        /// 一种类型的所有点可以放入一个marker的points中
        if(v_optrfs_center.size() ){
          for(int i=0;i < v_optrfs_center.size();i++){
            sum+=1;
            items_maker.id = sum;
            items_maker.header.stamp = ros::Time::now();
            geometry_msgs::Point p;
            p.x = v_optrfs_center[i].getX();
            p.y = v_optrfs_center[i].getY();
            p.z = 0;
            items_maker.points.push_back(p);
          }
        }
        pub_marker_rfs.publish(items_maker);
      }
      r.sleep();
      ros::spinOnce();;//attention this ros::spinOnce(); shoud put in cur loop otherwise we would not rec ros topoc
    }

    return 0;

}


