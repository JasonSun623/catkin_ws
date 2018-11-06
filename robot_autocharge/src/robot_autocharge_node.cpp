
#include <ros/ros.h>
#include <ros/duration.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <boost/thread.hpp>

#include "point.h"
#include "typevdock.h"
#include "boost/bind.hpp"

#define Vshape  1.0 //the edge length  chq
#define MIN_R  0.08 //连续激光点间距，作为线条的阈值 chq


//need to modify.it is not the real RobotShapeRect class !!!!!!!!!!!! chq ---->
//in this program it is really not used,so it is not very necessary
class RobotShapeRect{
public:
  RobotShapeRect():_front(0),_back(0),_left(0),_right(0){}
    RobotShapeRect(double front, double back, double left, double right ):
      _front(front),_back(back),_left(left),_right(right){}
private:
    double _front,_back,_left,_right;
}safe_shape;
//need to modify.it is not the real RobotShapeRect class !!!!!!!!!!!! chq-----< 

int main(int argc, char ** argv){
  ros::init(argc, argv, "auto_charge_node",ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ROS_INFO("auto_charge_node init done!");
  Docking V_dock;
  double last_dis(0), out_dis(0.7);//charge in :角度调正，距离小于阈值，再行驶的距离, charge out permitted end dist
  std::vector<double> safe;
  safe.push_back(0.7);safe.push_back(0.7);safe.push_back(0.48);safe.push_back(0.48);  // front back left right
  safe_shape = RobotShapeRect(safe[0], safe[1], safe[2], safe[3]);
  /// Set v data
  V_dock.setVTypeProperty(Vshape,  120, MIN_R,last_dis, out_dis);
  if(argc>1)V_dock.setInitTargetDist(atof(argv[1]));
  /*chq auto charge main loop*/
  boost::thread thread_charge(boost::bind(&Docking::autoChargeThread, &V_dock));
  ros::spin();
}
