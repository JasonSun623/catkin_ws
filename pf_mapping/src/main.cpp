#include "pf_mapping.h"
#include "pf_slam.h"
#include <ros/ros.h>
using namespace  pf_slam_space;

int main(int argc,char** argv){
  ros::init(argc,argv,"pf_mapping_node");
  //pf_mapping mapping;
  pfLocalize* slam = new pf_slam;
  //for debug
  if( argc >= 4)
    slam->setInitPos(atof(argv[1]),atof(argv[2]),atof(argv[3]));
  ros::spin();
  return 0;
}
