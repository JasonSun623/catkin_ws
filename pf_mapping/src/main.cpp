#include "pf_mapping.h"
#include <ros/ros.h>
using namespace  pf_mapping_space;

int main(int argc,char** argv){
  ros::init(argc,argv,"pf_mapping_node");
  pf_mapping mapping;
  //for debug
  if( argc >= 4)
    mapping.setInitPos(atof(argv[1]),atof(argv[2]),atof(argv[3]));
  ros::spin();
  return 0;
}
