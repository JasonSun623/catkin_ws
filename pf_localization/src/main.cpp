#include <ros/ros.h>
#include "pf_localize.h"
using namespace pf_localization_space;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "houdar_pf_localize");
    ros::NodeHandle nh;
    pfLocalize loc;
    //for debug
    if( argc >= 4)
      loc.setInitPos(atof(argv[1]),atof(argv[2]),atof(argv[3]));
    loc.localize();
    ros::spin();
    return 0;

}


