#include <imu_odom_combine/imu_odom_combine.h>
using namespace imu_odom_fusion_space;
int main (int argc, char **argv)
{
  ros::init (argc, argv, "sensorFuse");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  sensorFusion sensorfus;
  ros::spin();
  return 0;
}
