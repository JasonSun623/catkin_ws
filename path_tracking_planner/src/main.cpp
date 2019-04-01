#include "path_tracking_planner.h"
#include <boost/smart_ptr.hpp>

// MAIN
int main(int argc, char** argv)
{

  ros::init(argc, argv, "path_tracking_planner", ros::init_options::AnonymousName);

  // TODO: remove debug level
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
      ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nh_;

//  path_tracking_planner::PathTrackingPlanner planner(nh_);

  boost::shared_ptr<path_tracking_planner::PathTrackingPlanner> planner;

  planner.reset(new path_tracking_planner::PathTrackingPlanner(nh_));

  ros::Duration(0.1).sleep();

  if (!planner->init())
  {
    ROS_ERROR("%s initialization failed", ros::this_node::getName().c_str());
    return -1;
  }

  planner->ControlThread();

  return (0);
}
