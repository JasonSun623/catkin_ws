/** \file simpleclient.cpp
 * \author chq
 * \version 1.0
 * \date    2018
 *
 * \brief for testing send gotogoal
*/

#include <wash_floor_msgs/GoToAction.h>
#include <wash_floor_msgs/GoToActionFeedback.h>
#include <wash_floor_msgs/GoToGoal.h>
#include <wash_floor_msgs/GoToActionResult.h>
#include <wash_floor_msgs/GoToFeedback.h>
#include <wash_floor_msgs/GoToResult.h>
#include <wash_floor_msgs/goal.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<wash_floor_msgs::GoToAction> Client;
//这些消息类型名要与服务器发布的类型相对应
void doneCb(const actionlib::SimpleClientGoalState & state,
            const wash_floor_msgs::GoToResultConstPtr& result){
  ROS_INFO("yay!!!.done waypoints");
}
void actionActived(){
  ROS_INFO("server is connected suc!");
}
void feedbackCb(const wash_floor_msgs::GoToFeedbackConstPtr& feedback){
  ROS_INFO("percent comp perc:%.3f",feedback->percent_complete);
}

int main(int argc, char** argv)
{

  ///程序会以这个名字发布server,所以不要轻易改这个个名字
  ros::init(argc, argv, "planner_status_client");
  ros::NodeHandle n;
  Client client("path_tracking_planner",true);
  ROS_INFO("PurepursuitClient waiting for server start...") ;
  ros::Duration(5).sleep();
  client.waitForServer();
  ROS_INFO("PurepursuitClient watting for server suc!");

  ros::Rate rate(1.0);

  while ( ros::ok() )
  {
    rate.sleep();
    ros::spinOnce();

    if(client.isServerConnected()){

        actionlib::SimpleClientGoalState goal_state = client.getState();

        ROS_INFO("[ PurepursuitClient ] Path traking goal has %s state!", goal_state.toString().c_str());
    }







  }


   ///stop proc
//   ros::Duration(5).sleep();
//   client.cancelGoal();
//   ros::spin();
  return 0;
}
// EOF
