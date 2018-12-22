/** \file simpleclient.cpp
 * \author chq
 * \version 1.0
 * \date    2018
 *
 * \brief for testing send gotogoal
*/

#include <robotnik_pp_msgs/GoToAction.h>
#include <robotnik_pp_msgs/GoToActionFeedback.h>
#include <robotnik_pp_msgs/GoToGoal.h>
#include <robotnik_pp_msgs/GoToActionResult.h>
#include <robotnik_pp_msgs/GoToFeedback.h>
#include <robotnik_pp_msgs/GoToResult.h>
#include <robotnik_pp_msgs/goal.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<robotnik_pp_msgs::GoToAction> Client;
//这些消息类型名要与服务器发布的类型相对应
void doneCb(const actionlib::SimpleClientGoalState & state,
            const robotnik_pp_msgs::GoToResultConstPtr& result){
  ROS_INFO("yay!!!.done waypoints");
}
void actionActived(){
  ROS_INFO("server is connected suc!");
}
void feedbackCb(const robotnik_pp_msgs::GoToFeedbackConstPtr& feedback){
  ROS_INFO("percent comp perc:%.3f",feedback->percent_complete);
}

int main(int argc, char** argv)
{

  ///程序会以这个名字发布server,所以不要轻易改这个个名字
  ros::init(argc, argv, "robotnik_pp_planner_simple_client");
  ros::NodeHandle n;
  Client client("robotnik_pp_planner",true);
  ROS_INFO("PurepursuitClient waiting for server start...") ;
  client.waitForServer();
  ROS_INFO("PurepursuitClient watting for server suc!");
  robotnik_pp_msgs::GoToGoal gotogoal;
  robotnik_pp_msgs::goal _goal;

  //_goal.pose = geometry_msgs::Pose2D(1.772,-0.059,0);
  _goal.pose.x = 1.772;
  _goal.pose.y = -0.059;
  _goal.pose.theta = 0;
  _goal.speed = 0.3;
  gotogoal.target.push_back(_goal);

  //_goal.pose = geometry_msgs::Pose2D(7.787,-0.287,0);
  _goal.pose.x = 7.787;
  _goal.pose.y = -0.287;
  _goal.pose.theta = 0;
  _goal.speed = 0.3;
   gotogoal.target.push_back(_goal);

 //_goal.pose = geometry_msgs::Pose2D(15.862,-0.495,0);
 _goal.pose.x = 15.862;
 _goal.pose.y = -0.495;
 _goal.pose.theta = 0;
 _goal.speed = 0.3;
  gotogoal.target.push_back(_goal);

  //_goal.pose = geometry_msgs::Pose2D(23.407,-0.809,0);
  _goal.pose.x = 23.407;
  _goal.pose.y = -0.809;
  _goal.pose.theta = 0;
  _goal.speed = 0.3;
   gotogoal.target.push_back(_goal);

   //_goal.pose = geometry_msgs::Pose2D(23.727,5.374,0);
   _goal.pose.x = 23.727;
   _goal.pose.y = 5.374;
   _goal.pose.theta = 0;
   _goal.speed = 0.3;
    gotogoal.target.push_back(_goal);

 // _goal.pose = geometry_msgs::Pose2D(24.158,17.887,0);
  _goal.pose.x = 24.158;
  _goal.pose.y = 17.887;
  _goal.pose.theta = 0;
  _goal.speed = 0.3;
   gotogoal.target.push_back(_goal);

   client.sendGoal(gotogoal,&doneCb,&actionActived,&feedbackCb);

   ///stop proc
   ros::Duration(5).sleep();
   client.cancelGoal();
   ros::spin();
  return (0);
}
// EOF
