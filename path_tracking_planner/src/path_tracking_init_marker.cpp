
#include "path_tracking_planner.h"



//#include <s3000_laser/enable_disable.h>

namespace path_tracking_planner {

PathTrackingPlanner::PathTrackingPlanner(ros::NodeHandle h) :
    node_handle_(h),
    private_node_handle_("~"),
    desired_freq_(50.0),
    Component(desired_freq_),
    setup_(false),
    action_server_goto(node_handle_, "path_tracking_planner", false),
    g_dir(look_back),
    cnt_(0),
    ready_cnt_(0)
    // boost::bind(&purepursuit_planner_node::executeCB, this, _1), false)
{
    bRunning = false;

    ROSSetup();

    dLookAhead = d_lookahear_min_;
    dLinearSpeed = dAngulerSpeed = 0;
    pose2d_robot.x = pose2d_robot.y = pose2d_robot.theta = 0.0;
    bEnabled = true;
    bCancel = false;

    comp_firstpoint = false;
    need_repub_path = true;
    sComponentName.assign("path_tracking_planner_node");
    iState = INIT_STATE;
    direction = 0;




}


PathTrackingPlanner::~PathTrackingPlanner(){

}


void PathTrackingPlanner::ROSSetup()
{
    string s_command_type;

    private_node_handle_.param<std::string>("odom_topic", odom_topic_, "/wheel_diff_controller/odom");
    private_node_handle_.param("cmd_topic_vel", cmd_topic_vel_, std::string("/wheel_diff_controller/cmd_vel"));
    private_node_handle_.param("d_lookvertical_min", d_lookvertical_min_, D_LOOKVERTICAL_MIN);
    private_node_handle_.param("d_lookahear_min", d_lookahear_min_, D_LOOKAHEAD_MIN);
    private_node_handle_.param("look_ahead_inc", look_ahead_inc, D_LOOKAHEAD_INC);

    private_node_handle_.param("d_lookahear_max", d_lookahear_max_, D_LOOKAHEAD_MAX);
    private_node_handle_.param("d_dist_wheel_to_center", d_dist_wheel_to_center_, D_WHEEL_ROBOT_CENTER);
    private_node_handle_.param("max_speed", max_speed_, MAX_SPEED);
    private_node_handle_.param("kr", Kr, AGVS_DEFAULT_KR);
    private_node_handle_.param<std::string>("position_source", position_source_, "ODOM");
    private_node_handle_.param("desired_freq", desired_freq_, desired_freq_);
    private_node_handle_.param<std::string>("command_type", s_command_type, COMMAND_TWIST_STRING);
    private_node_handle_.param<double>("waypoint_pop_distance", waypoint_pop_distance_, WAYPOINT_POP_DISTANCE_M);

    private_node_handle_.param<std::string>("target_frame", target_frame_, "/base_link");

    private_node_handle_.param<std::string>("global_frame_id", global_frame_id_, "map");
    //PID
    private_node_handle_.param("max_w", maxT, 0.57);
    private_node_handle_.param("min_w", minT, -0.57);
    private_node_handle_.param("kp_w", Kp, 1.0);
    private_node_handle_.param("ki_w", Ki, 0.00);
    private_node_handle_.param("kd_w", Kd, 0.000);
    if(desired_freq_)
      dt = 1.0/desired_freq_;
    else
      dt = 0.1;
    private_node_handle_.param("max_v", maxS, 1.5);
    private_node_handle_.param("min_v", minS, 0.1);
    private_node_handle_.param("kp_v", KpS, 0.8);
    private_node_handle_.param("ki_v", KiS, 0.05);
    private_node_handle_.param("kd_v", KdS, 0.005);
    if(desired_freq_)
      dtS = 1.0/desired_freq_;
    else
      dtS = 0.1;
     pidTheta = new PID(dt, maxT, minT, Kp, Kd, Ki);
     pidVelocity = new PID(dtS, maxS, minS, KpS, KdS, KiS);
    //private_node_handle_.param<std::string>("name_sc_enable_frot_laser_", name_sc_enable_front_laser_, "/s3000_laser_front/enable_disable");
    //private_node_handl_.param<std::string>("name_sc_enable_back_laser", name_sc_enable_back_laser_, "/s3000_laser_back/enable_disable"  );

     private_node_handle_.param<string>("pcl_topic_front", pcl_topic_front_, "scan_cloud_front");
     private_node_handle_.param<string>("pcl_topic_back", pcl_topic_back_, "scan_cloud_back");

     private_node_handle_.param("footprint_width",footprint_width_,DEFAULT_FOOTPRINT_WIDTH);
     private_node_handle_.param("footprint_length",footprint_length_,DEFAULT_FOOTPRINT_LENGTH);
     private_node_handle_.param("lateral_clearance",nav_lateral_clearance_,DEFAULT_LATERAL_CLEARANCE);
     private_node_handle_.param("obstacle_range", nav_obstacle_range_, DEFAULT_OBSTACLE_RANGE);
     private_node_handle_.param("goal_tolerance", goal_tolerance_, WAYPOINT_POP_DISTANCE_M);
     private_node_handle_.param("goal_error_tolerance", goal_error_tolerance_, GOAL_ERROR_TOLERANCE); // Not used
     private_node_handle_.param("path_turn_radius_distance", path_turn_radius_distance_, AGVS_TURN_RADIUS);
     private_node_handle_.param("static_lookahead", static_lookahead_, true);
     private_node_handle_.param("obstacle_avoidance", obstacle_avoidance_, true);

     //private_node_handle_.param<std::string>("name_sc_enable_frot_laser_", name_sc_enable_front_laser_, "/s3000_laser_front/enable_disable");
     //private_node_handl_.param<std::string>("name_sc_enable_back_laser", name_sc_enable_back_laser_, "/s3000_laser_back/enable_disable"	);

     pcl_sub_front_ = private_node_handle_.subscribe(pcl_topic_front_, 10, &PathTrackingPlanner::pclCallbackFront, this);
     pcl_sub_back_ = private_node_handle_.subscribe(pcl_topic_back_, 10, &PathTrackingPlanner::pclCallbackBack, this);

    if(s_command_type.compare(COMMAND_ACKERMANN_STRING) == 0){
      command_type = COMMAND_ACKERMANN;
    }else if(s_command_type.compare(COMMAND_TWIST_STRING) == 0){
      command_type = COMMAND_TWIST;
    }else{
      // default value
      command_type = COMMAND_TWIST;
      d_dist_wheel_to_center_ = 1.0;
    }

    // From Component class
    threadData.dDesiredHz = desired_freq_;

    if(position_source_ == "MAP")
      ui_position_source = MAP_SOURCE;
    else
      ui_position_source = ODOM_SOURCE;
    ROS_INFO_STREAM("PathTracking.ui_position_source:"<<ui_position_source);//chq
    if(command_type == COMMAND_ACKERMANN){
      //
      // Publish through the node handle Ackerman type messages to the command vel topic
      vel_pub_ = private_node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>(cmd_topic_vel_, 100);
    }else{
      //
      // Publish through the node handle Twist type messages to the command vel topic
      vel_pub_ = private_node_handle_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
    }

    path_marker_pub = private_node_handle_.advertise<visualization_msgs::MarkerArray>("purepersuit_marker_path",100);
    tar_marker_pub = private_node_handle_.advertise<visualization_msgs::MarkerArray>("purepersuit_marker_target",100);

    path_pub_ = private_node_handle_.advertise<nav_msgs::Path>("purepersuit_path", 100);
    //
    if(ui_position_source == MAP_SOURCE)
      tranform_map_pub_ = private_node_handle_.advertise<geometry_msgs::TransformStamped>("map_location", 100);
    //status_pub_ = private_node_handle_.advertise<purepursuit_planner::ControllerStatus>("status", 1);
    odom_sub_ = private_node_handle_.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &PathTrackingPlanner::OdomCallback, this );
    //cmd_vel_sub_ = private_node_handle_.subscribe<purepursuit_planner::AckermannDriveStamped>("command", 1, &purepursuit_planner_node::CmdVelCallback, this );

    // Diagnostics
    updater_diagnostic.setHardwareID("PathTrackingPlanner");
    // Topics freq control
    min_odom_freq = 5.0;
    max_odom_freq = 100.0;
    updater_diagnostic_odom = new diagnostic_updater::TopicDiagnostic(odom_topic_, updater_diagnostic,
              diagnostic_updater::FrequencyStatusParam(&min_odom_freq, &max_odom_freq, 0.1, 10),
              diagnostic_updater::TimeStampStatusParam(0.001, 0.1));


    // Action server
    action_server_goto.registerGoalCallback(boost::bind(&PathTrackingPlanner::GoalCB, this));
    action_server_goto.registerPreemptCallback(boost::bind(&PathTrackingPlanner::PreemptCB, this));

        // Services
        //sc_enable_front_laser_ = private_node_handle_.serviceClient<s3000_laser::enable_disable>(name_sc_enable_front_laser_);
        //sc_enable_back_laser_ = private_node_handle_.serviceClient<s3000_laser::enable_disable>(name_sc_enable_back_laser_);
        
//    ROS_INFO("%s::ROSSetup(): odom_topic = %s, command_topic_vel = %s, position source = %s, desired_hz=%.1lf, min_lookahead = %.1lf, max_lookahead = %.1lf, kr = %.2lf, command_type = %s", sComponentName.c_str(), odom_topic_.c_str(),
//     cmd_topic_vel_.c_str(), position_source_.c_str(), desired_freq_, d_lookahear_min_, d_lookahear_max_, Kr, s_command_type.c_str());

    input_cloud_front_.reset(new  pcl::PointCloud<pcl::PointXYZ>());
    obstacle_cloud_front_.reset(new  pcl::PointCloud<pcl::PointXYZ>());
    input_cloud_back_.reset(new  pcl::PointCloud<pcl::PointXYZ>());
    obstacle_cloud_back_.reset(new  pcl::PointCloud<pcl::PointXYZ>());

    obstacle_range_=nav_obstacle_range_;
    lateral_clearance_=nav_lateral_clearance_;

    bObstacle=false;
    bObsFront = false;
    bObsBack = false;
    obs_x_low_=-footprint_length_/2;
    obs_x_high_= obstacle_range_;//+footprint_length_/2;
    obs_y_low_=-(footprint_width_/2+lateral_clearance_);
    obs_y_high_=(footprint_width_/2+lateral_clearance_);

    //pub_obs_front = private_node_handle_.advertise<sensor_msgs::PointCloud2> ("obstacle_front", 1);
    //pub_obs_back = private_node_handle_.advertise<sensor_msgs::PointCloud2> ("obstacle_back", 1);
    pub_objectDetected = private_node_handle_.advertise<wash_floor_msgs::ObjectDetected>("object_detection",100);




    //ROS_INFO("%s::ROSSetup(): laser_topics: front -> %s, back -> %s", sComponentName.c_str(), name_sc_enable_front_laser_.c_str(), name_sc_enable_back_laser_.c_str());


}

//-----------------------------------------------------------------------------
bool PathTrackingPlanner::init()
{
    ros::Time::waitForValid();

    std::string tf_error;
    ros::Time last_error_time= ros::Time::now();

    // we need to make sure that the transform between the robot base frame and the global frame is available
    while (ros::ok()
        && !listener.waitForTransform(global_frame_id_, target_frame_, ros::Time(), ros::Duration(0.5), ros::Duration(0.01),
                                 &tf_error))
    {
      ros::spinOnce();

      ros::Time current_error = ros::Time::now();
      ROS_DEBUG_STREAM("last time : " << last_error_time  << ", current time : " << current_error);

      if ( (last_error_time + ros::Duration(10.0)) < current_error)
      {
        ROS_WARN("%s::init timed out waiting for transform from %s to %s to become available before running planner, tf error: %s",
                sComponentName.c_str(), target_frame_.c_str(), global_frame_id_.c_str(), tf_error.c_str());
//        last_error = ros::Time::now();

        return false;
      }
      // The error string will accumulate and errors will typically be the same, so the last
      // will do for the warning above. Reset the string here to avoid accumulation.
      tf_error.clear();
    }


    ///
    /// \brief reconfig
    ///
    dsrv_ = new dynamic_reconfigure::Server<path_tracking_planner::PathTrackingPlannerConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<path_tracking_planner::PathTrackingPlannerConfig>::CallbackType cb = boost::bind(&PathTrackingPlanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    ROS_DEBUG_STREAM("path_tracking_planner_node init done! this_node name : " << ros::this_node::getName());

    last_command_time = ros::Time::now();

    return true;
}

void PathTrackingPlanner::reconfigureCB(path_tracking_planner::PathTrackingPlannerConfig &config, uint32_t level)
{
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

    if(!setup_)
    {
      default_config_ = config;

      setup_ = true;
      return;
    }

    maxT = config.max_w;
    minT = config.min_w;
    Kp = config.kp_w;
    Ki = config.ki_w;
    Kd = config.kd_w;

    look_ahead_inc = config.look_ahead_inc;
    d_lookahear_min_ = config.d_lookahear_min;
    d_lookahear_max_ = config.d_lookahear_max;
    max_speed_ = config.max_speed;
    desired_freq_ = config.desired_freq;

    ROS_INFO("PathTrackingPlanner.reconfigureCB.now reconfig rotate speed pid para:pid_kp:%.3f,pid_ki:%.3f,pid_kd:%.3f ",Kp,Ki,Kd);
    ROS_INFO("PathTrackingPlanner.reconfigureCB.now reconfig trajectory track para:look_ahead_inc: %.3f,d_lookahear_min_: %.3f,d_lookahear_max_: %.3f,max_speed_: %.3f, desired_freq_: %.3f."
             ,look_ahead_inc, d_lookahear_min_, d_lookahear_max_, max_speed_, desired_freq_);

}


void PathTrackingPlanner::pclCallbackFront(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg)
{

//  ROS_INFO("[PathTrackingPlanner::pclCallbackFront] begin!!!");
//  ROS_INFO("Get an obstacle cloud of %u points",(uint32_t)(obstacle_cloud_->size()));

  if(direction > 0)
  {

    pcl::PCLPointCloud2 temp_pcl_;

    pcl_conversions::toPCL(*pcl_msg, temp_pcl_);

    pcl::fromPCLPointCloud2(temp_pcl_,*input_cloud_front_);
    //ROS_INFO("Received a tunnel cloud of %u points",(uint32_t)(input_cloud_->size()));

    header_= pcl_msg->header;

    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud (input_cloud_front_);

    pass.setFilterFieldName ("x");
    pass.setFilterLimits (obs_x_low_,obs_x_high_);
    pass.filter (*obstacle_cloud_front_);

    pass.setInputCloud (obstacle_cloud_front_);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (obs_y_low_,obs_y_high_);
    pass.filter (*obstacle_cloud_front_);

    bObstacle = (obstacle_cloud_front_->points.size()>0) ? true : false;
    bObsFront = bObstacle;

  }
  else if(direction == 0)
  {
      bObstacle = false;
      bObsFront = bObstacle;
  }
  //pub_obs_front.publish(obstacle_cloud_front_);

//  ROS_INFO("[PathTrackingPlanner::pclCallbackFront] end!!!");
}

void PathTrackingPlanner::pclCallbackBack(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg)
{


  //ROS_INFO("Get an obstacle cloud of %u points",(uint32_t)(obstacle_cloud_->size()));
  if(direction < 0){
    pcl::PCLPointCloud2 temp_pcl_;
    pcl_conversions::toPCL(*pcl_msg, temp_pcl_);
    pcl::fromPCLPointCloud2(temp_pcl_,*input_cloud_back_);
    //ROS_INFO("Received a tunnel cloud of %u points",(uint32_t)(input_cloud_->size()));

    header_ = pcl_msg->header;

    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud (input_cloud_back_);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-obs_x_high_,-obs_x_low_);
    pass.filter (*obstacle_cloud_back_);

    pass.setInputCloud (obstacle_cloud_back_);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (obs_y_low_,obs_y_high_);
    pass.filter (*obstacle_cloud_back_);
    bObstacle = (obstacle_cloud_back_->points.size()>0) ? true : false;
    bObsBack = bObstacle;
  }
  else if(direction == 0){
      bObstacle = false;
      bObsBack = bObstacle;
  }

  //pub_obs_back.publish(obstacle_cloud_back_);

}


/*! \fn int ReadAndPublish()
 * Reads data a publish several info into different topics
*/

int PathTrackingPlanner::ReadAndPublish()
{
    objDet.front = bObsFront;
    objDet.back = bObsBack;
    pub_objectDetected.publish(objDet);

    //updater_diagnostic_odom->tick(ros::Time::now());
    updater_diagnostic.update();

    return(0);
}

void PathTrackingPlanner::pubMarkerRfs(std::vector<Waypoint> new_points){
  visualization_msgs::Marker points_marker;
  visualization_msgs::Marker text_marker;
  nav_msgs::Path path_points;
  visualization_msgs::MarkerArray marker_array;
  points_marker.header.stamp = ros::Time::now();
  path_points.header.stamp = points_marker.header.stamp;
  std::string frame;
  if(position_source_ == "MAP")
    frame = "/map";
  else
    frame = "/odom";
  points_marker.header.frame_id = frame;
  path_points.header.frame_id = points_marker.header.frame_id;

  points_marker.type = visualization_msgs::Marker::POINTS;
  points_marker.ns = "map_nmspace";
  points_marker.action = visualization_msgs::Marker::ADD;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  points_marker.scale.x = 0.1;
  points_marker.scale.y = 0.1;
  points_marker.scale.z = 0.1;
  // Set the color -- be sure to set alpha to something non-zero!
  //DarkOrchid	153 50 204
  points_marker.color.r = 153;
  points_marker.color.g = 50;
  points_marker.color.b = 204;
  points_marker.color.a = 0.3;
  points_marker.lifetime = ros::Duration(100000);
  geometry_msgs::Point p;
  geometry_msgs::Pose pose;
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = path_points.header.frame_id;
  if(new_points.size() )
  {
    for(int i=0;i < new_points.size();i++)
    {
      points_marker.id = i;
      //在相对于激光头的坐标空间
      p.x = new_points[i].dX;
      p.y = new_points[i].dY;
      p.z = 0;
      points_marker.points.push_back(p);

      pose.position.x = p.x;
      pose.position.y = p.y;
      pose.position.z = 0;
      pose_stamped.pose.position.x = p.x ;
      pose_stamped.pose.position.y = p.y ;
      pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(new_points[i].dA);
      path_points.poses.push_back(pose_stamped);

      ////fort text type,Only scale.z is used. scale.z specifies the height of an uppercase "A".
      text_marker = points_marker;
      text_marker.ns = "purepersuit_text_space";
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.color.r = 255;
      text_marker.color.g = 0;
      text_marker.color.b = 0;
      text_marker.color.a = 1;
      text_marker.scale.z = 0.1;
      text_marker.id = i;
      stringstream ss ;
      ss << i;
      string str;
      ss >> str;
      text_marker.text = str ;
      text_marker.pose = pose;
      marker_array.markers.push_back(text_marker);

    }
    marker_array.markers.push_back(points_marker);
    marker_array.markers.push_back(text_marker);
    path_marker_pub.publish(marker_array);
    path_pub_.publish(path_points);
  }


}

void PathTrackingPlanner::pubTarget(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D next_position,double w)
{
    visualization_msgs::Marker points_marker;
    visualization_msgs::Marker dir_marker;
    visualization_msgs::MarkerArray marker_array;
    points_marker.header.stamp = ros::Time::now();
    dir_marker.header.stamp = points_marker.header.stamp;
    std::string frame;
    if(position_source_ == "MAP")
    frame = "/map";
    else
    frame = "/odom";
    points_marker.header.frame_id = frame;
    dir_marker.header.frame_id = points_marker.header.frame_id;

    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.ns = "purepersuit_space";
    points_marker.action = visualization_msgs::Marker::ADD;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    points_marker.scale.x = 0.2;
    points_marker.scale.y = 0.2;
    points_marker.scale.z = 0.2;
    // Set the color -- be sure to set alpha to something non-zero!
    //DarkOrchid	153 50 204

    if( g_dir == look_back){
    points_marker.color.r = 0.8;
    points_marker.color.g = 0;
    points_marker.color.b = 0;
    points_marker.color.a = 0.5;
    }
    else{
    points_marker.color.r = 0.0;
    points_marker.color.g = 0.8;
    points_marker.color.b = 0.0;
    points_marker.color.a = 0.5;
    }

    points_marker.lifetime = ros::Duration(100000);
    points_marker.id = 0;
    geometry_msgs::Point p;
    p.x = next_position.x;
    p.y = next_position.y;
    p.z = 1;
    points_marker.points.push_back(p);


    dir_marker.type = visualization_msgs::Marker::ARROW;
    dir_marker.ns = "purepersuit_space";
    dir_marker.action = visualization_msgs::Marker::ADD;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    //if is arrow scale.x is the shaft diameter, and scale.y is the head diameter.
    //If scale.z is not zero, it specifies the head length.
    dir_marker.scale.x = 0.1;
    dir_marker.scale.y = 0.1;
    dir_marker.scale.z = 0.05;
    // Set the color -- be sure to set alpha to something non-zero!
    //DarkOrchid	153 50 204
    dir_marker.color.r = 153;
    dir_marker.color.g = 50;
    dir_marker.color.b = 204;
    dir_marker.color.a = 0.3;
    dir_marker.lifetime = ros::Duration(100000);
    ///do not euqal to last points_marker id!!!
    dir_marker.id = 1;

    p.x = current_position.x;
    p.y = current_position.y;
    p.z = 0;
    //The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end.
    dir_marker.points.push_back(p);

    float len = 0.5;//arrow len
    float abs_angle = current_position.theta+w;
    p.x = current_position.x+cos(abs_angle)*len;
    p.y = current_position.y+sin(abs_angle)*len;
    p.z = 0;
    dir_marker.points.push_back(p);

    marker_array.markers.push_back(points_marker);
    marker_array.markers.push_back(dir_marker);
    tar_marker_pub.publish(marker_array);

}

bool PathTrackingPlanner::SetLaserFront()
{
    /*s3000_laser::enable_disable srv;

    srv.request.value = false;
    sc_enable_back_laser_.call(srv);
    ROS_INFO("%s::SetLaserFront: Setting laser back to false, ret = %d", sComponentName.c_str(), srv.response.ret);

    srv.request.value = true;
    sc_enable_front_laser_.call(srv);
    ROS_INFO("%s::SetLaserFront: Setting laser front to true, ret = %d", sComponentName.c_str(), srv.response.ret);*/
}

bool PathTrackingPlanner::SetLaserBack()
{
    /*s3000_laser::enable_disable srv;

    srv.request.value = false;
    sc_enable_front_laser_.call(srv);
    ROS_INFO("%s::SetLaserBack: Setting laser front to false, ret = %d", sComponentName.c_str(), srv.response.ret);

    srv.request.value = true;
    sc_enable_back_laser_.call(srv);
    ROS_INFO("%s::SetLaserBack: Setting laser back to true, ret = %d", sComponentName.c_str(), srv.response.ret);*/
}


}


