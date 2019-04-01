#ifndef PATH_TRACKING_PLANNER_H_
#define PATH_TRACKING_PLANNER_H_

#include <string.h>
#include <vector>
#include <queue>
#include <stdint.h>//<stdint.h>中定义了几种扩展的整数类型和宏
#include <ros/ros.h>
#include <math.h>
#include <cstdlib>

#include <ackermann_msgs/AckermannDriveStamped.h>//disable by chq
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
#include <actionlib/server/simple_action_server.h>
#include <wash_floor_msgs/GoToAction.h>
#include <wash_floor_msgs/goal.h>
#include <wash_floor_msgs/ObjectDetected.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>///added for display path
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <dynamic_reconfigure/server.h>
#include <path_tracking_planner/PathTrackingPlannerConfig.h>

#include "pid.h"
//#include "Geometry.h"
#include "path_algorithm.h"

//#include "Component.h"
//#include "path_tracking_comm.h"
//#define ODOM_TIMEOUT_ERROR					0.2				// max num. of seconds without receiving odom values
//#define MAP_TIMEOUT_ERROR					0.2				// max num. of seconds without receiving map transformations
//#define AGVS_TURN_RADIUS					0.20			// distancia en la que empieza a girar el robot cuando llega a una esquina
//#define MIN_ANGLE_BEZIER					0.261799388		// ángulo (radianes) mínimo entre segmentos de la recta para los que ajustaremos a una curva de BEZIER
//#define BEZIER_CONTROL_POINTS				5

//#define D_LOOKAHEAD_MIN						0.3		// Minima distancia del punto objetivo en m (PurePursuit con lookahead dinámico)
//#define D_LOOKAHEAD_MAX						1.1		// Maxima distancia del punto objetivo
//#define D_WHEEL_ROBOT_CENTER    			0.478   // Distance from the motor wheel to the robot center

//#define MAX_TURN_SPEED_LVL1					0.5
//#define MAX_TURN_SPEED_LVL2					0.2
//#define MAX_SPEED							1.5

//#define WAYPOINT_POP_DISTANCE_M				0.10		//Distancia mínima para alcanzar punto objetivo m (PurePursuit)

//#define AGVS_FIRST_DECELERATION_DISTANCE 	2.0 	// meters -> when the vehicle is arriving to the goal, it has to decelarate at this distance
//#define AGVS_FIRST_DECELERATION_MAXSPEED	0.3     // m/s
//#define AGVS_SECOND_DECELERATION_DISTANCE   0.5 	// meters -> when the vehicle is arriving to the goal, it has to decelarate another time at this distance
//#define AGVS_SECOND_DECELERATION_MAXSPEED	0.1 	// m/s
//#define AGVS_DEFAULT_KR	0.20						//


//#define COMMAND_ACKERMANN					100
//#define COMMAND_TWIST						200
//#define COMMAND_ACKERMANN_STRING			"Ackermann"
//#define COMMAND_TWIST_STRING				"Twist"

//#define DEFAULT_OBSTACLE_RANGE 				1.0
//#define DEFAULT_FOOTPRINT_WIDTH 			0.6
//#define DEFAULT_FOOTPRINT_LENGTH 			1.0
//#define DEFAULT_LATERAL_CLEARANCE 			0.5


//#define GOAL_ERROR_TOLERANCE				0.05
//#define ERROR_GOAL_DISTANCE					0.10

namespace path_tracking_planner {


class PathTrackingPlanner: public Component
{

private:
    void pclCallbackFront(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg);
    void pclCallbackBack(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg);


    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;
    double desired_freq_;
    //! constant for Purepursuit
    double Kr;

    //! Variable lookahead
    double dLookAhead;
     //! Object with the current path that the robot is following
    Path pathCurrent;
    //! Object with the path that is being filled
    Path pathFilling;
    //! Vector with next paths to follow
    queue <Path> qPath;
    //! current robot's position
    geometry_msgs::Pose2D pose2d_robot;
    //! current robot's odometry
    nav_msgs::Odometry odometry_robot;
    //! current robot's linear speed
    double dLinearSpeed, dAngulerSpeed;
    //! Lookahead bounds
    double look_ahead_inc,d_lookahear_min_, d_lookahear_max_;
    double d_lookvertical_min_;//大于这个距离，瞄向垂点
    //! Distance from the robot center to the wheel's center
    double d_dist_wheel_to_center_;
    //! Max allowed speed
    double max_speed_;
    //! Flag to enable/disable the motion
    bool bEnabled;
    //! Flag to cancel the following path
    bool bCancel;
    //! Mode for reading the position of the robot ("ODOM", "MAP")
    std::string position_source_;
    //! Target frame for the transform from /map to it
    std::string target_frame_;
    std::string global_frame_id_;
    //! Mode in numeric format
    unsigned int ui_position_source;
    //!  Sets the type of command to send to the robot (Twist or ackermann)
    int command_type;
    //! Direction of movement (-1 or +1)
    int direction;

    //!Flag variable to memorize if obstacles were detected
    bool bObstacle;
    bool bObsFront;
    bool bObsBack;

    double obs_x_low_, obs_x_high_,obs_y_low_,obs_y_high_;
    double nav_lateral_clearance_;
    double lateral_clearance_;
    double footprint_width_;
    double footprint_length_;
    double nav_obstacle_range_;
    double obstacle_range_;

    //!Input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_front_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_back_;
    //!Obstacle cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_front_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_back_;
    //!PointCloud2 header
    std_msgs::Header header_;

    //! Topic name to read the point cloud from
    std::string pcl_topic_front_;
    std::string pcl_topic_back_;

    ros::Subscriber pcl_sub_front_;
    ros::Subscriber pcl_sub_back_;

    //! goal tolerance
    double goal_tolerance_;
    double last_dist_to_goal;
    //!
    bool static_lookahead_;
    //! error tolerance
    double goal_error_tolerance_;
    //!	Distance from the point to start to turn
    double path_turn_radius_distance_;
    //! Flag to enable / disable the obstacle avoidance
    bool obstacle_avoidance_;
    //! flag ok if transform between frames is OK
    bool transform_ok;

    bool need_repub_path;
    bool comp_firstpoint;
    ///PID
    double dt , maxT , minT, Kp , Ki, Kd;
    double dtS, maxS , minS, KpS, KiS, KdS;
    PID* pidTheta ;
    PID* pidVelocity;

    //reconfigure
    bool setup_;
    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<path_tracking_planner::PathTrackingPlannerConfig> *dsrv_;
    // void reconfigureCB(robotnik_pp_planner::robotnik_pp_plannerConfig &config, uint32_t level);
    path_tracking_planner::PathTrackingPlannerConfig default_config_;


    //////// ROS
    //! pub the path points chq
    ros::Publisher path_pub_;
    ros::Publisher path_marker_pub;
    ros::Publisher tar_marker_pub;

    //! Publishes the status of the robot
    ros::Publisher status_pub_;
    ros::Publisher pub_obs_front;
    ros::Publisher pub_obs_back;
    ros::Publisher pub_objectDetected;

    //! Publish to cmd vel (Ackermann)
    //! It will publish into command velocity (for the robot)
    ros::Publisher vel_pub_;
    //! publish the transformation between map->base_link
    ros::Publisher tranform_map_pub_;
    //! it subscribes to /odom
    ros::Subscriber odom_sub_;
    //! Topic name to read the odometry from
    std::string odom_topic_;
    //! Topic name to publish the vel & pos commands
    std::string cmd_topic_vel_;
    // DIAGNOSTICS
    //! Diagnostic to control the frequency of the published odom
    diagnostic_updater::TopicDiagnostic *updater_diagnostic_odom;
    //! General status diagnostic updater
    diagnostic_updater::Updater updater_diagnostic;
    //! Diagnostics min & max odometry freq
    double min_odom_freq, max_odom_freq;
    //! Saves the time whenever receives a command msg and a transform between map and base link (if configured)
    ros::Time last_command_time, last_map_time;
    // ACTIONLIB
    actionlib::SimpleActionServer<wash_floor_msgs::GoToAction> action_server_goto;
    wash_floor_msgs::GoToFeedback goto_feedback;
    wash_floor_msgs::GoToResult goto_result;
    wash_floor_msgs::GoToGoal goto_goal;

    wash_floor_msgs::ObjectDetected objDet;
    // TFs
    tf::TransformListener listener;
    tf::StampedTransform transform;
    // SERVICES
    //! service name to enable disable lasers
    std::string name_sc_enable_front_laser_, name_sc_enable_back_laser_;
    //! Service to enable/disable front laser
    ros::ServiceClient sc_enable_front_laser_;
    //! Service to enable/disable back laser
    ros::ServiceClient sc_enable_back_laser_;

    look_dir g_dir;
    int cnt_,ready_cnt_;
    double waypoint_pop_distance_;
    boost::mutex dAnguler_mutex_;

public:

  /*!  \fn summit_controller::purepursuit_planner()
   *   \brief Public constructor
  */
  PathTrackingPlanner(ros::NodeHandle h);

  bool init();

  /*!  \fn purepursuit_planner::~purepursuit_planner()
   *   \brief Public destructor
  */
  virtual ~PathTrackingPlanner();

  /*!  \fn oid ROSSetup()
   *   \brief Setups ROS' stuff
  */
  void ROSSetup();

  /*!  \fn reconfigureCB(AMCLConfig &config, uint32_t level)
   *   \brief
  */
  void reconfigureCB(path_tracking_planner::PathTrackingPlannerConfig &config, uint32_t level);

  /*!  \fn ReturnValue Setup()
   *   \brief
  */
  ReturnValue Setup();

  /*! \fn int ReadAndPublish()
   * Reads data a publish several info into different topics
  */
  int ReadAndPublish();

  /*! \fn void pubMarkerRfs()
   * Pub the path to rviz
   * @author chq
  */
  void pubMarkerRfs(std::vector<Waypoint> new_points);

  /*!  \fn ReturnValue Start()
   *   \brief Start Controller
  */
  ReturnValue Start();

  /*!  \fn ReturnValue Stop()
   *   \brief Stop Controller
  */
  ReturnValue Stop();

  /*! \fn void ControlThread()
  */
  void ControlThread();

  /*!  \fn void InitState()
  */
  void InitState();

  /*!  \fn void StandbyState()
  */
  void StandbyState();

  /*!  \fn void ReadyState()
  */
  void ReadyState();

  /*! \fn void UpdateLookAhead()
  *   \brief Updates (little by little) the variable lookahead depending of the current velocity
  */
  void UpdateLookAhead();

  /*! \fn double Dot2( double x1, double y1, double x2, double y2)
  *   \brief Obtains vector cross product w x v
  *   \return w.x * v.x + w.y * w.y
  */
  double Dot2( double x1, double y1, double x2, double y2);


  /*! \fn double Dist(double x1, double y1, double x2, double y2)
  *   \brief obtains distance between points p1 and p2
  */
  double Dist(double x1, double y1, double x2, double y2);

  /*! \fn double DistP2S( Odometry current_position, Waypoint s0, Waypoint s1, Waypoint *Pb)
   *  \brief obtains distance between the current position and segment s0->s1, and returns the point
   *  Return: the shortest distance from p to s (utm points) and the point
   *  of the segment that gives the shortest distance
  */
  double DistP2S( geometry_msgs::Pose2D current_position, Waypoint s0, Waypoint s1, Waypoint *Pb);

  ///从第一个点逐一执行路径而不跳跃执行
  ReturnValue PointOneByOne(geometry_msgs::Pose2D& current_position, geometry_msgs::Pose2D *wp);

  /*! \fn ReturnValue PointDlh(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp  )
   *  \brief Returns a point in a distance dlookahead on the path
   *  \brief 0,跟踪的不是给定的目标点　而是由给定的路径点构成的线段上的动态形成点，由lookahead推动点移动，这样跟踪更平滑!!!
   *  \brief 1,找到距离当前位置最近的线段索引k，并设为最新的跟踪索引
   *  \brief 2,沿着索引k线段向前累计线段距离，直到加到索引k'线段(长度l)，sum距离为s,超过dLookAhead
   *  \brief 3,last跟踪ｋ’上的哪一点，取决于k'长度l在s中，超过dLookAhead部分所占的比重，比重逐渐增大，跟踪点逐渐由k'起点趋向于k'末尾点
   *  \return OK
   *  \return ERROR
  */
  ReturnValue PointDlh(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp);

  /*!  \fn void pubTarget()
   * \brief visualize the target and the rotation speed for debugging
   * \param current_position cur robot pos
   * \param next_position target pos
   * \param w rotate angle(speed)
   */
  void pubTarget(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D next_position,double w);

  /*!  \fn int PurePursuit()
   * \brief High level control loop in cartesian coordinates
   * obtains desiredSpeedMps and desiredPhiEffort according to
   * the robot location and the path defined by the waypoints
   *  \return 0 if the iteration is OK
   *  \return -1 if there's a problem
   *  \return 1 if the route finishes
   */
  int PurePursuit();

  /*!  \fn void CancelPath()
   * Removes all the waypoints introduced in the system
  */
  void CancelPath();


  /*!  \fn void SetRobotSpeed()
  */
  void SetRobotSpeed(double speed, double angle);

  /*!  \fn void ShutDownState()
  */
  void ShutDownState();

  /*!  \fn void EmergencyState()
  */
  void EmergencyState();

  /*!  \fn void FailureState()
  */
  void FailureState();

  /*!  \fn void AllState()
  */
  void AllState();

  /*! \fn void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
    * Receives odom values
  */
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value);

  /*! \fn int CheckOdomReceive()
    * Checks whether or not it's receiving odom values and/or map transformations
    * \return 0 if OK
    * \return -1 if ERROR
  */
  int CheckOdomReceive();

  void executeCB(const wash_floor_msgs::GoToGoalConstPtr &goal);

  /*! \fn void GoalCB()
    * Called when receiving a new target. (ActionServer)
  */
  void GoalCB();

  /*! \fn void PreemptCB()
    * Called to cancel or replace current mision. (ActionServer)
  */
  void PreemptCB();

  /*! \fn void AnalyseCB()
    * Checks the status. (ActionServer)
  */
  void AnalyseCB();

  /*! \fn void SetLaserFront()
    * Disables laser back, enables laser front
  */
  bool SetLaserFront();

  /*! \fn void SetLaserBack()
    * Disables laser front, enables laser back
  */
  bool SetLaserBack();

  /*! \fn int CalculateDirectionSpeed(Waypoint target_position)
  *  \brief Calcula el sentido de movimiento de una ruta, dependiendo de la posición inicial y el ángulo del robot
  *  \return 1 si el sentido es positivo
  *  \return -1 si el sentido es negativo
  */
  int CalculateDirectionSpeed(Waypoint target_position);

  /*! \fn int CalculateDirectionSpeed(geometry_msgs::Pose2D target_position)
  *  \brief Calcula el sentido de movimiento de una ruta, dependiendo de la posición inicial y el ángulo del robot
  * 根据机器人的初始位置和角度计算路线的移动方向
  *  \return 1 si el sentido es positivo
  *  \return -1 si el sentido es negativo
  */
  int CalculateDirectionSpeed(geometry_msgs::Pose2D target_position);

  /*!  \fn ReturnValue Agvs::MergePath()
   *   \brief Merges the current path with the next path
  */
  ReturnValue MergePath();



};

}


#endif  /*  PATH_TRACKING_PLANNER_H_  */
