/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "amcl_optimize/map/map.h"
#include "amcl_optimize/pf/pf.h"
#include "amcl_optimize/sensors/amcl_odom.h"
#include "amcl_optimize/sensors/amcl_laser.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl_optimize/AMCL_OPTIMIZEConfig.h"

// Allows AMCL to run from bag file
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

 #include<stdlib.h>//chq add for launching dump to file to save last pos 
#include <std_msgs/Int16.h>//chq gor lobal loc
#include "GALGO_Lib/Galgo.hpp"


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>

#define NEW_UNIFORM_SAMPLING 1
 using namespace message_filters;
using namespace amcl_optimize;
///GA For InIt POS Optimize
//objective class example
template <typename T>
class InitLocObjective
{
public:
   // objective function example : Rosenbrock function
   // minimizing f(x,y) = (1 - x)^2 + 100 * (y - x^2)^2
#if 0
   static std::vector<T> Objective(const std::vector<T>& x)
   {

     pf_vector_t pose;
     pose.v[0] = x[0];
     pose.v[1] = x[1];
     pose.v[2] = x[2];
     T obj = evaluateOnePose(pose);
     //ROS_INFO("GA.Obj.pose:(%.6f,%.6f,%.6f) objv:%.6f",pose.v[0],pose.v[1],pose.v[2],obj);
     //// NB: GALGO maximize by default
     return {obj};
   }

   static T evaluateOnePose(pf_vector_t pose){
     int i, j, step;
     double obs_range, obs_bearing;
     T p;
     double z, pz;
     double worst =  -999999;
     pf_vector_t hit;
     pf_vector_t map2base = pose;
     // Take account of the laser pose relative to the robot
     //map2base * base2laser = map2laser.
     pose = pf_vector_coord_add(laser_pose, map2base);///相对激光头[head]位置转到绝对激光位置
     //printf("AMCLLaser::evaluateOnePose.map2base:(%.6f,%.6f,%.6f).base2laser:(%.6f,%.6f,%.6f).map2laser:(%.6f,%.6f,%.6f)\n",
     //    map2base.v[0],map2base.v[1],map2base.v[2],
     //    self->laser_pose.v[0],self->laser_pose.v[1],self->laser_pose.v[2],
     //    pose.v[0],pose.v[1],pose.v[2]) ;
    p = 0;


     step = (data.range_count - 1) / (max_beams - 1);

     // Step size must be at least 1
     if(step < 1)
       step = 1;
     // 开始通过利用与最近物体的欧氏距离计算激光模型似然的算法，
     // 对所有特征（激光数据）进行遍历
     //printf("evaluate one scan.scan size:%d,step:%d\n",data->range_count,step);
     ///!!!step 间隔过大　不利于收敛（大部分数据没用上），故这里设为１(but influence speed)
     static bool cnt = 0;
     if(!cnt){
       printf("evaluate one scan.data->range_max:%.3f\n",data.range_max);
       cnt = true;
     }

     double max_err_bound_thread  = 0.05;
     double min_err_bound_thread  = 0.05;
     int mi, mj;
     int use_beams_cnt = 0;
     double sum_temp = 0.0;
     for (i = 0; i < data.range_count; i += step)
     {
       //printf("AMCLLaser::evaluateOnePose.loop start \n");
       obs_range = data.ranges[i][0];
       obs_bearing = data.ranges[i][1];
       // This model ignores max range readings
       // 似然域测量模型简单地将最大距离读数丢弃
       if(obs_range >= data.range_max - max_err_bound_thread || obs_range <= min_err_bound_thread  )
         continue;

       // Check for NaN
       if(obs_range != obs_range)
         continue;
       use_beams_cnt++;
       hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);///相对激光点[endpoint]位置转到绝对激光点位置
       hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

       mi = MAP_GXWX(map, hit.v[0]);
       mj = MAP_GYWY(map, hit.v[1]);

       // Part 1: Get distance from the hit to closest obstacle.
       // Off-map penalized as max distance
       if(!MAP_VALID(map, mi, mj)){
        // printf("AMCLLaser::evaluateOnePose.obs cord outbound.use min prob value\n");
         z = map->max_occ_dist;
       }
       else{
        // printf("AMCLLaser::evaluateOnePose.obs cord outbound.use prob value\n");
         z =map->cells[MAP_INDEX(map,mi,mj)].occ_dist;
       }
       if(z == 0.0){
          printf("oh.my god\n");
       }
       sum_temp+=z;
       p += -z;

     }
   //  if(p >= 0 ){
   //    printf("hyoou!\n");
   //  }
     //printf("ga.evalute use_beams_cnt:%d\n",use_beams_cnt);
     if(sum_temp <= 0)
       printf("oh.my god\n");
     return p;
   }

   //static AMCLLaser *laser;

   static AMCLLaserData  data;

   static pf_vector_t laser_pose;
   static int max_beams;
   static map_t* map;

#endif
   static std::vector<T> Objective(const std::vector<T>& x)
   {
//     laser = (AMCLLaser*) data.sensor;
     pf_vector_t pose;
     pose.v[0] = x[0];
     pose.v[1] = x[1];
     pose.v[2] = x[2];
     T obj = laser->evaluateOnePose(data,pose);
     //ROS_INFO("GA.Obj.pose:(%.6f,%.6f,%.6f) objv:%.6f",pose.v[0],pose.v[1],pose.v[2],obj);
     //// NB: GALGO maximize by default
      return {obj};
   }
   static AMCLLaser *laser;
   static AMCLLaserData  *data;
private:


};

///!!!!!!!!!!!!!!!!!we must init it
template<typename T>
AMCLLaser* InitLocObjective<T>::laser = NULL;

template<typename T>
AMCLLaserData* InitLocObjective<T>::data=NULL;
#if 0
template<typename T>
pf_vector_t InitLocObjective<T>::laser_pose;

template<typename T>
int InitLocObjective<T>::max_beams;

template<typename T>
map_t* InitLocObjective<T>::map=NULL;
#endif

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

static const std::string scan_topic_ = "/scan";

class AmclNode
{
  public:
    AmclNode();
    ~AmclNode();

    /**
     * @brief Uses TF and LaserScan messages from bag file to drive AMCL instead
     */
    void runFromBag(const std::string &in_bag_fn);

    int process();
    void savePoseToServer();

  private:
    tf::TransformBroadcaster* tfb_;

    // Use a child class to get access to tf2::Buffer class inside of tf_
    struct TransformListenerWrapper : public tf::TransformListener
    {
      inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
    };

    TransformListenerWrapper* tf_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
    static pf_vector_t limitUniformPoseGenerator(void* pf,void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif
    // Callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    bool nomotionUpdateCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    bool setMapCallback(nav_msgs::SetMap::Request& req,
                        nav_msgs::SetMap::Response& res);

    int tailoringScan(const sensor_msgs::LaserScanConstPtr& laser_scan,AMCLSensor *sensor,AMCLLaserData& ldata);
    int updateLaserVector(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void pubPoseAndTF(const sensor_msgs::LaserScanConstPtr& laser_scan,pf_vector_t bestPos,double best_weight_perc);
    bool judgeUpdateByMove(pf_vector_t pose,pf_vector_t old_pos,pf_vector_t& delta);

    void resetAndForceUpdate(pf_vector_t pose);
    void updateDeadRecking(pf_vector_t pose,pf_vector_t delta);
    void pubPfCloud(void);
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void globalPoseReceived(const std_msgs::Int16ConstPtr & msg);

    void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    void updatePoseFromServer();
    void applyInitialPose();
    void applyInitialPose(AMCLLaser* lasers_amcl, AMCLLaserData& ldata,pf_vector_t pos);

    double getYaw(tf::Pose& t);

    //for global search pos chq
    void LocalLocByGaAlogrithm(double search_radius, pf_vector_t init_pos);
    void LocalLocBySplitMap(double search_w,double search_h,double init_x,double init_y);
    void globalLocBySplitMap(void);
    void uniformSplitSubMap(pf_t *pf,std::vector<std::pair<double,double> > & mid_pos_set,
                    double &search_width,double &search_height, double init_x, double init_y);
    void uniformSplitMap(pf_t *pf,std::vector<std::pair<double,double> > & mid_pos_set,double &search_width,double &search_height);
    //按照给定的单位面积内采样粒子数　生成随机粒子
    //int generatePfBySampleDensity(pf_vector_t* out_pf_set,map_t* map,int den,std::pair<double,double> mid_pos);
    int generateRandomPfSet(pf_vector_t* out_pf_set,int gennum, std::pair<double,double>mid_pos,double search_width,double search_height);
    void updatePfByLaser(AMCLLaser* laser,AMCLLaserData& ldata );
    void evaluatePfByLaser(pf_t *pf,AMCLLaser* laser,AMCLLaserData& ldata);
    int getBestSample(pf_t *pf,pf_vector_t& best_pose,double& best_weight);
    int getBestClusterSample(pf_t *pf,pf_vector_t& best_pose,double &best_weight_perc);


    //parameter for what odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;

    //boost::shared_ptr<message_filters::Subscriber<std_msgs::Int16 > > loc_sub_;
    //boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> > laser_sub_;
    //定义同步成员变量：
    //typedef message_filters::sync_policies::ApproximateTime<std_msgs::Int16,sensor_msgs::LaserScan> SyncPolicy;
    //boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;


    ros::Subscriber initial_pose_sub_;
    ros::Subscriber global_pose_sub_;//chq mod global loc

    sensor_msgs::LaserScan::ConstPtr cur_laser_scan;
    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    // Particle filter
    pf_t *pf_;
    pf_t *pf_global_loc;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    AMCLOdom* odom_;
    AMCLLaser* laser_;
    std::string init_loc_method;
    AMCLLaserData glo_ldata;//for init pos by GA alogrithm

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    // For slowing play-back when reading directly from a bag file
    ros::WallDuration bag_scan_period_;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_,pos_marker_pub;
    ros::Publisher particlecloud_pub_;
    ros::ServiceServer global_loc_srv_;
    ros::ServiceServer nomotion_update_srv_; //to let amcl update samples without requiring motion
    ros::ServiceServer set_map_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber map_sub_;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<amcl_optimize::AMCL_OPTIMIZEConfig> *dsrv_;
    amcl_optimize::AMCL_OPTIMIZEConfig default_config_;
    ros::Timer check_laser_timer_;

    int max_beams_, min_particles_, max_particles_;
    int pf_sample_dens;//used for pf init or global loc .the pf num in one grid
    int particles_per_m2;//每平方多少个粒子
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    ///chq add for gen a limit random pos
    bool resampled_in_limit_area_;
    static double resample_search_radius_;

    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    laser_model_t laser_model_type_;
    bool tf_broadcast_;

    ///ga for init loc
    int ga_pop_num;
    int ga_generation;
    double ga_search_radius;
    double ga_angle_error_para;
    int ga_x_encode_len;
    int ga_y_encode_len;
    int ga_angle_encode_len;
    double ga_covrate;
    double ga_mutrate;
    int ga_outputting_precision;
    std::string ga_select_method;
    std::string ga_cross_method;
    std::string ga_mutation_method;

    void reconfigureCB(amcl_optimize::AMCL_OPTIMIZEConfig &config, uint32_t level);

    ros::Time last_laser_received_ts_;
    ros::Duration laser_check_interval_;
    void checkLaserReceived(const ros::TimerEvent& event);
};
double AmclNode::resample_search_radius_ = 1;
std::vector<std::pair<int,int> > AmclNode::free_space_indices;

#define USAGE "USAGE: amcl"

boost::shared_ptr<AmclNode> amcl_node_ptr;

void sigintHandler(int sig)
{
  // Save latest pose as we're shutting down.
  amcl_node_ptr->savePoseToServer();
  ros::shutdown();
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl");
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  amcl_node_ptr.reset(new AmclNode());

  if (argc == 1)
  {
    // run using ROS input
    ros::spin();
  }
  else if ((argc == 3) && (std::string(argv[1]) == "--run-from-bag"))
  {
    amcl_node_ptr->runFromBag(argv[2]);
  }

  // Without this, our boost locks are not shut down nicely
  amcl_node_ptr.reset();

  // To quote Morgan, Hooray!
  return(0);
}

AmclNode::AmclNode() :
        sent_first_transform_(false),
        latest_tf_valid_(false),
        map_(NULL),
        pf_(NULL),
        pf_global_loc(NULL),
        resample_count_(0),
        odom_(NULL),
        laser_(NULL),
        private_nh_("~"),
        initial_pose_hyp_(NULL),
        first_map_received_(false),
        first_reconfigure_call_(true)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Grab params off the param server
  private_nh_.param("use_map_topic", use_map_topic_, false);
  private_nh_.param("first_map_only", first_map_only_, false);

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 0.5);
  save_pose_period = ros::Duration(1.0/tmp);

  private_nh_.param("laser_min_range", laser_min_range_, -1.0);
  private_nh_.param("laser_max_range", laser_max_range_, -1.0);
  //chq 激光匹配点束用数目，如果是360全向激光，720意味着没度使用两个激光束，减少这个数目可以有效提高定位计算速度
  // in a 100*10) m2 map , if =720 the waster 5 second to global loc if = 360 it only need 2 second
  private_nh_.param("laser_max_beams", max_beams_,720);//
  private_nh_.param("min_particles", min_particles_, 500);
  private_nh_.param("max_particles", max_particles_, 2000);

  private_nh_.param("pf_sample_dens", pf_sample_dens, 10);
  private_nh_.param("particles_per_m2", particles_per_m2, 200);

  private_nh_.param("kld_err", pf_err_, 0.01);
  private_nh_.param("kld_z", pf_z_, 0.99);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);
  
  private_nh_.param("do_beamskip", do_beamskip_, false);
  private_nh_.param("beam_skip_distance", beam_skip_distance_, 0.5);
  private_nh_.param("beam_skip_threshold", beam_skip_threshold_, 0.3);
  private_nh_.param("beam_skip_error_threshold_", beam_skip_error_threshold_, 0.9);

  ///chq z_hit_+z_short_+z_max_+z_rand_ shoud be 1
  ///
  private_nh_.param("laser_z_hit", z_hit_, 0.95);
  private_nh_.param("laser_z_short", z_short_, 0.1);
  private_nh_.param("laser_z_max", z_max_, 0.05);
  private_nh_.param("laser_z_rand", z_rand_, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("laser_lambda_short", lambda_short_, 0.1);
  private_nh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);
  std::string tmp_model_type;
  private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
  private_nh_.param<std::string>("init_loc_method",init_loc_method,"ga");//raw / rand_pf / ga
  if(tmp_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(tmp_model_type == "likelihood_field_prob"){
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  }
  else
  {
    ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
             tmp_model_type.c_str());
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  }

  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if(tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(tmp_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(tmp_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
             tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  private_nh_.param("update_min_d", d_thresh_, 0.2);
  private_nh_.param("update_min_a", a_thresh_, M_PI/6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_footprint"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("resample_interval", resample_interval_, 2);
  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
  ///chq慢速平均权重滤波器的指数衰减率，
  //用于决定何时通过添加随机姿态进行恢复操作，0.0表示禁用

  private_nh_.param("resampled_in_limit_area", resampled_in_limit_area_, true);///chq a rand pos gen range
  private_nh_.param("resample_search_radius", resample_search_radius_, 2.0);///chq a rand pos gen range
  private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.5/*0.001*/);//chq mod
  private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);

  private_nh_.param("tf_broadcast", tf_broadcast_, true);

  ///ga for init loc
  private_nh_.param("ga/pop_num", ga_pop_num, 200);
  private_nh_.param("ga/generation", ga_generation, 200);
  private_nh_.param("ga/search_radius", ga_search_radius, 10.0);
  private_nh_.param("ga/angle_error_para", ga_angle_error_para, 1.0);
//  private_nh_.param("ga/x_encode_len", ga_x_encode_len, 25);
//  private_nh_.param("ga/y_encode_len", ga_y_encode_len, 25);
//  private_nh_.param("ga/angle_encode_len", ga_angle_encode_len, 25);
  private_nh_.param("ga/covrate", ga_covrate, 0.95);
  private_nh_.param("ga/mutrate", ga_mutrate, 0.5);
  private_nh_.param("ga/outputting_precision", ga_outputting_precision, 6);
  private_nh_.param("ga/select_method", ga_select_method, std::string("SUS"));
  private_nh_.param("ga/cross_method", ga_cross_method, std::string("P1XO"));
  private_nh_.param("ga/mutation_method", ga_mutation_method, std::string("SPM"));

  std::cout << private_nh_.getNamespace()<<" pop_num: " << ga_pop_num <<" ga/generation: " << ga_generation <<std::endl;
  transform_tolerance_.fromSec(tmp_tol);

  {
    double bag_scan_period;
    private_nh_.param("bag_scan_period", bag_scan_period, -1.0);
    bag_scan_period_.fromSec(bag_scan_period);
  }

  updatePoseFromServer();

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new TransformListenerWrapper();

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);
  pos_marker_pub = nh_.advertise<visualization_msgs::Marker>("amcl_pos_marker",1,true);

  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
  global_loc_srv_ = nh_.advertiseService("global_localization", 
           &AmclNode::globalLocalizationCallback,
                                         this);
  nomotion_update_srv_= nh_.advertiseService("request_nomotion_update", &AmclNode::nomotionUpdateCallback, this);
  set_map_srv_= nh_.advertiseService("set_map", &AmclNode::setMapCallback, this);

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
  laser_scan_filter_ = 
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, 
                                                        *tf_, 
                                                        odom_frame_id_, 
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                   this, _1));
  ///for global loc

  //1
  //loc_sub_.reset(new message_filters::Subscriber<std_msgs::Int16>(nh_, "globalpose", 100));
  //laser_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100));
  //2
  //TimeSynchronizer<geometry_msgs::Pose, sensor_msgs::LaserScan> sync(loc_sub_, laser_sub_, 100);
  //3 注册回调
  //sync.registerCallback(boost::bind(&AmclNode::globalPoseReceived, this, _1, _2));

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
  global_pose_sub_ = nh_.subscribe("globalpose", 2, &AmclNode::globalPoseReceived, this);//chq
  if(use_map_topic_) {
    map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
    ROS_INFO("Subscribed to map topic.");
  }
  else {
    requestMap();
  }
  m_force_update = false;

  dsrv_ = new dynamic_reconfigure::Server<amcl_optimize::AMCL_OPTIMIZEConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<amcl_optimize::AMCL_OPTIMIZEConfig>::CallbackType cb = boost::bind(&AmclNode::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = ros::Duration(15.0);
  check_laser_timer_ = nh_.createTimer(laser_check_interval_, 
                                       boost::bind(&AmclNode::checkLaserReceived, this, _1));
  ///added by chq for global loc from starting loc
  //ros::Duration(0.5).sleep();
 // globalLocBySplitMap();
}

void AmclNode::reconfigureCB(AMCL_OPTIMIZEConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  //we don't want to do anything on the first call
  //which corresponds to startup
  if(first_reconfigure_call_)
  {
    first_reconfigure_call_ = false;
    default_config_ = config;
    return;
  }

  if(config.restore_defaults) {
    config = default_config_;
    //avoid looping
    config.restore_defaults = false;
  }

  d_thresh_ = config.update_min_d;
  a_thresh_ = config.update_min_a;

  resample_interval_ = config.resample_interval;

  laser_min_range_ = config.laser_min_range;
  laser_max_range_ = config.laser_max_range;

  gui_publish_period = ros::Duration(1.0/config.gui_publish_rate);
  save_pose_period = ros::Duration(1.0/config.save_pose_rate);

  transform_tolerance_.fromSec(config.transform_tolerance);

  max_beams_ = config.laser_max_beams;
  alpha1_ = config.odom_alpha1;
  alpha2_ = config.odom_alpha2;
  alpha3_ = config.odom_alpha3;
  alpha4_ = config.odom_alpha4;
  alpha5_ = config.odom_alpha5;

  z_hit_ = config.laser_z_hit;
  z_short_ = config.laser_z_short;
  z_max_ = config.laser_z_max;
  z_rand_ = config.laser_z_rand;
  sigma_hit_ = config.laser_sigma_hit;
  lambda_short_ = config.laser_lambda_short;
  laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;

  if(config.laser_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(config.laser_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(config.laser_model_type == "likelihood_field_prob")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;

  if(config.odom_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(config.odom_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(config.odom_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(config.odom_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;

  if(config.min_particles > config.max_particles)
  {
    ROS_WARN("You've set min_particles to be greater than max particles, this isn't allowed so they'll be set to be equal.");
    config.max_particles = config.min_particles;
  }

  min_particles_ = config.min_particles;
  max_particles_ = config.max_particles;

  ///pf_sample_dens = config.pf_sample_dens;

  alpha_slow_ = config.recovery_alpha_slow;
  alpha_fast_ = config.recovery_alpha_fast;
  tf_broadcast_ = config.tf_broadcast;

  do_beamskip_= config.do_beamskip; 
  beam_skip_distance_ = config.beam_skip_distance; 
  beam_skip_threshold_ = config.beam_skip_threshold; 

  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);

  ///chq
  pf_->limit_random_pose_fn = (pf_limit_sample_fn_t)AmclNode::limitUniformPoseGenerator;
  resampled_in_limit_area_ = config.resampled_in_limit_area;
  resample_search_radius_ = config.resample_search_radius;

  pf_err_ = config.kld_err; 
  pf_z_ = config.kld_z; 
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  ///ga
  init_loc_method = config.init_loc_method;
  ga_pop_num = config.pop_num;
  ga_generation = config.generation;
  ga_covrate = config.covrate;
  ga_mutrate = config.mutrate;

//  ga_x_encode_len = config.x_encode_len;
//  ga_y_encode_len = config.y_encode_len;
//  ga_angle_encode_len = config.angle_encode_len;

  ga_angle_error_para = config.angle_error_para;
  ga_search_radius = config.search_radius;
  ga_outputting_precision = config.outputting_precision;
  ga_cross_method = config.cross_method;
  ga_mutation_method =config.mutation_method;
  ga_select_method = config.select_method;


  ///for init or global loc

  pf_global_loc = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);//chq create pf
  pf_global_loc->pop_err = pf_err_;
  pf_global_loc->pop_z = pf_z_;

  pf_global_loc->limit_random_pose_fn = (pf_limit_sample_fn_t)AmclNode::limitUniformPoseGenerator;
  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose.pose.pose.orientation);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
  pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
  pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  ROS_ASSERT(laser_);
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_, 0.0);
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
          laser_likelihood_max_dist_,
          do_beamskip_, beam_skip_distance_,
          beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model with probabilities.");
  }
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }

  odom_frame_id_ = config.odom_frame_id;
  base_frame_id_ = config.base_frame_id;
  global_frame_id_ = config.global_frame_id;

  delete laser_scan_filter_;
  laser_scan_filter_ = 
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, 
                                                        *tf_, 
                                                        odom_frame_id_, 
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                   this, _1));

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
}


void AmclNode::runFromBag(const std::string &in_bag_fn)
{
  rosbag::Bag bag;
  bag.open(in_bag_fn, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("tf"));
  std::string scan_topic_name = "base_scan"; // TODO determine what topic this actually is from ROS
  topics.push_back(scan_topic_name);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Publisher laser_pub = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_name, 100);
  ros::Publisher tf_pub = nh_.advertise<tf2_msgs::TFMessage>("/tf", 100);

  // Sleep for a second to let all subscribers connect
  ros::WallDuration(1.0).sleep();

  ros::WallTime start(ros::WallTime::now());

  // Wait for map
  while (ros::ok())
  {
    {
      boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
      if (map_)
      {
        ROS_INFO("Map is ready");
        break;
      }
    }
    ROS_INFO("Waiting for map...");
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0));
  }

  BOOST_FOREACH(rosbag::MessageInstance const msg, view)
  {
    if (!ros::ok())
    {
      break;
    }

    // Process any ros messages or callbacks at this point
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());

    tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg != NULL)
    {
      tf_pub.publish(msg);
      for (size_t ii=0; ii<tf_msg->transforms.size(); ++ii)
      {
        tf_->getBuffer().setTransform(tf_msg->transforms[ii], "rosbag_authority");
      }
      continue;
    }

    sensor_msgs::LaserScan::ConstPtr base_scan = msg.instantiate<sensor_msgs::LaserScan>();
    if (base_scan != NULL)
    {
      laser_pub.publish(msg);
      laser_scan_filter_->add(base_scan);
      if (bag_scan_period_ > ros::WallDuration(0))
      {
        bag_scan_period_.sleep();
      }
      continue;
    }

    ROS_WARN_STREAM("Unsupported message type" << msg.getTopic());
  }

  bag.close();

  double runtime = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Bag complete, took %.1f seconds to process, shutting down", runtime);

  const geometry_msgs::Quaternion & q(last_published_pose.pose.pose.orientation);
  double yaw, pitch, roll;
  tf::Matrix3x3(tf::Quaternion(q.x, q.y, q.z, q.w)).getEulerYPR(yaw,pitch,roll);
  ROS_INFO("Final location %.3f, %.3f, %.3f with stamp=%f",
            last_published_pose.pose.pose.position.x,
            last_published_pose.pose.pose.position.y,
            yaw, last_published_pose.header.stamp.toSec()
            );

  ros::shutdown();
}


void AmclNode::savePoseToServer()
{
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose.
  tf::Pose map_pose = latest_tf_.inverse() * latest_odom_pose_;
  double yaw,pitch,roll;
  map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  ROS_DEBUG("Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y() );

  private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
  private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
  private_nh_.setParam("initial_pose_a", yaw);
  private_nh_.setParam("initial_cov_xx", 
                                  last_published_pose.pose.covariance[6*0+0]);
  private_nh_.setParam("initial_cov_yy", 
                                  last_published_pose.pose.covariance[6*1+1]);
  private_nh_.setParam("initial_cov_aa", 
                                  last_published_pose.pose.covariance[6*5+5]);
  
  //disable temp
  //std::cout << "chq.amcl.savePoseToServer done!"<<std::endl;


  
}

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
  // Check for NAN on input from param server, #5239
  double tmp_pos;
  private_nh_.param("initial_pose_x", tmp_pos, init_pose_[0]);
  if(!std::isnan(tmp_pos))
    init_pose_[0] = tmp_pos;
  else 
    ROS_WARN("ignoring NAN in initial pose X position");
  private_nh_.param("initial_pose_y", tmp_pos, init_pose_[1]);
  if(!std::isnan(tmp_pos))
    init_pose_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Y position");
  private_nh_.param("initial_pose_a", tmp_pos, init_pose_[2]);
  if(!std::isnan(tmp_pos))
    init_pose_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Yaw");
  private_nh_.param("initial_cov_xx", tmp_pos, init_cov_[0]);
  if(!std::isnan(tmp_pos))
    init_cov_[0] =tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance XX");
  private_nh_.param("initial_cov_yy", tmp_pos, init_cov_[1]);
  if(!std::isnan(tmp_pos))
    init_cov_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance YY");
  private_nh_.param("initial_cov_aa", tmp_pos, init_cov_[2]);
  if(!std::isnan(tmp_pos))
    init_cov_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance AA");
}

void 
AmclNode::checkLaserReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_laser_received_ts_;
  if(d > laser_check_interval_)
  {
    ROS_WARN("No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(),
             ros::names::resolve(scan_topic_).c_str());
  }
}

void
AmclNode::requestMap()
{
  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );
}

void
AmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  handleMapMessage( *msg );

  first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);

  freeMapDependentMemory();
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
  //marker cell occupy state
  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));//if not obs then marker freespace
#endif
  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);//chq create pf
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;
  pf_->limit_random_pose_fn = (pf_limit_sample_fn_t)AmclNode::limitUniformPoseGenerator;
  //the max count is fit map size actively
  //this pf is used for init orr global loc

  pf_global_loc = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);//chq create pf
  pf_global_loc->pop_err = pf_err_;
  pf_global_loc->pop_z = pf_z_;
  pf_global_loc->limit_random_pose_fn = (pf_limit_sample_fn_t)AmclNode::limitUniformPoseGenerator;


  // Initialize the filter
  updatePoseFromServer();
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);//create init pf sample chq
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  ROS_ASSERT(laser_);
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_, 0.0);
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
               laser_likelihood_max_dist_,
          do_beamskip_, beam_skip_distance_,
          beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }

  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  ///chq disabled if we change map then now we can init loc by optimized global loc method
  applyInitialPose();
  ///globalLocBySplitMap();

}

void
AmclNode::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
  if( pf_ != NULL ) {
    pf_free( pf_ );
    pf_ = NULL;
  }

  if( pf_global_loc != NULL ) {
    pf_free( pf_global_loc );
    pf_global_loc = NULL;
  }

  delete odom_;
  odom_ = NULL;
  delete laser_;
  laser_ = NULL;
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates a map_t and returns it.
 */
map_t*
AmclNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)       //free
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)//obs
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;//unkonwn
  }

  return map;
}

AmclNode::~AmclNode()
{
//for testing auto save init pos now disable temp
  //system("cd ~ && roslaunch para.launch");
  delete dsrv_;
  freeMapDependentMemory();
  delete laser_scan_filter_;
  delete laser_scan_sub_;
  delete tfb_;
  delete tf_;
  // TODO: delete everything allocated in constructor
}

bool
AmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)), t, f);
  try
  {
    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  return true;
}


pf_vector_t
AmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;

#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  ROS_DEBUG("Generating new uniform sample");
  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
#endif
  return p;
}
///chq generator a random pos in a limit area

pf_vector_t AmclNode::limitUniformPoseGenerator(void* pf,void* arg)
{
  pf_t* pf_ = (pf_t*)pf;
  map_t* map = (map_t*)arg;
  pf_vector_t bestpos,gen_pos;
  double x,y,angle;
  int q,p;

  double search_width_radius  = resample_search_radius_;
  int try_num = 2;
  double best_w;
  double max_weight = 0.0;
  int max_weight_hyp = -1;

  pf_sample_set_t* set;
  pf_sample_t *sample;
  set = pf_->sets + pf_->current_set;

  ///
  double best_weight = 0.0;
  ///
  for(int count = 0;count < set->sample_count;count++)
  {
    double weight;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    sample = set->samples + count;
    weight = sample->weight;
    pose_mean = sample->pose;
    pose_cov = pf_matrix_zero();
    if( weight > max_weight)
    {
      bestpos = pose_mean;
      max_weight = weight;
    }
  }
  if(max_weight <= 0.0)
  {
     return pf_vector_zero();
  }

  double mid_x = bestpos.v[0];
  double mid_y = bestpos.v[1];
  while(try_num--){
    //如果搜索长宽都等与０　代表此坐标已经check过了，默认有效
      x = mid_x + drand48()*2*search_width_radius - search_width_radius;
      y = mid_y + drand48()*2*search_width_radius - search_width_radius;
      // Check that it's a free cell
      q = MAP_GXWX(map,x);
      p = MAP_GYWY(map,y);
      //if pos is valid and is free ,suc
      if(MAP_VALID(map,q,p) && (map->cells[MAP_INDEX(map,q,p)].occ_state == -1)){
        break;
      }
  }
  if(try_num < 0)
    return bestpos;

  angle = drand48()*2*M_PI - M_PI;//
  angle = normalize(angle);
  gen_pos.v[0] = x;
  gen_pos.v[1] = y;
  gen_pos.v[2] = angle;

  return gen_pos;
}

void
AmclNode::LocalLocByGaAlogrithm(double search_radius, pf_vector_t init_pos){
  pf_vector_t mean_result;
  ROS_INFO("LocalLocByGaAlogrithm.start...");
  ///TODO 种群规模太大会出现问题　待解决
  /// 初始值很重要
  //模板参数都是在编译期间就确定下来的，你用一个变量来做参数怎么能编译通过呢
  galgo::Parameter<double,32> para1({init_pos.v[0]-search_radius, init_pos.v[0]+search_radius,init_pos.v[0]});//min max init_value
  galgo::Parameter<double,32> para2({init_pos.v[1]-search_radius, init_pos.v[1]+search_radius,init_pos.v[1]});
  galgo::Parameter<double,10> para3({ -M_PI,M_PI,init_pos.v[2]});

  ///激光数据必须先修正
  if(!cur_laser_scan /*||
     frame_to_laser_.find(cur_laser_scan->header.frame_id) == frame_to_laser_.end()*/){
    ROS_ERROR("!!!AmclNode::LocalLocByGaAlogrithm.cur_laser_scan is not valid.return");
    return;
  }

  const sensor_msgs::LaserScan::ConstPtr tmp_laser = cur_laser_scan;
  //insert the laser msg into laser vector
  //and update laser sensor pos in base frame
  int laser_index = -1;
  laser_index = updateLaserVector(tmp_laser);
  if( laser_index < 0){
    ROS_ERROR("AmclNode::LocalLocByGaAlogrithm.updateLaserVector. laser_index < 0 return!!!");
    return ;
  }

  AMCLLaser* lasers = lasers_[laser_index];
  AMCLLaserData ldata;
  int tail_result = tailoringScan(tmp_laser,lasers,ldata);
  if( tail_result < 0 ){
    ROS_ERROR("AmclNode::LocalLocByGaAlogrithm.tailoringScan failed return!!!");
    return;
  }

  InitLocObjective<double> t;
  #if 0
  t.data = ldata;
  pf_vector_t temp_pos;
  lasers->GetLaserPose(temp_pos);
  t.laser_pose = temp_pos;
  t.max_beams = max_beams_;
  t.map = map_;
#endif
  t.data = &ldata;
  t.laser = (AMCLLaser*) ldata.sensor;

  //GeneticAlgorithm(Func<T> objective, int popsize, int nbgen, bool output, const Parameter<T,N>&...args)
  galgo::GeneticAlgorithm<double> ga( InitLocObjective<double>::Objective,ga_pop_num,ga_generation,true,para1,para2,para3);
 // ga.tolerance= 0.0001;
  ///TODO switch case to change
  ga.Selection = SUS; // stochastic universal sampling (SUS)//sus 随机遍历采样
  ga.CrossOver = P2XO;// uniform cross-over (UXO)//
  ga.Mutation = UNM; // uniform mutation

  ga.covrate = ga_covrate;
  ga.mutrate = ga_mutrate;
  ga.precision = ga_outputting_precision;////number of decimals for <outputting> results
  #if 0
    // setting constraints
    ga.Constraint = MyObj<double>::MyConstraint;
    ga.tolerance = -0.05*0.05;//terminal condition to stop the algorithm
    ga.precision = 2;
  #endif
  ga.run();

  galgo::CHR<double> result(ga.result()) ;
  std::vector<double> para = result.get()->getParam();
  std::vector<double> err = result.get()->getResult();

  mean_result.v[0] = para[0];
  mean_result.v[1] = para[1];
  mean_result.v[2] = para[2];
  pf_init(pf_, mean_result,pf_matrix_zero());
  pf_init_ = false;
}

void
AmclNode::LocalLocBySplitMap(double search_w,double search_h,double init_x,double init_y){
  int result = -1;
  ROS_INFO("LocalLocBySplitMap.start...");

  if(!cur_laser_scan ||
     frame_to_laser_.find(cur_laser_scan->header.frame_id) == frame_to_laser_.end()){
    ROS_ERROR("!!!AmclNode::LocalLocBySplitMap.cur_laser_scan is not valid.return");
    return;
  }

  const sensor_msgs::LaserScan::ConstPtr tmp_laser = cur_laser_scan;
  //insert the laser msg into laser vector
  //and update laser sensor pos in base frame
  int laser_index = -1;
  laser_index = updateLaserVector(tmp_laser);
  if( laser_index < 0){
    ROS_ERROR("AmclNode::LocalLocBySplitMap.updateLaserVector. laser_index < 0 return!!!");
    return ;
  }

  AMCLLaser* lasers = lasers_[laser_index];
  AMCLLaserData ldata;
  result = tailoringScan(tmp_laser,lasers,ldata);
  if(result < 0 ){
    ROS_ERROR("AmclNode::LocalLocBySplitMap.tailoringScan failed return!!!");
    return;
  }

  double search_width = search_w;
  double search_height = search_h;

  std::pair<double,double> mid_pos;
  std::vector<std::pair<double,double> >  mid_pos_set;

  pf_vector_t bestPos;
  double best_w;
  pf_global_loc->max_samples = max_particles_;

  //uniformly split the map , get every mid pos
  //and the sub map's width and height in sub map
  uniformSplitSubMap(pf_global_loc,mid_pos_set,search_width,search_height,init_x,init_y);

  //ROS_INFO_STREAM("AmclNode::LocalLocBySplitMap.uniformSplitMap.mid_pos_set size:" << mid_pos_set.size()
  //                << " search w and h: " << search_width << search_height);

  int mid_pos_num = mid_pos_set.size();

  double x,y;
  int count = 0;
  pf_vector_t v_submap_pos[pf_global_loc->max_samples];
  pf_vector_t v_map_pos[mid_pos_num];
  //假设搜索长宽是10m,100m2,分成最大粒子数块，如2000块，则每块是长宽0.05m2,one grid is qual to 2.5e-3m2,差不每块5e-2/2.5*e-3 = 20个栅格，
  //每个栅格，在角度上是-180-180度，共计360，如果每10度一个随机数据，则每块粒子最大数目是36*20=720个
  int split_sample_size = 80;
  for(int i = 0; i < mid_pos_num; i++){
    x = mid_pos_set[i].first;
    y = mid_pos_set[i].second;
    mid_pos = std::make_pair(x,y);
    //in every sub map ,generate random pos samples
    //ROS_INFO("AmclNode::LocalLocBySplitMap.bef generateRandomPfSet.loop_index:%d.",loop_index++);
    int set_size = generateRandomPfSet(v_submap_pos,split_sample_size,mid_pos,search_width,search_height);
    if(!set_size)continue;
    if(i%20 == 0){
      double pec = (double)i/(double)mid_pos_num;
      pec*=100;
      ROS_INFO("AmclNode::LocalLocBySplitMap.search percent:%d%%",(int)pec);
    }
    //reset th pf weight
    //ROS_INFO("AmclNode::LocalLocBySplitMap.bef resetAndClustingPfSet.");
    resetAndClustingPfSet(pf_global_loc,set_size,v_submap_pos,false);
    //ROS_INFO("AmclNode::LocalLocBySplitMap.aft resetAndClustingPfSet.");
    //score every sample
    //ROS_INFO("AmclNode::LocalLocBySplitMap.bef evaluatePfByLaser.");
    evaluatePfByLaser(pf_global_loc,lasers,ldata);
    //ROS_INFO("AmclNode::LocalLocBySplitMap.after evaluatePfByLaser.");

    //cluster the evaluted pf
    //then we can get a comprehensive loc result in every sub map
    //ROS_INFO("AmclNode::LocalLocBySplitMap.bef clustingPfSet.");
    ///clustingPfSet(pf_global_loc,false);
    //ROS_INFO("AmclNode::LocalLocBySplitMap.aft clustingPfSet.");
    //get best clustered sample pos in sub map
    // ROS_INFO("AmclNode::LocalLocBySplitMap.bef getBestSample.");
    double
    result = getBestSample(pf_global_loc,bestPos,best_w);
     //ROS_INFO("AmclNode::LocalLocBySplitMap.after getBestSample.");
    if(result >= 0 ){
      //store every cluster pos for evaluting in complete map
      v_map_pos[count++]=bestPos;
    }
    }

  //ROS_INFO("AmclNode::LocalLocBySplitMap.global sample set size:%d,now do global clusting...:",count);

  //reset the pf weight bef evaluting the pf and do not clustering
  resetAndClustingPfSet(pf_global_loc,count,v_map_pos,false);
  evaluatePfByLaser(pf_global_loc,lasers,ldata);
  // clustering the pf to get comprehensive loc result but do not set uniform weight
  // so we can get a max best clustered result
  clustingPfSet(pf_global_loc,false);
  result = getBestSample(pf_global_loc,bestPos,best_w);
  if(result < 0) {
    ROS_ERROR("globalposeinit.get last best pos failed!return");
    return;
  }

  pf_matrix_t cov =pf_matrix_zero();
  pf_init(pf_, bestPos,cov);
  pf_init_ = false;
  ROS_INFO("AmclNode::LocalLocBySplitMap.Done");

}

void
AmclNode::globalLocBySplitMap(void){
  int result = -1;
  ROS_INFO("----------------globalLocBySplitMap.Start-----------------");

  if(!cur_laser_scan /*||
     frame_to_laser_.find(cur_laser_scan->header.frame_id) == frame_to_laser_.end()*/){
    ROS_ERROR("!!!Error!AmclNode::globalLocBySplitMap.cur_laser_scan is not valid.return");
    return;
  }

  const sensor_msgs::LaserScan::ConstPtr tmp_laser = cur_laser_scan;
  //insert the laser msg into laser vector
  //and update laser sensor pos in base frame
  int laser_index = -1;
  laser_index = updateLaserVector(tmp_laser);
  if( laser_index < 0){
    ROS_ERROR("AmclNode::globalLocBySplitMap.updateLaserVector. laser_index < 0 return!!!");
    return ;
  }

  AMCLLaser* lasers = lasers_[laser_index];
  AMCLLaserData ldata;
  result = tailoringScan(tmp_laser,lasers,ldata);
  if(result < 0 ){
    ROS_ERROR("AmclNode::globalLocBySplitMap.tailoringScan failed return!!!");
    return;
  }

  double search_width,search_height;

  std::pair<double,double> mid_pos;
  std::vector<std::pair<double,double> >  mid_pos_set;

  pf_vector_t bestPos;
  double best_w;
  pf_global_loc->max_samples = max_particles_;
  //uniformly split the map , get every mid pos
  //and the sub map's width and height in sub map
  uniformSplitMap(pf_global_loc,mid_pos_set,search_width,search_height);

  //ROS_INFO_STREAM("globalLocBySplitMap.uniformSplitMap.mid_pos_set size:" << mid_pos_set.size()
  //                << " search w and h: " << search_width << search_height);

  int mid_pos_num = mid_pos_set.size();

  double x,y;
  int count = 0;
  pf_vector_t v_submap_pos[pf_global_loc->max_samples];
  pf_vector_t v_map_pos[mid_pos_num];
  /// 假设搜索长宽是200,4e4m2,分成最大粒子数块，如2e3块，则每块是长宽20m2,长宽大致是4.47约为5m
  int grid_size_x = map_->size_x;
  int grid_size_y = map_->size_y;
  double scale = map_->scale;
  int max_sam = pf_global_loc->max_samples;
  //the area of  one sbumap
  double one_split_area = (double)(scale*grid_size_x) * (double) (scale*grid_size_y) / max_sam ;
  //the specified sampled num of one area submap
  double one_area_num = 36;
  //the sampled num for one sub map
  int split_area_num = one_split_area * one_area_num;

  int split_sample_size = split_area_num < max_sam ? split_area_num:max_sam;
  /// 粗定位 每一块大致选出最好的结果
  for(int i = 0; i < mid_pos_num; i++){
        x = mid_pos_set[i].first;
        y = mid_pos_set[i].second;
        mid_pos = std::make_pair(x,y);
       //in every sub map ,generate random pos samples
       //ROS_INFO("globalLocBySplitMap.bef generateRandomPfSet.loop_index:%d.",loop_index++);
      int set_size = generateRandomPfSet(v_submap_pos,split_sample_size,mid_pos,search_width,search_height);

      if(!set_size)continue;
      if(i%100 == 0){
        double pec = (double)i/(double)mid_pos_num;
        pec *= 100;
        ROS_INFO("AmclNode::GlobalLocBySplitMap.search percent:%d %%",(int)pec);
      }
      //reset th pf weight
      //ROS_INFO("globalLocBySplitMap.bef resetAndClustingPfSet.");
      resetAndClustingPfSet(pf_global_loc,set_size,v_submap_pos,false);
      //ROS_INFO("globalLocBySplitMap.aft resetAndClustingPfSet.");
      //score every sample
      //ROS_INFO("globalLocBySplitMap.bef evaluatePfByLaser.");
      evaluatePfByLaser(pf_global_loc,lasers,ldata);
      //ROS_INFO("globalLocBySplitMap.after evaluatePfByLaser.");

      //cluster the evaluted pf
      //then we can get a comprehensive loc result in every sub map
      //ROS_INFO("globalLocBySplitMap.bef clustingPfSet.");
      ///clustingPfSet(pf_global_loc,false);
      //ROS_INFO("globalLocBySplitMap.aft clustingPfSet.");
      //get best clustered sample pos in sub map
      // ROS_INFO("globalLocBySplitMap.bef getBestSample.");
      result = getBestSample(pf_global_loc,bestPos,best_w);
       //ROS_INFO("globalLocBySplitMap.after getBestSample.");
      if(result >= 0 ){
        //store every cluster pos for evaluting in complete map
        v_map_pos[count++]=bestPos;
      }
    }

    ROS_INFO("globalLocBySplitMap.global sample set size:%d,now do global clusting...:",count);

    //reset the pf weight bef evaluting the pf and do not clustering
    resetAndClustingPfSet(pf_global_loc,count,v_map_pos,false);
    evaluatePfByLaser(pf_global_loc,lasers,ldata);
    // clustering the pf to get comprehensive loc result but do not set uniform weight
    // so we can get a max best clustered result
    clustingPfSet(pf_global_loc,false);
    result = getBestSample(pf_global_loc,bestPos,best_w);
    if(result < 0) {
      ROS_ERROR("globalposeinit.get last best pos failed!return");
      return;
    }else{
      ROS_INFO("globalposeinit.get Init global best pos!pos:(%.3f,%.3f,%.3f),weight:%.3f",
               bestPos.v[0],bestPos.v[1],bestPos.v[2],best_w);
    }
    double w = sqrt(one_split_area);
    double h = w;
    /// 在近似最好的那一块精确定位
    ///LocalLocBySplitMap(w,h,bestPos.v[0],bestPos.v[1]);
    LocalLocByGaAlogrithm(w,bestPos);
    //pf_init_ = false;
   ROS_INFO("----------------globalLocBySplitMap.Done-----------------");
}

//split a sub map uniformly
void
AmclNode::uniformSplitSubMap(pf_t *pf,std::vector<std::pair<double,double> > & mid_pos_set,
                double &search_width,double &search_height, double init_x, double init_y){
  double x,y;
  int pf_samplecount = pf->max_samples;
  //因为限定了pf_global_loc->max_samples的大小等于单个栅格采样数目*地图大小，故可以划分每个栅格作为采样点
  if( pf_samplecount <= 0 ){
    ROS_ERROR("AmclNode::uniformSplitSubMap.pf->max_samples <= 0.0.return!");
    return;
  }
  if(map_->scale <= 0.0){
    ROS_ERROR("AmclNode::uniformSplitSubMap.map_->scale <= 0.0.return!");
    return;
  }
  int map_size_x = search_width / map_->scale;
  int map_size_y = search_height / map_->scale;
  double map_scale = map_->scale;
  mid_pos_set.clear();
  int q,p;
  int k ;
    q = MAP_GXWX(map_,init_x);
    p = MAP_GYWY(map_,init_y);
    //if pos is valid and is free ,suc
    if(!MAP_VALID(map_,q,p)|| (map_->cells[MAP_INDEX(map_,q,p)].occ_state != -1)){
      ROS_ERROR("AmclNode::uniformSplitSubMap.init pos is not valid or occupied.return!");
      return;
    }
    //求得左上角网格坐标
  int left_top_q = q - (double)map_size_x/2.0;
  int left_top_p = p - (double)map_size_y/2.0;
  if( left_top_q < 0) left_top_q = 0;
  if( left_top_p < 0) left_top_p = 0;
  int grid_x,grid_y;

  if( map_size_x * map_size_y <= pf_samplecount){//if the gird num size below to pf_samplecount then the split num = map_size_x * map_size_y
    for(int i = 0 ; i < map_size_x; i++){
      for(int j = 0 ; j < map_size_y; j++){
        grid_x = left_top_q+i;
        grid_y = left_top_p+j;
        if(!MAP_VALID(map_,grid_x,grid_y)|| (map_->cells[MAP_INDEX(map_,grid_x,grid_y)].occ_state != -1)){
          continue;
        }
        x = MAP_WXGX(map_,grid_x);
        y = MAP_WYGY(map_,grid_y);
        mid_pos_set.push_back(std::make_pair(x,y));
      }
    }
    search_width = search_height = 0.0;
    return;
  }
  else{
    //min(k)>=1
    // proportion split make sure that the mid_pos cord num is never beyond pf_samplecount
    k = ceil(sqrt((double)(map_size_x * map_size_y)/(double)pf_samplecount));//otherwise proportion split
    map_size_x = map_size_x / k;
    map_size_y = map_size_y / k;
    for(int i = 0 ;i < map_size_x;i++){
      for(int j = 0 ; j < map_size_y;j++){
        grid_x = left_top_q+(i*k+(i+1)*k)/2;
        grid_y = left_top_p+(j*k+(j+1)*k)/2;
        if(!MAP_VALID(map_,grid_x,grid_y)|| (map_->cells[MAP_INDEX(map_,grid_x,grid_y)].occ_state != -1)){
          continue;
        }
        x = MAP_WXGX(map_,grid_x );
        y = MAP_WYGY(map_,grid_y );
        mid_pos_set.push_back(std::make_pair(x,y));
      }
  }
    search_width = k * map_scale;
    search_height = k * map_scale;
}

}

//
void
AmclNode::uniformSplitMap(pf_t *pf,std::vector<std::pair<double,double> > & mid_pos_set,
                double &search_width,double &search_height){
  double x,y;
  int pf_samplecount = pf->max_samples;
  int map_size_x = map_->size_x;
  int map_size_y = map_->size_y;
  double map_scale = map_->scale;
  mid_pos_set.clear();
  int q,p;
  int k ;
  //因为限定了pf_global_loc->max_samples的大小等于单个栅格采样数目*地图大小，故可以划分每个栅格作为采样点
  if(!map_size_x || !map_size_y || pf_samplecount <= 0)return;

  if( map_size_x * map_size_y <= pf_samplecount){//if the gird num size below to pf_samplecount then the split num = map_size_x * map_size_y
    for(int i = 0 ; i < map_size_x; i++){
      for(int j = 0 ; j < map_size_y; j++){
        x = MAP_WXGX(map_,i);
        y = MAP_WYGY(map_,j);
        mid_pos_set.push_back(std::make_pair(x,y));
      }
    }
    search_width = search_height = 0.0;
  }
  else{
    k = ceil(sqrt((double)(map_size_x * map_size_y)/(double)pf_samplecount));//otherwise proportion split
    map_size_x = map_size_x / k;
    map_size_y = map_size_y / k;
    for(int i = 0 ;i < map_size_x;i++){
      for(int j = 0 ; j < map_size_y;j++){
        x = MAP_WXGX(map_,(i*k+(i+1)*k)/2 );
        y = MAP_WYGY(map_,(j*k+(j+1)*k)/2 );
        mid_pos_set.push_back(std::make_pair(x,y));
      }
  }
    search_width = k * map_scale;
    search_height = k * map_scale;
}

}


int
AmclNode::generateRandomPfSet(pf_vector_t* out_pf_set,int gennum,std::pair<double,double> mid_pos,double search_width, double search_height){
 double mid_x = mid_pos.first;
 double mid_y = mid_pos.second;
 //pf_matrix_t cov = initial_pose_hyp_->pf_pose_cov;
 pf_vector_t mean_search,pose;
 double x,y,angle;
 int q,p;
 double search_width_radius  = search_width/2.0;
 double search_height_radius  = search_height/2.0;
 int cnt = 0 ;
 bool try_once = false ;
 if( fabs(search_width_radius)<= 0.0 && fabs(search_height_radius) <= 0.0 )
   try_once = true;
 int const_chance = try_once?1:5;
 int rand_gen_chance;

 for (int i = 0; i < gennum ; i++)
 {
   rand_gen_chance = const_chance;//default search once (if search w and h is equal to zero)
#if 0
   while(rand_gen_chance--)//防止该区域没有有效点
   {
     if( try_once ){
       x = mid_x;
       y = mid_y;
     }
     else
     {
       x = mid_x + drand48()*2*search_width_radius - search_width_radius;
       y = mid_y + drand48()*2*search_height_radius - search_height_radius;
     }
     angle = drand48()*2*M_PI - M_PI;//
     angle = normalize(angle);
     // Check that it's a free cell
     q = MAP_GXWX(map_,x);
     p = MAP_GYWY(map_,y);
     //if pos is valid and is free ,suc
     if(MAP_VALID(map_,q,p) && (map_->cells[MAP_INDEX(map_,q,p)].occ_state == -1)){
       break;
     }
   }
   //ROS_INFO("AmclNode::generateRandomPfSet.i:%d,rand_gen_chance:%d",i,rand_gen_chance);
   if (rand_gen_chance < 0)continue;
   //std::cout << "AmclNode::applyInitialPose.search (x,y): (" << x << y <<") \n";
    pf_matrix_t cov;
    if(initial_pose_hyp_ !=NULL )
      cov = initial_pose_hyp_->pf_pose_cov;
    else
      cov = pf_matrix_zero();

   mean_search.v[0] = x;
   mean_search.v[1] = y;
   mean_search.v[2] = angle;
   pose = pf_get_one_guassian_sample(pf_,mean_search, cov);

   //结合了之前init算法的优势，建立高斯密度函数，在随机点处高斯采样
   pose = pf_get_one_guassian_sample(pf_,mean_search, cov);
#endif
   if( try_once ){
     x = mid_x;
     y = mid_y;
     // Check that it's a free cell
     q = MAP_GXWX(map_,x);
     p = MAP_GYWY(map_,y);
     //if pos is valid and is free ,suc
     if( !MAP_VALID(map_,q,p) || (map_->cells[MAP_INDEX(map_,q,p)].occ_state != -1)){
      return cnt;
     }
   }
   else
   {
     while(rand_gen_chance--)//防止该区域没有有效点
     {
     //如果搜索长宽都等与０　代表此坐标已经check过了，默认有效
       x = mid_x + drand48()*2*search_width_radius - search_width_radius;
       y = mid_y + drand48()*2*search_height_radius - search_height_radius;
       // Check that it's a free cell
       q = MAP_GXWX(map_,x);
       p = MAP_GYWY(map_,y);
       //if pos is valid and is free ,suc
       if(MAP_VALID(map_,q,p) && (map_->cells[MAP_INDEX(map_,q,p)].occ_state == -1)){
         break;
       }
     }
   }

   if (rand_gen_chance < 0)continue;
   angle = drand48()*2*M_PI - M_PI;//
   angle = normalize(angle);
   mean_search.v[0] = x;
   mean_search.v[1] = y;
   mean_search.v[2] = angle;
   out_pf_set[cnt++] = mean_search;
 }
 //ROS_INFO("AmclNode::generateRandomPfSet.cnt set:%d",cnt);
 return cnt;
}

#if 0
int
AmclNode::generatePfBySampleDensity(pf_vector_t* out_pf_set,map_t* map,int den,std::pair<double,double> mid_pos){
 double mid_x = mid_pos.first;
 double mid_y = mid_pos.second;
 //pf_matrix_t cov = initial_pose_hyp_->pf_pose_cov;
 pf_vector_t mean_search;
 double x,y,angle;
 int q,p;
 int cnt = 0 ;
 double sample_prob = (double)den/pow(1.0/map->scale,2);
 //如果搜索长宽都等与０　代表此坐标已经check过了，默认有效
 x = mid_x ;
 y = mid_y;
 // Check that it's a free cell
 q = MAP_GXWX(map_,x);
 p = MAP_GYWY(map_,y);
 //if pos is valid and is free ,suc
 if( !MAP_VALID(map_,q,p) || (map_->cells[MAP_INDEX(map_,q,p)].occ_state != -1)){
   return cnt;
 }
 int gennum = sample_prob >= 1?sample_prob:1;
 for (int i = 0; i < gennum ; i++)
 {
   if(sample_prob < 1)//if the   gen num prob is below to 1 then we create a pf by the prob
   {
     if(drand48() > sample_prob)// not gen prob
       return cnt;
   }
   angle = drand48()*2*M_PI - M_PI;//
   angle = normalize(angle);
   mean_search.v[0] = x;
   mean_search.v[1] = y;
   mean_search.v[2] = angle;
   out_pf_set[cnt++] = mean_search;
 }
 //ROS_INFO("AmclNode::generateRandomPfSet.cnt set:%d",cnt);
 return cnt;
}
#endif
void
AmclNode::evaluatePfByLaser(pf_t *pf,AMCLLaser* laser,AMCLLaserData& ldata){

  pf_sample_set_t* set;
  pf_sample_t *sample;

  set = pf->sets + pf->current_set;
  int sample_size = set->sample_count;
  double p;
  double sum_w = 0.0;

  for(int i = 0; i < sample_size; i++)
  {
    sample = set->samples+i;
    pf_vector_t pos =sample->pose;
    p = -laser->evaluateOnePose(&ldata,pos);
    //printf("evaluatePfByLaser.evaluateOnePose pos:(%.2f,%.2f,%.2f) err dist:%.3f\n",pos.v[0],pos.v[1],pos.v[2],p);
    if(p <= 0 )p = 1e-4;
    sample->weight = 1.0/(double)p;
    sum_w+=sample->weight;
  }
  //normalize
  for(int i= 0; i<sample_size; i++){
     sample = set->samples+i;
     sample->weight = sample->weight / sum_w ;
  }

 }

void
AmclNode::updatePfByLaser(AMCLLaser* laser,AMCLLaserData& ldata ){
  laser->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
}

int
AmclNode::getBestSample(pf_t *pf,pf_vector_t& best_pose,double &best_weight){
  int result = -1;
  double max_weight = 0.0;
  int max_weight_hyp = -1;
  std::vector<amcl_hyp_t> hyps;
  pf_sample_set_t* set;
  pf_sample_t *sample;
  set = pf->sets + pf->current_set;
  hyps.resize(set->sample_count);
  ///
  best_weight = 0.0;
  ///
  for(int hyp_count = 0;
      hyp_count < set->sample_count; hyp_count++)
  {
    double weight;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    sample = set->samples + hyp_count;
    weight = sample->weight;
    pose_mean = sample->pose;
    pose_cov = pf_matrix_zero();

    hyps[hyp_count].weight = weight;
    hyps[hyp_count].pf_pose_mean = pose_mean;
    hyps[hyp_count].pf_pose_cov = pose_cov;

    if(hyps[hyp_count].weight > max_weight)
    {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
  }

  if(max_weight > 0.0)
  {
    best_pose =  hyps[max_weight_hyp].pf_pose_mean;

    ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
              best_pose.v[0],
              best_pose.v[1],
              best_pose.v[2]);
    result = 0;
    best_weight = max_weight;
    return result;
  }
  else
    return result;
}

/// the third para: best_weight_perc can be decribled as loc reliablity
// in real experiment ,if it is below to 0.9 ,the loc reliablity would be thought as low
int
AmclNode::getBestClusterSample(pf_t *pf, pf_vector_t& best_pose,double &best_weight_perc){
  int result = -1;
  double max_weight = 0.0;
  int max_weight_hyp = -1;
  // std::vector<amcl_hyp_t> hyps;
  double sum_w=0.0;
  //hyps.resize(pf->sets[pf->current_set].cluster_count);
  for(int hyp_count = 0;
      hyp_count < pf->sets[pf->current_set].cluster_count; hyp_count++)
  {
    double weight;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    if (!pf_get_cluster_stats(pf, hyp_count, &weight, &pose_mean, &pose_cov))
    {
      ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
      break;
    }
    sum_w+=weight;
    //hyps[hyp_count].weight = weight;
   // hyps[hyp_count].pf_pose_mean = pose_mean;
    //hyps[hyp_count].pf_pose_cov = pose_cov;

    if(weight > max_weight)
    {
      max_weight = weight;
      max_weight_hyp = hyp_count;
      best_pose = pose_mean;
    }
  }

  if( std::isnan(best_pose.v[0]) || std::isnan(best_pose.v[1]) || std::isnan(best_pose.v[2])){
    ROS_ERROR("getBestClusterSample.Max weight pose is nan ");
    return result;
  }

  if(max_weight > 0.0)
  {

    best_weight_perc = sum_w > 0?max_weight/sum_w : 0;
    ROS_ERROR("Max weight pose: %.3f %.3f %.3f",
              best_pose.v[0],
              best_pose.v[1],
              best_pose.v[2]);

    result = 0;
    return result;
  }
  else
    return result;
}

bool
AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  if( map_ == NULL ) {
    return true;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("globalLocalizationCallback.Initializing with uniform distribution");
  //传入uniformPoseGenerator函数，在采样个数限定下,每次调用生成一个地图内的随机位置
  //pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
  //                (void *)map_);
  globalLocBySplitMap();
  ROS_INFO("globalLocalizationCallback.Global initialisation done!");
  pf_init_ = false;
  return true;
}

// force nomotion updates (amcl updating without requiring motion)
bool 
AmclNode::nomotionUpdateCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  m_force_update = true;
  //ROS_INFO("Requesting no-motion update");
  return true;
}

bool
AmclNode::setMapCallback(nav_msgs::SetMap::Request& req,
                         nav_msgs::SetMap::Response& res)
{
  handleMapMessage(req.map);
  handleInitialPoseMessage(req.initial_pose);
  res.success = true;
  return true;
}
///按照给定的激光最大最小范围参数裁剪激光，不符合范围的置为无效数据
int AmclNode::tailoringScan(const sensor_msgs::LaserScanConstPtr& laser_scan,
                            AMCLSensor *sensor,AMCLLaserData& ldata){
int result  = -1;
ldata.sensor = sensor;
ldata.range_count = laser_scan->ranges.size();

// To account for lasers that are mounted upside-down, we determine the
// min, max, and increment angles of the laser in the base frame.
//
// Construct min and max angles of laser, in the base_link frame.
tf::Quaternion q;
q.setRPY(0.0, 0.0, laser_scan->angle_min);
tf::Stamped<tf::Quaternion> min_q(q, laser_scan->header.stamp,
                                  laser_scan->header.frame_id);
q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
tf::Stamped<tf::Quaternion> inc_q(q, laser_scan->header.stamp,
                                  laser_scan->header.frame_id);
try
{
  tf_->transformQuaternion(base_frame_id_, min_q, min_q);
  tf_->transformQuaternion(base_frame_id_, inc_q, inc_q);
}
catch(tf::TransformException& e)
{
  ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
           e.what());
  return result;
}

double angle_min = tf::getYaw(min_q);
double angle_increment = tf::getYaw(inc_q) - angle_min;

// wrapping angle to [-pi .. pi]
angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

//ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);

// Apply range min/max thresholds, if the user supplied them
if(laser_max_range_ > 0.0)
  ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
else
  ldata.range_max = laser_scan->range_max;
double range_min;
if(laser_min_range_ > 0.0)
  range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
else
  range_min = laser_scan->range_min;
// The AMCLLaserData destructor will free this memory
ldata.ranges = new double[ldata.range_count][2];
ROS_ASSERT(ldata.ranges);
for(int i=0;i<ldata.range_count;i++)
{
  // amcl doesn't (yet) have a concept of min range.  So we'll map short
  // readings to max range.
  if(laser_scan->ranges[i] <= range_min)
    ldata.ranges[i][0] = ldata.range_max;
  else
    ldata.ranges[i][0] = laser_scan->ranges[i];
  // Compute bearing
  ldata.ranges[i][1] = angle_min +
          (i * angle_increment);
}
result = 0;
return result;
}

int
AmclNode::updateLaserVector(const sensor_msgs::LaserScanConstPtr& laser_scan){
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  /// for more than a laser ///chq
  if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
    lasers_.push_back(new AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)),
                                 ros::Time(), laser_scan->header.frame_id);
    tf::Stamped<tf::Pose> laser_pose;
    try
    {
      this->tf_->transformPose(base_frame_id_, ident, laser_pose);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(),
                base_frame_id_.c_str());
      return laser_index;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;///??????chq unknown
    lasers_[laser_index]->SetLaserPose(laser_pose_v);///!!!!
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }
  return laser_index;
}

void AmclNode::pubPoseAndTF(const sensor_msgs::LaserScanConstPtr& laser_scan,pf_vector_t bestPos,double best_weight_perc){
  geometry_msgs::PoseWithCovarianceStamped p;
  visualization_msgs::Marker text_marker;

  // Fill in the header
  p.header.frame_id = global_frame_id_;
  p.header.stamp = laser_scan->header.stamp;

  text_marker.header.frame_id = p.header.frame_id;
  text_marker.header.stamp = p.header.stamp;
  text_marker.ns = "amcl_pos_space";
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.color.r = 255;
  text_marker.color.g = 0;
  text_marker.color.b = 0;
  text_marker.color.a = 1;
  text_marker.scale.x = 0.05;
  text_marker.scale.y = 0.05;
  text_marker.scale.z = 0.05;
  text_marker.scale.z = 0.1;
  text_marker.lifetime = ros::Duration(1000);

  std::stringstream ss ;
  int int_v = best_weight_perc*100.0;
  ss << int_v;
  std::string str;
  ss >> str;
  text_marker.text = str ;

  // Copy in the pose
  p.pose.pose.position.x = bestPos.v[0];
  p.pose.pose.position.y = bestPos.v[1];

  text_marker.pose = p.pose.pose ;

  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(bestPos.v[2]),
                        p.pose.pose.orientation);
  // Copy in the covariance, converting from 3-D to 6-D
  pf_sample_set_t* set = pf_->sets + pf_->current_set;
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
      p.pose.covariance[6*i+j] = set->cov.m[i][j];
    }
  }
  // Report the overall filter covariance, rather than the
  // covariance for the highest-weight cluster
  //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
  p.pose.covariance[6*5+5] = set->cov.m[2][2];

  /*
     printf("cov:\n");
     for(int i=0; i<6; i++)
     {
     for(int j=0; j<6; j++)
     printf("%6.3f ", p.covariance[6*i+j]);
     puts("");
     }
   */
  ///将位姿、粒子集、协方差矩阵等进行更新、发布
  pose_pub_.publish(p);
  pos_marker_pub.publish(text_marker);

  last_published_pose = p;
  // subtracting base to odom from map to base and send map to odom instead
  tf::Stamped<tf::Pose> odom_to_map;
  try
  {
    //ROS_INFO("pubposandtf.bestpos info.x:%.4f",bestPos.v[0]);

    tf::Transform tmp_tf(tf::createQuaternionFromYaw(bestPos.v[2]),
                         tf::Vector3(bestPos.v[0],
                                     bestPos.v[1],
                                     0.0));

    //ROS_INFO("pubposandtf.map2base info.x:%.4f",tmp_tf.getOrigin().x());

    tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                          laser_scan->header.stamp,
                                          base_frame_id_);
   //transformPose Transform a Stamped Pose Message into the target frame
   //This can throw all that lookupTransform can throw as well as tf::InvalidTransform
    //void transformPose(
    //const std::string& target_frame,
    //const geometry_msgs::PoseStamped& stamped_in,
    //geometry_msgs::PoseStamped& stamped_out) const;
   //chq odom2map =odom2pos * pos2map = odom2pos * inv(map2pos)
    //transformPose所作的事　是　将base_link下的数据(pos2map)　转换到　目标　odom上去
    // 即　获得　odom 到base_link的转换　乘以 　base_link下的数据
    //ROS_INFO("pubposandtf.transformPose...");
    //ROS_INFO("pubposandtf.base2map info.x:%.4f",tmp_tf_stamped.getOrigin().x());

    this->tf_->transformPose(odom_frame_id_,
                             tmp_tf_stamped,
                             odom_to_map);
    //ROS_INFO("pubposandtf.odom2map info.x:%.4f",odom_to_map.getOrigin().x());

  }
  catch(tf::TransformException)
  {
    ROS_WARN("Failed to subtract base to odom transform");
    return;
  }
  //ROS_INFO("pubposandtf.odom_to_map info.x:%.4f",odom_to_map.getOrigin().x());

  latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                             tf::Point(odom_to_map.getOrigin()));
  latest_tf_valid_ = true;
  //ROS_INFO("pubposandtf.latest_tf_ info.x:%.4f",latest_tf_.getOrigin().x());
  if (tf_broadcast_ == true)
  {
    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Time transform_expiration = (laser_scan->header.stamp +
                                      transform_tolerance_);
    //chq map2odom = inv(odom2map)
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_id_, odom_frame_id_);
    //ROS_INFO("pubposandtf.sendTransform...");

    this->tfb_->sendTransform(tmp_tf_stamped);
    sent_first_transform_ = true;
  }

}
bool
AmclNode::judgeUpdateByMove(pf_vector_t pose,pf_vector_t old_pos,pf_vector_t& delta){

   delta.v[0] = pose.v[0] - old_pos.v[0];//pose: this time laser pose ,pf_odom_pose_:last time pose
   delta.v[1] = pose.v[1] - old_pos.v[1];
   delta.v[2] = angle_diff(pose.v[2], old_pos.v[2]);

   // See if we should update the filter
   bool update = fabs(delta.v[0]) > d_thresh_ ||
                 fabs(delta.v[1]) > d_thresh_ ||
                 fabs(delta.v[2]) > a_thresh_;
   update = update || m_force_update;
   m_force_update=false;
   return update;
}

void
AmclNode::resetAndForceUpdate(pf_vector_t pose){
  // Pose at last filter update
  pf_odom_pose_ = pose;

  // Filter is now initialized

  pf_init_ = true;
  ROS_INFO("laserReceived.set pf_init_ = true");
  // Should update sensor data
  for(unsigned int i=0; i < lasers_update_.size(); i++)
    lasers_update_[i] = true;///chq comment : force to update pf by laser data

  resample_count_ = 0;///chq comment: forbidden resampled

}
void
AmclNode::updateDeadRecking(pf_vector_t pose,pf_vector_t delta){
  //printf("pose\n");
  //pf_vector_fprintf(pose, stdout, "%.3f");

  AMCLOdomData odata;
  odata.pose = pose;
  // HACK
  // Modify the delta in the action data so the filter gets
  // updated correctly
  odata.delta = delta;

  // Use the action data to update the filter
  // 用运动模型来更新现有的每一个粒子的位姿（这里得到的只是当前时刻的先验位姿）
  odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

  // Pose at last filter update
  //this->pf_odom_pose = pose;
}
void
AmclNode::pubPfCloud(void){
  pf_sample_set_t* set = pf_->sets + pf_->current_set;
  ROS_INFO("AmclNode::laserReceived.particlecloud_pub_");
  geometry_msgs::PoseArray cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = global_frame_id_;
  cloud_msg.poses.resize(set->sample_count);
  for(int i=0;i<set->sample_count;i++)
  {
    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                             tf::Vector3(set->samples[i].pose.v[0],
                                       set->samples[i].pose.v[1], 0)),
                    cloud_msg.poses[i]);
  }
  particlecloud_pub_.publish(cloud_msg);
}
void
AmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  ROS_INFO("laserReceived.start");
  last_laser_received_ts_ = ros::Time::now();
  if( map_ == NULL ) {
    ROS_ERROR("AmclNode::laserReceived.map is null return!");//CHQ 20190128
    return;
  }

  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;
  laser_index = updateLaserVector(laser_scan);
  ROS_INFO("laserReceived.suc update laser frame");
  if(laser_index < 0)return;

  cur_laser_scan = laser_scan;

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
   ROS_INFO("laserReceived.getOdomPose...");
  if(!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }


  pf_vector_t delta = pf_vector_zero();
  if(pf_init_)
  {
    ROS_INFO("laserReceived.judgeUpdateByMove...");

    bool update = judgeUpdateByMove(pose,pf_odom_pose_,delta);
    // Set the laser update flags
    if(update)
      for(unsigned int i=0; i < lasers_update_.size(); i++)
        lasers_update_[i] = true;
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    resetAndForceUpdate(pose);
    force_publication = true;///chq comment : force to pub pose tf
  }
  else if(pf_init_ && lasers_update_[laser_index])
  {
    updateDeadRecking(pose,delta);
  }

  bool resampled = false;

  ///for  init loc by ga
  AMCLLaserData ldata;
  ROS_INFO("laserReceived.tailoringScan...");

  int result = tailoringScan(laser_scan,lasers_[laser_index],ldata);
  if(result < 0 ) return;
  glo_ldata = ldata;

  // If the robot has moved, update the filter
  if(lasers_update_[laser_index])
  {
    #if 0
    AMCLLaserData ldata;
    int result = tailoringScan(laser_scan,lasers_[laser_index],ldata);
    if(result < 0 ) return;
    #endif
    ROS_INFO("AmclNode::laserReceived.UpdateSensor...");
    updatePfByLaser(lasers_[laser_index],ldata);
    //evaluatePfByLaser(lasers_[laser_index],ldata);

    lasers_update_[laser_index] = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % resample_interval_))
    {
      //chq comment: force to resample pf.the pf_current set will be change
      ROS_INFO("laserReceived.pf_update_resample...");
      if(resampled_in_limit_area_)
        pf_update_resample_in_local_area(pf_);
      else
        pf_update_resample(pf_);
      //chq comment:force to pub pose tf
      resampled = true;
    }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    ROS_DEBUG("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    if (!m_force_update) {
      pubPfCloud();
    }
  }

  if(resampled || force_publication)
  {
    // 遍历所有粒子簇，找出权重均值最大的簇，
    // 其平均位姿就是我们要求的机器人后验位姿，到此一次循环已经所有完成
    // Read out the current hypotheses
       pf_vector_t bestPos;
       double best_w_perc =0.0;
      int result = getBestClusterSample(pf_,bestPos,best_w_perc);
      if( result >= 0 ){
        ROS_INFO("laserReceived.pubPoseAndTF...");
        pubPoseAndTF(laser_scan,bestPos,best_w_perc);
      }
      else
      {
        ROS_ERROR("laserReceived.getBestClusterSample.No pose!");
      }
  }
  else if(latest_tf_valid_)
  {
    if (tf_broadcast_ == true)
    {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      ros::Time transform_expiration = (laser_scan->header.stamp +
                                        transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                          transform_expiration,
                                          global_frame_id_, odom_frame_id_);
      ROS_INFO("laserReceived.sendTransform...");

      this->tfb_->sendTransform(tmp_tf_stamped);
    }

    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if((save_pose_period.toSec() > 0.0) &&
       (now - save_pose_last_time) >= save_pose_period)
    {
      this->savePoseToServer();
      save_pose_last_time = now;
    }
  }

}

double
AmclNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}
void
AmclNode::globalPoseReceived(const std_msgs::Int16ConstPtr & msg){
  if( map_ == NULL ) {
    ROS_ERROR("!!!AmclNode::globalPoseReceived.map is null. return!");//CHQ 20190128
    return ;
  }
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  //boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("globalPoseReceived.Initializing with global uniform distribution");
  //传入uniformPoseGenerator函数，在采样个数限定下,每次调用生成一个地图内的随机位置
 // pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
  //              (void *)map_);
  globalLocBySplitMap();

  ROS_INFO("globalPoseReceived initialisation done!");
}
void
AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  handleInitialPoseMessage(*msg);
}

void
AmclNode::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  pf_sample_set_t *set;
  set = pf_->sets + pf_->current_set;
  ROS_INFO("handleInitialPoseMessage.start pf samplecount:%d,cluster_count:%d",set->sample_count,set->cluster_count);
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if(msg.header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if(tf_->resolve(msg.header.frame_id) != tf_->resolve(global_frame_id_))
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             msg.header.frame_id.c_str(),
             global_frame_id_.c_str());
    return;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    ros::Time now = ros::Time::now();
    // wait a little for the latest tf to become available
    tf_->waitForTransform(base_frame_id_, msg.header.stamp,
                         base_frame_id_, now,
                         odom_frame_id_, ros::Duration(0.5));
    tf_->lookupTransform(base_frame_id_, msg.header.stamp,
                         base_frame_id_, now,
                         odom_frame_id_, tx_odom);
  }
  catch(tf::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  tf::poseMsgToTF(msg.pose.pose, pose_old);
  ///map2base * base2odom = map2oodm chq
  pose_new = pose_old * tx_odom;

  // Transform into the global frame

  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
           ros::Time::now().toSec(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           getYaw(pose_new));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = getYaw(pose_new);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6*5+5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  set = pf_->sets + pf_->current_set;
  ROS_INFO("handleInitialPoseMessage.pf samplecount:%d,cluster_count:%d",set->sample_count,set->cluster_count);
  applyInitialPose();
}
void AmclNode::applyInitialPose(AMCLLaser* lasers_amcl, AMCLLaserData& ldata,pf_vector_t pos){
  int i;

  pf_vector_t mean = pos;
  pf_vector_t mean_search;
  int max_sample = pf_->max_samples;
  pf_vector_t v_mean[max_sample];

  mean_search.v[2] = mean.v[2];
  double search_radius = 5.0;//距离搜索范围，值越大，搜索范围越大，可能导致的匹配误差越大
  std::pair<double,double> mid_pos = std::make_pair(mean.v[0],mean.v[1]);
  int set_size = generateRandomPfSet(v_mean,max_sample,mid_pos,search_radius,search_radius);
  if(!set_size){
    ROS_ERROR("!!!AmclNode::applyInitialPose.generateRandomPfSet failed.return");
    return;
  }
  resetAndClustingPfSet(pf_,max_sample,v_mean,false);
  evaluatePfByLaser(pf_,lasers_amcl,ldata);
  clustingPfSet(pf_,false);

  //pf_init_ = false;

}
/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void
AmclNode::applyInitialPose()
{

  if (init_loc_method == "rand_pf")
  {
  //#if 0
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
    if( initial_pose_hyp_ != NULL && map_ != NULL ) {
       int i;
       double search_radius = 5.0;//距离搜索范围
       pf_vector_t pose;
       pf_vector_t bestPos;
       pf_vector_t mean = initial_pose_hyp_->pf_pose_mean;
       pf_matrix_t cov = initial_pose_hyp_->pf_pose_cov;
       ROS_INFO("applyInitialPose.LocalLocBySplitMap");
       LocalLocBySplitMap(2*search_radius,2*search_radius,mean.v[0],mean.v[1]);
       delete initial_pose_hyp_;
       initial_pose_hyp_ = NULL;
    }
    //#endif
  }

  else if (init_loc_method == "ga"){
    //#if 0
    ///method 0 ga alogrithm
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
    if( initial_pose_hyp_ != NULL && map_ != NULL ) {
      LocalLocByGaAlogrithm(ga_search_radius,initial_pose_hyp_->pf_pose_mean);
      delete initial_pose_hyp_;
      initial_pose_hyp_ = NULL;
   }
  //#endif
  }
  else //raw or other ,use raw method
  {
    //#if 0
      boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
      if( initial_pose_hyp_ != NULL && map_ != NULL ) {
        pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
        pf_init_ = false;

        delete initial_pose_hyp_;
        initial_pose_hyp_ = NULL;
      }
    //#endif
  }
}
