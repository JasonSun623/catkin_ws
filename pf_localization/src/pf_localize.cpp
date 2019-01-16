#include "pf_localize.h"
namespace pf_localization_space {


pfLocalize::pfLocalize(){
_re_deadrecking = false;
_run_loc_thread = false;
ros::NodeHandle private_nh("~");

 ///for filter rfs center---start
 private_nh.param<int>("scan_update_fre",scan_update_fre,30);
 private_nh.param<int>("fiter_scan_num", _fiter_scan_num, 1);

 private_nh.param<std::string>("pub_rfs_topic", pub_rfs_topic, "pf_reflectors");
 private_nh.param<std::string>("scan_frame", scan_frame, "hokuyo_link");
 private_nh.param<std::string>("base_frame", base_frame, "base_footprint");
 private_nh.param<std::string>("odom_frame", odom_frame, "odom");
 private_nh.param<std::string>("map_frame",map_frame,"/map");
 private_nh.param<std::string>("scan_topic", scan_topic, "filter_scan");
///for filter rfs center---end


private_nh.param<std::string>("map_name",map_name,"/home/hdros/catkin_ws/src/pf_localization/map/rfs.map");
private_nh.param<std::string>("pos_topic",pos_topic,"pose");
private_nh.param<std::string>("recking_pos_topic",recking_pos_topic,"pose");
private_nh.param<std::string>("rfs_topic",rfs_topic,"pf_reflectors");
private_nh.param<std::string>("odom_topic",odom_topic,"/wheel_diff_controller/odom");
private_nh.param("tf_broadcast", tf_broadcast_, true);

private_nh.param<double>("perimeter_rang1",lrange1,0.05);
private_nh.param<double>("perimeter_rang2",lrange2,0.1);
private_nh.param<double>("perimeter_rang3",lrange3,0.2);
private_nh.param<double>("perimeter_rang4",lrange4,0.3);

private_nh.param<double>("side_var_rang1",dxrange1,0.05);
private_nh.param<double>("side_var_rang2",dxrange2,0.1);
private_nh.param<double>("side_var_rang3",dxrange3,0.2);
private_nh.param<double>("side_var_rang4",dxrange4,0.3);

private_nh.param<double>("p_rang1",prange1,0.05);
private_nh.param<double>("p_rang2",prange2,0.1);
private_nh.param<double>("p_rang3",prange3,0.2);
private_nh.param<double>("p_rang4",prange4,0.3);

private_nh.param<double>("k_perimeter_rang1",kl1,0.5);
private_nh.param<double>("k_perimeter_rang2",kl2,0.35);
private_nh.param<double>("k_perimeter_rang3",kl3,0.1);
private_nh.param<double>("k_perimeter_rang4",kl4,0.05);

private_nh.param<double>("k_side_var1",kdx1,0.5);
private_nh.param<double>("k_side_var2",kdx2,0.35);
private_nh.param<double>("k_side_var3",kdx3,0.1);
private_nh.param<double>("k_side_var4",kdx4,0.05);

private_nh.param<double>("k_p_rang1",kp1,0.5);
private_nh.param<double>("k_p_rang2",kp2,0.35);
private_nh.param<double>("k_p_rang3",kp3,0.1);
private_nh.param<double>("k_p_rang4",kp4,0.05);

private_nh.param<double>("init_pos_x",init_pos.x,0.0);
private_nh.param<double>("init_pos_y",init_pos.y,0.0);
private_nh.param<double>("init_pos_angle",init_pos.theta,0.0);//rad

private_nh.param<double>("triangle_side_min_len",triangle_side_min_len,2);
private_nh.param<double>("triangle_grouping_thread",triangle_grouping_thread,20);
private_nh.param<double>("search_triangle_thread",search_triangle_thread,20);
private_nh.param<double>("match_angle_thread",match_angle_thread,10);//deg
private_nh.param<double>("match_dist_thread",match_dist_thread,0.4);
private_nh.param<double>("score_thread",score_thread,0.9);
private_nh.param<int>("scan_update_fre",scan_update_fre,30);
private_nh.param("transform_tolerance", tmp_tol, 0.1);
transform_tolerance_.fromSec(tmp_tol);
tfb_ = new tf::TransformBroadcaster();
tf_ = new TransformListenerWrapper();

//filter(FilterRfsCenter(_judge_by_dist,_rf_radius,_echo,_step,_err));

timer= nh.createTimer(ros::Duration(0.01), boost::bind(&pfLocalize::deadReckingThread,this));
timer_cal= nh.createTimer(ros::Duration(0.06), boost::bind(&pfLocalize::calGlobalPosThread,this));
pub_rfs_center = nh.advertise<geometry_msgs::PoseArray>(pub_rfs_topic,1,true);
scan_sub = nh.subscribe(scan_topic,100,&pfLocalize::callBackScan,this);
//rfs_sub = nh.subscribe(rfs_topic,1,&pfLocalize::callBackRelativeRfs,this);
odom_sub = nh.subscribe(odom_topic,1,&pfLocalize::callbackOdom,this);
global_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(pos_topic,1);
global_recking_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(recking_pos_topic,1);

ROS_INFO_STREAM("pf_Localize.construct.init done!");

}

void pfLocalize::localize(){
  loadRfsMap(map_name);
  createTriangleTemplate();
  recking_pos = init_pos;
  run();
  //boost::thread recking_thread(boost::bind(&pfLocalize::deadReckingThread,this));
  //boost::thread calpos_thread(boost::bind(&pfLocalize::calGlobalPosThread,this));
}

void pfLocalize::callbackOdom(const nav_msgs::Odometry::ConstPtr& state_odata){
  boost::mutex::scoped_lock l(odom_mut);
  const nav_msgs::Odometry odo = (*state_odata);
  m_cur_vec_x = odo.twist.twist.linear.x;
  m_cur_vec_y = odo.twist.twist.linear.y;
  m_cur_w = odo.twist.twist.angular.z;
}
void  pfLocalize::callBackScan(const sensor_msgs::LaserScan & scan){
  boost::mutex::scoped_lock l(scan_lock_);
  static CountTime timer;//compensate the time scan received delay
  static int push_num = 0;
  if( push_num < _fiter_scan_num ){
    raw_scan.push_back(scan);
    push_num++;
  }
  else{
    raw_scan.pop_back();
    raw_scan.push_back(scan);
  }

  //cal the cneter of rfs
  filter.getReflectorsCenter(raw_scan,v_optrfs_center);
  timer.end();
  double dt = timer.getTime()/1000.0;
  //补偿不同帧激光点采样时间存在的不同时间差　以及采样后到接收到存在的相同时间差
  compensateScanRecDelay(v_optrfs_center,dt);
  ROS_DEBUG("pfLocalize::callBackScan.the scan reced delay time:%.6f(ms)",dt*1000);
  std::vector<VecPosition> temp_rfs;
  rfs_pos_pub.poses.clear();
  rfs_pos_pub.header.frame_id = scan_frame;
  geometry_msgs::Pose pose;
  for (int i = 0;i < v_optrfs_center.size(); i++){
    temp_rfs.push_back( VecPosition(v_optrfs_center[i]) );
    pose.position.x = v_optrfs_center[i].getX();
    pose.position.y = v_optrfs_center[i].getY();
    pose.position.z = 0;
    rfs_pos_pub.poses.push_back(pose);
  }
  //pub mea rfs
  pub_rfs_center.publish(rfs_pos_pub);
  _rfs=temp_rfs;
  //getPubPos(v_optrfs_center,rfs_pos,items_MarkerArray);
  //ROS_INFO("pfLocalize.rate:%.3f(ms).rfs num:%d",dt*1000,v_optrfs_center.size());
  timer.begin();
  //记录每次计算到反光板时的时间，在全局定位线程中会终止此定时器，用于补偿从反光板计算到　到　使用　存在的时间差　导致的反光板测量时移
  compensateTimeStart();
}

/*void pfLocalize::callBackRelativeRfs(const geometry_msgs::PoseArray::ConstPtr& rfs_data){
  boost::mutex::scoped_lock l(rfs_mut);
  static std::vector<VecPosition> temp_rfs;
  temp_rfs.clear();
  geometry_msgs::PoseArray v_pos = *rfs_data;
  for (int i = 0;i < v_pos.poses.size(); i++){
    temp_rfs.push_back( VecPosition(v_pos.poses[i].position.x,v_pos.poses[i].position.y) );
  }
  _rfs=temp_rfs;
  //when the rfs msg arrived we start the compensate_timer
  //when we used the rfs msg ,we stop then timer
  compensate_timer.begin();

}*/

void pfLocalize::pubMapToThisTF(geometry_msgs::Pose2D cur_pos ){
  static CountTime pub_waster_t,loop_pub;
  pub_waster_t.begin();
  loop_pub.end();
  double dt_loop = loop_pub.getTime();
  loop_pub.begin();
  ros::Time cur_time = ros::Time::now();
  tf::Stamped<tf::Pose> odom_to_map;
  try
  {

    tf::Transform tmp_tf(tf::createQuaternionFromYaw(cur_pos.theta),tf::Vector3(cur_pos.x,cur_pos.y,0) );///map2base
    tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                          ros::Time(0),///chq!!!这个时间参数很重要，设置不当会导致transformPose转换失败
                                          base_frame);///base2map
   //transformPose Transform a Stamped Pose Message into the target frame
   //This can throw all that lookupTransform can throw as well as tf::InvalidTransform
    //void transformPose(
    //const std::string& target_frame,
    //const geometry_msgs::PoseStamped& stamped_in,
    //geometry_msgs::PoseStamped& stamped_out) const;
   //chq odom2map =odom2pos * pos2map = odom2pos * inv(map2pos)
    //transformPose所作的事　是　将base下的数据(pos2map)　转换到　目标　odom上去
    // 即　获得　odom 到base的转换　乘以base下的数据
    this->tf_->transformPose(odom_frame,
                             tmp_tf_stamped,
                             odom_to_map);///odom2base* base2map =  odom2map
  }
  catch(tf::TransformException)
  {
    ROS_ERROR("pfLocalize::pubMapToThisTF.Failed to subtract odom2map transform");
    return;
  }

  latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                             tf::Point(odom_to_map.getOrigin()));
  latest_tf_valid_ = true;

  if (tf_broadcast_ == true)
  {
    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Time transform_expiration = (cur_time +
                                      transform_tolerance_);
    //chq map2odom = inv(odom2map)
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        //transform_expiration,
                                        ros::Time::now(),
                                        map_frame, odom_frame);
    this->tfb_->sendTransform(tmp_tf_stamped);
  }
  pub_waster_t.end();

  double dt = pub_waster_t.getTime();
  ///ROS_INFO("pf_Localize.pubmaptothisTF.done!pub waster:%.6f,loop time:%.6f",dt,dt_loop);
}

///map2base , not odom2base(recking by gazebo or  real robot wheel controller)
void pfLocalize::deadReckingThread(){

  double delta_x,delta_y,theta,cur_theta,delta_theta,dt;
  double r_x,r_y;
  static CountTime gcount_time;
  if(!_run_loc_thread){
    ROS_WARN("wait for running.do not start pf_Localize dead recking thread");
    return;
   }
    gcount_time.end();
    dt = gcount_time.getTime()/1000;//getTime is ms
    gcount_time.begin();
    //TODO need to optimize
    ///ROS_INFO("pfLocalize::deadReckingThread.loop period(ms):%.6f",dt*1000);
    if(_re_deadrecking){
      ///更新为全局坐标时，会打乱里程计的计时，从而影响到位置，角度的推算，
      ///从而影响反光板的匹配，间接影响定位精度～！！！！(important!!!)
      ///
      /// 目前的处理是，一旦有全局位置更新进来，丢失上次计时（因为上次的参考点已经被更新了），
      /// 更新本次位置为反光板匹配位置
      double dx = recking_pos.x-global_pos.x;
      double dy = recking_pos.y-global_pos.y;
      ROS_DEBUG("pfLocalize::deadReckingThread.upate global pos to recking pos!err dx,dy:(%.6f,%.6f)",dx,dy);
      recking_pos = global_pos;
      pubMapToThisTF(recking_pos);
      _re_deadrecking = false;
      //if need upodate pos,reset clock
      gcount_time.end();
      gcount_time.begin();
      return;
    }

    //已知：x,y为初始坐标，theta为初始角，(vx,vy),vtheta 为线速度和角速度
    theta = recking_pos.theta;

    double temp_angle = theta;
    static double last_angle_result=temp_angle;
    double dec = last_angle_result-temp_angle;
    dec =atan2(sin(dec),cos(dec));
    if(fabs(dec) > M_PI/3  ){
      ROS_WARN("pfLocalize::deadReckingThrea.angle delta is beyond thread.may be meet error!");
    }
    else{
      last_angle_result= temp_angle;
    }

    delta_theta = m_cur_w * dt;
    cur_theta = theta + delta_theta;
    if ( fabs( m_cur_w ) < EPSILON )
    {// fit the condition that there is no rotate speed
      delta_x = (m_cur_vec_x * cos(theta) - m_cur_vec_y * sin(theta) ) * dt;
      delta_y = (m_cur_vec_x * sin(theta) + m_cur_vec_y * cos(theta) ) * dt;
    }
    else
    {
       r_x = m_cur_vec_x/m_cur_w;
       r_y = m_cur_vec_y/m_cur_w;

      delta_x = (  r_x * ( sin( cur_theta ) - sin( theta ) ) + r_y * ( cos( cur_theta ) - cos( theta ) ) ) ;
      delta_y = ( -r_x * ( cos( cur_theta ) - cos( theta ) ) + r_y * ( sin( cur_theta ) - sin( theta ) ) ) ;//注意，y方向上，前后偏移差值再取负，因为投影方向是向上为正
    }
     recking_pos.x += delta_x;
     recking_pos.y += delta_y;
     recking_pos.theta = atan2(sin(cur_theta),cos(cur_theta));
     ROS_DEBUG("pf_Localize.cur deckrecking pos (%.6f,%.6f),rot_x radius:%.6f,ang(deg):%.3f",recking_pos.x,recking_pos.y,r_x,Rad2Deg( recking_pos.theta));
     //static CountTime pubtf_t;
     //pubtf_t.begin();
     pubMapToThisTF(recking_pos);
     //pubtf_t.end();
    // double dt_tf = pubtf_t.getTime();
     //ROS_INFO("pf_local.pub tf waster(ms):%.6f",dt_tf);
     ros::spinOnce();
}
void pfLocalize::loadRfsMap(string name){
  map_rfs.clear();
  double x,y;
  std::ifstream ifs;
  ifs.open(name.c_str());
  if(!ifs) {
    ROS_INFO_STREAM("pfLocalize::loadRfsMap.FATAL: map file: "<< name << " not exist!!");
  }
  assert(ifs);
  while(ifs >> x){
    ifs >> y;
    map_rfs.push_back(VecPosition((double)x/1000.0,(double)y/1000.0));
  }
  ROS_INFO("pfLocalize::loadRfsMap.finish!");
}
comparedata pfLocalize::calCoefficient(VecPosition v_a,VecPosition v_b,VecPosition v_c){
  double i_j_dist = v_a.getDistanceTo(v_b);
  double i_k_dist = v_a.getDistanceTo(v_c);
  double j_k_dist = v_b.getDistanceTo(v_c);
  double len = i_j_dist+i_k_dist+j_k_dist;
  double mean = len/3.0;
  double dx = (pow(i_j_dist-mean,2)+pow(i_k_dist-mean,2)+pow(j_k_dist-mean,2))/3;
  double max_side = max(max(i_j_dist,i_k_dist),j_k_dist);
  double min_side = min(min(i_j_dist,i_k_dist),j_k_dist);
  double p = max_side/min_side;
  return comparedata(len,dx,p);
}


void pfLocalize::createTriangleTemplate(){
  int map_rfs_size = map_rfs.size();
  double i_j_dist,i_k_dist,j_k_dist;
  double len,mean,dx,max_side,min_side,p;
  Line a;
  for(int i = 0; i < map_rfs_size-2;i++){
    for(int j = i+1; j < map_rfs_size-1 ;j++ ){
      i_j_dist = map_rfs[j].getDistanceTo(map_rfs[i]);
      if( i_j_dist > triangle_grouping_thread || i_j_dist < triangle_side_min_len  ){
        if(i_j_dist < triangle_side_min_len)
          ROS_WARN_STREAM("WARNIGN! pfLocalize::createTriangleTemplate.dist bet rfs index i:" << i << " and j:" << j << " dist:" << i_j_dist << " do not fit in thread.continue");
        continue;
      }
      for(int k = j+1; k < map_rfs_size; k++ ){
        i_k_dist = map_rfs[k].getDistanceTo(map_rfs[i]);
        if( i_k_dist <= triangle_grouping_thread &&i_k_dist >= triangle_side_min_len ){
          a = Line::makeLineFromTwoPoints(map_rfs[i],map_rfs[j]);
          double dist = a.getDistanceWithPoint(map_rfs[k]);
          if( dist > EPSILON ){//如果不共线，则可以构成三角形
            int arr[3]={i,j,k};
            set<int> set_index(arr,arr+3);
             j_k_dist = map_rfs[j].getDistanceTo(map_rfs[k]);
             if(j_k_dist < triangle_side_min_len ){
               ROS_WARN_STREAM("WARIGN! pfLocalize::createTriangleTemplate.dist bet rfs index j: " << j << " and k: " << k << " dist:" << j_k_dist << " do not fit in thread.continue");
               continue;
             }
              len = i_j_dist+i_k_dist+j_k_dist;
              mean = len/3.0;
              dx = (pow(i_j_dist-mean,2)+pow(i_k_dist-mean,2)+pow(j_k_dist-mean,2))/3;
              max_side = max(max(i_j_dist,i_k_dist),j_k_dist);
              min_side = min(min(i_j_dist,i_k_dist),j_k_dist);
              p = max_side/min_side;
             comparedata cp(len,dx,p);
             _triangle_template.insert(std::pair<set<int>,comparedata>(set_index,cp) );
          }
      }
        else{
          if(i_k_dist < triangle_side_min_len)
            ROS_WARN_STREAM("WARIGN! pfLocalize::createTriangleTemplate.dist bet rfs index i:" << i << " and k:" << k << " dist:" << i_k_dist << " do not fit in thread.continue");
        }
      }

    }
  }

}

 void pfLocalize::getMatchedMeaRfs(geometry_msgs::Pose2D pos,
                                   std::vector<VecPosition> mea_rfs,
                                   std::vector<std::pair<int,int> > &matched_mea_rfs){
//1,TODO given map2base,how to cal map2odom(we only know the relationship bet odom ->base->...->scan)
// caled in scan abs frame
  VecPosition abs_mea_rfs;
  std::vector<VecPosition> cur_map_rfs;
  {
     boost::mutex::scoped_lock l(temp_mut);
     cur_map_rfs= map_rfs;
  }


  VecPosition v_scan_pos(pos.x,pos.y);//map2scan
  std::vector<int> candidate_rfs;//record candidate rfs index
  std::vector<VecPosition> relative_candidate_rfs;//the cord pos in scan frame
  std::list<std::pair<int,VecPosition> > flash_candidate_rfs;//temp

  bool dis_match,ang_match;
  matched_mea_rfs.clear();

  //小于匹配搜索半径，作为待匹配项(store index!!!)
  for(int i= 0;i < cur_map_rfs.size();i++){
    if( v_scan_pos.getDistanceTo(cur_map_rfs[i]) < search_triangle_thread )
      candidate_rfs.push_back(i);//record index instead of value
  }

  //rot to scan relative cord for comparing
  for(int i= 0;i < candidate_rfs.size(); i++){
    VecPosition p = cur_map_rfs[candidate_rfs[i]]-v_scan_pos;
    p.rotate(-Rad2Deg(pos.theta));
    relative_candidate_rfs.push_back(p);
  }

  //store the candidate pair<index,relative cord in scan frame>
  for(int i=0;i < candidate_rfs.size();i++){
    flash_candidate_rfs.push_back( std::make_pair(candidate_rfs[i],relative_candidate_rfs[i]) );
  }

  double rel_dist[mea_rfs.size()][flash_candidate_rfs.size()];
  double rel_angle[mea_rfs.size()][flash_candidate_rfs.size()];
  //try to match every measured rfs
  std::list<std::pair<int,VecPosition>>::iterator itr;
  for(int i=0; i < mea_rfs.size(); i++){
    itr = flash_candidate_rfs.begin();
    int cnt =0;
    for(;itr != flash_candidate_rfs.end();itr++ ){

      rel_dist[i][cnt] = mea_rfs[i].getDistanceTo((*itr).second);
      rel_angle[i][cnt]= fabs(VecPosition::normalizeAngle(mea_rfs[i].getDirection() - ((*itr).second).getDirection()));

      dis_match = rel_dist[i][cnt] <= match_dist_thread;
      ang_match = rel_angle[i][cnt] <= match_angle_thread;
      cnt++;
      if( dis_match && !ang_match ){
        ROS_ERROR("pfLocalize::getMatchedMeaRfs.meet error:dis_match but angle not matched!");
      }
      if( dis_match && ang_match ){
        abs_mea_rfs = mea_rfs[i];
        abs_mea_rfs.rotate(Rad2Deg(pos.theta));
        abs_mea_rfs += v_scan_pos;//convert to abs cord,we will score them later
        matched_mea_rfs.push_back(std::make_pair((*itr).first,i) );//store the rfs index in map and the index  of the measured rfs
        flash_candidate_rfs.erase(itr);
        break;
      }
    }
  }
  if( matched_mea_rfs.size() && matched_mea_rfs.size() < 3 ){
    ROS_ERROR("WARNNING!!pfLocalize::getMatchedMeaRfs size below 3!scan_pos(%.6f.%.6f).measured rfs size:%d, comp data:",pos.x,pos.y,mea_rfs.size());
    for(int i=0; i < mea_rfs.size(); i++){
      abs_mea_rfs = mea_rfs[i];
      abs_mea_rfs.rotate(Rad2Deg(pos.theta));
      abs_mea_rfs += v_scan_pos;//convert to abs cord,we will score them later
      ROS_INFO("abs mea index:%d,pos:(%.6f,%.6f)",i,abs_mea_rfs.getX(),abs_mea_rfs.getY());
      itr = flash_candidate_rfs.begin();
      int cnt = 0;
      for( ;itr != flash_candidate_rfs.end(); itr++ ){
        VecPosition p = cur_map_rfs[itr->first];
        ROS_INFO("abs map pos:(%.6f,%.6f),dist:%.6f,angle_deg:%.6f",p.getX(),p.getY(),rel_dist[i][cnt],rel_angle[i][cnt]);
        cnt++;
      }
    }

 }
  if( matched_mea_rfs.empty() ){
    ROS_ERROR("pfLocalize::getMatchedMeaRfserror to match!");
  }
}

 double pfLocalize::getScore( comparedata & comp){
   double sum = 0.0;
   double klen = 0.0;
   double kdx = 0.0;
   double kp = 0.0;
   double len = comp.getLen();
   len =fabs(len);
   if( len < lrange1 ){
     sum+=kl1;
   }else
   if( len < lrange2 ){
     sum+=kl2;
   }else
   if( len < lrange3 ){
     sum+=kl3;
   }else
   if( len < lrange4 ){
     sum+=kl4;
   }

   double dx =comp.getDx();
   dx= fabs(dx);
   if( dx < dxrange1 ){
     sum+=kdx1;
   }else
   if( dx < dxrange2 ){
     sum+=kdx2;
   }else
   if( dx < dxrange3 ){
     sum+=kdx3;
   }else
   if( dx < dxrange4 ){
     sum+=kdx4;
   }
   double p = comp.getP();
   p = fabs(p);
   if( p < prange1 ){
     sum+=kp1;
   }else
   if( p < prange2 ){
     sum+=kp2;
   }else
   if( p < prange3 ){
     sum+=kp3;
   }else
   if( p < prange4 ){
     sum+=kp4;
   }
   return sum;
 }

 int pfLocalize::getMinIndex( std::vector< std::vector<std::pair<int,int> > >& group){
  int k = 0;
  //double var_k,var_i,dist0,dist1,dist2,mean_dist;
  double angle0,angle1,angle2,rel_ang01,rel_ang02,rel_ang12,ang_delta,ang_delta_k = 0;
  angle0=mea_rfs[group[0][0].second].getDirection();
  angle1=mea_rfs[group[0][1].second].getDirection();
  angle2=mea_rfs[group[0][2].second].getDirection();
  //set to 0~360
  if(angle0<0)angle0+=360;
  if(angle1<0)angle1+=360;
  if(angle2<0)angle2+=360;

  rel_ang01 = fabs(angle0-angle1);
  rel_ang02 = fabs(angle0-angle2);
  rel_ang12 = fabs(angle1-angle2);
  //取角度小于１８0的角
  if(rel_ang01>180)rel_ang01=360-rel_ang01;
  if(rel_ang02>180)rel_ang02=360-rel_ang02;
  if(rel_ang12>180)rel_ang12=360-rel_ang12;
  //每个三角形夹角最大减去最小的偏差越小，三角形形状越偏向于等边三角形
  //（按照距离可能会出现三个点都偏离中心点（如落在以激光头为中心，测距为半径的圆上），非等边形）
  ang_delta_k = pow(rel_ang01-120,2)+pow(rel_ang02-120,2)+pow(rel_ang12-120,2);

  //
  //dist0=mea_rfs[group[0][0].second].getMagnitude();
  //dist1=mea_rfs[group[0][1].second].getMagnitude();
  //dist2=mea_rfs[group[0][2].second].getMagnitude();
  //mean_dist =(dist0+dist1+dist2)/3.;
  //var_k = pow(dist0-mean_dist,2)+pow(dist1-mean_dist,2)+pow(dist2-mean_dist,2);

  for(int i = 1; i < group.size(); i++)//limit the size is 5
  {
    angle0=mea_rfs[group[i][0].second].getDirection();
    angle1=mea_rfs[group[i][1].second].getDirection();
    angle2=mea_rfs[group[i][2].second].getDirection();
    //all set to 0~360
    if(angle0<0)angle0+=360;
    if(angle1<0)angle1+=360;
    if(angle2<0)angle2+=360;

    rel_ang01 = fabs(angle0-angle1);
    rel_ang02 = fabs(angle0-angle2);
    rel_ang12 = fabs(angle1-angle2);
    //取角度小于１８0的角
    if(rel_ang01>180)rel_ang01=360-rel_ang01;
    if(rel_ang02>180)rel_ang02=360-rel_ang02;
    if(rel_ang12>180)rel_ang12=360-rel_ang12;
    //每个三角形夹角最大减去最小的偏差越小，三角形形状越偏向于等边三角形
    //（按照距离可能会出现三个点都偏离中心点（如落在以激光头为中心，测距为半径的圆上），非等边形）
    ang_delta = pow(rel_ang01-120,2)+pow(rel_ang02-120,2)+pow(rel_ang12-120,2);

    if( ang_delta < ang_delta_k ){
      k = i;
      ang_delta_k = ang_delta;
    }
  }
  return k;
  }

 int pfLocalize::getOptimizeTriangle(geometry_msgs::Pose2D pos,std::vector<std::pair<int,int> > matched_rfs,
                                      std::vector<std::pair<int,int> >& best){
  int size = matched_rfs.size();
   //std::pair<set<int>,VecPosition> first:the rfs index in map,second: the measured rfs abs cord
  std::vector<std::vector<std::pair<int,int> > > good_group;
  std::map<set<int>,comparedata> cur_triangle_template;
  {
    boost::mutex::scoped_lock l(temp_mut);
    cur_triangle_template = _triangle_template;
  }
    //获得评分超过阈值的若干三角形
  VecPosition v_scan_pos(pos.x,pos.y);
  std::vector<VecPosition > _mea_rfs = mea_rfs;//value transform
  double cur_scan_angle = pos.theta;
   for(int i = 0; i < size-2; i++){
     for(int j = i+1; j < size-1; j++){
       for(int k = j+1; k < size; k++){
         int a[3]={matched_rfs[i].first,matched_rfs[j].first,matched_rfs[k].first};//get the index
         std::set<int> _set(a,a+3);
         if(cur_triangle_template.find(_set)!=cur_triangle_template.end()){
           //计算测量反光板的系数
           VecPosition v_a = _mea_rfs[(matched_rfs[i]).second].rotate(Rad2Deg(cur_scan_angle));
           v_a+=v_scan_pos;
           VecPosition v_b = _mea_rfs[(matched_rfs[j]).second].rotate(Rad2Deg(cur_scan_angle));
           v_b+=v_scan_pos;
           VecPosition v_c = _mea_rfs[(matched_rfs[k]).second].rotate(Rad2Deg(cur_scan_angle));
           v_c+=v_scan_pos;
           comparedata mea_rfs_triangle = calCoefficient(v_a,v_b,v_c);
           comparedata comp = mea_rfs_triangle - cur_triangle_template[_set];
          //超过一定阈值的放入待筛选列表
           //if( getScore(comp) > score_thread)//disable temp
           {
             std::vector<std::pair<int,int> > one_group;
             one_group.push_back(matched_rfs[i]);
             one_group.push_back(matched_rfs[j]);
             one_group.push_back(matched_rfs[k]);
             good_group.push_back(one_group);//store mea_rfs_triangle to comp side relation next step
           }
         }
       }
     }
   }

   //找出最优的三角形(等边最好)
   int best_index;
   if(good_group.size()){
     best_index = getMinIndex(good_group);
     best = good_group[best_index];
     return 1;
   }
   else
     return -1;
 }
 double pfLocalize::getOptimizeAngle(std::vector<std::pair<int,int> > result,VecPosition cord_result){
   double angle_result;
    std::vector<VecPosition> cur_map_rfs = map_rfs;
   VecPosition v_a = cur_map_rfs[result[0].first];
   VecPosition v_b = cur_map_rfs[result[1].first];
   VecPosition v_c = cur_map_rfs[result[2].first];
   VecPosition relative_v_a = mea_rfs[result[0].second];
   VecPosition relative_v_b = mea_rfs[result[1].second];
   VecPosition relative_v_c = mea_rfs[result[2].second];
   v_a -= cord_result;
   v_b -= cord_result;
   v_c -= cord_result;
   //get relative angle
   double r_a_angle = relative_v_a.getDirection();
   double r_b_angle = relative_v_b.getDirection();
   double r_c_angle = relative_v_c.getDirection();

   double theta[3];
   //atan2(v_a.getY(),v_a.getX()) the abs angle bet  the map rfs and  measured rfs
    theta[0] = Deg2Rad( v_a.getDirection()-r_a_angle );
    theta[1] = Deg2Rad( v_b.getDirection()-r_b_angle );
    theta[2] = Deg2Rad( v_c.getDirection()-r_c_angle );
   //nomorlize angle
    for(int i=0;i<3;i++)
     theta[i] = atan2(sin(theta[i]),cos(theta[i]));

    double sum_neg=0.0;double sum_pos=0.0;
    int cnt_neg=0;int cnt_pos=0;

    for(int i=0;i<3;i++){
      if(theta[i]<0){
        sum_neg+=theta[i];
        cnt_neg++;
      }
      else{
        sum_pos+=theta[i];
      }
    }
   cnt_pos=3-cnt_neg;
   if(cnt_neg>1)sum_neg/=cnt_neg;
   if(cnt_pos>1)sum_pos/=cnt_pos;
   //如果有正有负，负的归一化到０－２*pi,两者相减，超过pi,正角减去平均，否则正角加上平均
   if( sum_neg*sum_pos < 0){
     double delta_pos=(sum_pos-sum_neg)/2;
     double temp_neg=sum_neg+2*M_PI;
     if(temp_neg-sum_pos>M_PI)
       angle_result-=delta_pos;
     else
       angle_result+=delta_pos;
   }
   else{
     angle_result=cnt_neg>0?sum_neg:sum_pos;
   }

   angle_result = atan2(sin(angle_result),cos(angle_result));

   //double temp_angle = (angle_result <-0.5*M_PI)?(angle_result+2*M_PI):angle_result;
   double temp_angle = angle_result;
   static double last_angle_result=temp_angle;
   double dec = last_angle_result-temp_angle;
   dec =atan2(sin(dec),cos(dec));
   if(fabs(dec) > M_PI/3  ){
     ROS_INFO("pfLocalize::getOptimizeAngle.the delta bet last angle and this angle is beyond thread,may be meet error!");
   }
   else{
     last_angle_result= temp_angle;
   }

   return angle_result;
 }
 void pfLocalize::compensateScanRecDelay(std::vector<VecPositionDir>& mea_rfs,double loop_time){

   static double every_angle_waster_t = (1.0/(double)scan_update_fre)/360;
   static double min_scan_period = (1.0/(double)scan_update_fre);
   if(loop_time < min_scan_period)return ;//below to scan period we no not compensate

   std::vector<VecPositionDir> temp_rfs;
   for(int i =0 ;i < mea_rfs.size();i++){
     double cur_angle = mea_rfs[i].getDirection();
     if( cur_angle < 0 ) cur_angle+=360;
     double dt = (loop_time - min_scan_period)+(360-cur_angle)*every_angle_waster_t;

     double theta = 0;
     double delta_theta = m_cur_w * dt;//因为里程计更新速度足够快，我们认为里程计是准的,忽略了延迟
     double cur_theta = delta_theta+theta ;
     double delta_x,delta_y;

     if ( fabs( m_cur_w ) < EPSILON )
     {// fit the condition that there is no rotate speed
       delta_x = (m_cur_vec_x * cos(theta) - m_cur_vec_y * sin(theta) ) * dt;
       delta_y = (m_cur_vec_x * sin(theta) + m_cur_vec_y * cos(theta) ) * dt;
     }
     else
     {
       double r_x = m_cur_vec_x/m_cur_w;
       double r_y = m_cur_vec_y/m_cur_w;

       delta_x = (  r_x * ( sin( cur_theta ) - sin( theta ) ) + r_y * ( cos( cur_theta ) - cos( theta ) ) ) ;
       delta_y = ( -r_x * ( cos( cur_theta ) - cos( theta ) ) + r_y * ( sin( cur_theta ) - sin( theta ) ) ) ;//注意，y方向上，前后偏移差值再取负，因为投影方向是向上为正
     }
     VecPosition scan_pre=mea_rfs[i];
     VecPosition new_origin(delta_x,delta_y);
     VecPosition scan_new = scan_pre.globalToRelative(new_origin,Rad2Deg( cur_theta) );
     temp_rfs.push_back(VecPositionDir(scan_new,mea_rfs[i].angle()));
   }
   mea_rfs = temp_rfs;
 }

 void pfLocalize::compensateRfsUsedDelay(std::vector<VecPosition>& mea_rfs,double dt){
   if(dt < EPSILON)return ;//below to scan period we no not compensate
   std::vector<VecPosition> temp_rfs;
   for(int i =0 ;i < mea_rfs.size();i++){
     double theta =0;
     double delta_theta = m_cur_w * dt;//因为里程计更新速度足够快，我们认为里程计是准的,忽略了延迟
     double cur_theta = delta_theta+theta ;
     double delta_x,delta_y;
     if ( fabs( m_cur_w ) < EPSILON )
     {// fit the condition that there is no rotate speed
       delta_x = (m_cur_vec_x * cos(theta) - m_cur_vec_y * sin(theta) ) * dt;
       delta_y = (m_cur_vec_x * sin(theta) + m_cur_vec_y * cos(theta) ) * dt;
     }
     else
     {
       double r_x = m_cur_vec_x/m_cur_w;
       double r_y = m_cur_vec_y/m_cur_w;
       delta_x = (  r_x * ( sin( cur_theta ) - sin( theta ) ) + r_y * ( cos( cur_theta ) - cos( theta ) ) ) ;
       delta_y = ( -r_x * ( cos( cur_theta ) - cos( theta ) ) + r_y * ( sin( cur_theta ) - sin( theta ) ) ) ;//注意，y方向上，前后偏移差值再取负，因为投影方向是向上为正
     }
     VecPosition scan_pre=mea_rfs[i];
     VecPosition new_origin(delta_x,delta_y);
     VecPosition scan_new = scan_pre.globalToRelative(new_origin,Rad2Deg( cur_theta) );
     temp_rfs.push_back(scan_new);
   }
   mea_rfs = temp_rfs;
 }

 void pfLocalize::calGlobalPosThread(){
  //ros::Rate r(100);
  if(!_run_loc_thread){
    ROS_DEBUG("wait for running.do not start cal GlobalPos Thread");
    return;
  }
  static CountTime tim;
  tim.begin();
  double angle_result;
  VecPosition cord_result;
  std::vector<std::pair<int,int> > best ;
  std::vector<std::pair<int,int> > matched_mea_rfs;
  std::vector<VecPosition> cur_map_rfs;
  {
    boost::mutex::scoped_lock l(temp_mut);
    cur_map_rfs = map_rfs;
  }
  static bool first_time = true;

  if(first_time){
    ///base2scan一般不会变化　为了加快速度　这里仅仅做一次转换
    // normally exiting the tf bet base  and scan
    tf::TransformListener tf_listener;
    ///这里有个缺陷，tf一般比较慢，所以对于反光板实时计算有影响
    try{
      tf_listener.waitForTransform(base_frame, scan_frame, ros::Time(0), ros::Duration(1));
      tf_listener.lookupTransform(base_frame, scan_frame, ros::Time(0), tf_base2scan);
      first_time = false;//only transform suc ,can we set true!
    }
    catch (tf::TransformException ex){
      ROS_ERROR("!!!pfLocalize::getMatchedMeaRfs.waitForTransform bet %s and %s .err:%s",base_frame.c_str(),scan_frame.c_str(),ex.what());
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("suc tf from base2scan!");
  }
  //while(nh.ok()){
    ///update
    {
      //disable lock temp
      boost::mutex::scoped_lock l(cal_mut);
     // best.clear();
      //matched_mea_rfs.clear();
      cur_recking_pos = recking_pos;//map2base update cur recking pos for calculating
      mea_rfs = _rfs;//update cur measured rfs for calculating
      compensate_timer.end();//the compensate_timer will be restarted by callBackScan fun when the new rfs be caled
      double comp_dt = compensate_timer.getTime()/1000.0;
      ROS_INFO("cal global pos.the rfs used delay time:%.6f(ms)",comp_dt*1000);
      compensateRfsUsedDelay(mea_rfs,comp_dt);
    }
    /// transform map2base to map2scan <改进２.0 follow 考虑了base2scan的坐标关系>
    //in tf tree 不一定有map2base的tf,所以我们创建了一个
    tf::Transform tmp_tf(tf::createQuaternionFromYaw(cur_recking_pos.theta),tf::Vector3(cur_recking_pos.x,cur_recking_pos.y,0) );///map2base
    tf::StampedTransform tf_map2base (tmp_tf,ros::Time(0),///chq!!!这个时间参数很重要，设置不当会导致transformPose转换失败
                                     map_frame,base_frame);///map2base
    tf::StampedTransform tf_map2scan;
    tf_map2scan.mult(tf_map2base, tf_base2scan);
    cur_scan_pos.x = tf_map2scan.getOrigin().getX();
    cur_scan_pos.y = tf_map2scan.getOrigin().getY();
    cur_scan_pos.theta = tf::getYaw(tf_map2scan.getRotation() );
    ROS_INFO("pfLocalize::calGlobalPosThread.%s2%s tf(x,y,angle):%.6f,%.6f,%.6f",
             base_frame.c_str(),scan_frame.c_str(),tf_base2scan.getOrigin().x(),tf_base2scan.getOrigin().y(),tf::getYaw(tf_base2scan.getRotation()));
    ROS_INFO("pfLocalize::calGlobalPosThread.%s2%s tf(x,y,angle):%.6f,%.6f,%.6f",
             map_frame.c_str(),base_frame.c_str(),tf_map2base.getOrigin().x(),tf_map2base.getOrigin().y(),tf::getYaw(tf_map2base.getRotation()));
    ROS_INFO("pfLocalize::calGlobalPosThread.%s2%s tf(x,y,angle):%.6f,%.6f,%.6f",
             map_frame.c_str(),scan_frame.c_str(),tf_map2scan.getOrigin().x(),tf_map2scan.getOrigin().y(),tf::getYaw(tf_map2scan.getRotation()));

    ///get the matched rfs correspond to map rfs
    getMatchedMeaRfs(cur_scan_pos,mea_rfs,matched_mea_rfs);
    if(mea_rfs.empty()){
      ROS_ERROR("ERROR!!!pfLocalize::calGlobalPosThread.no measured rfs.Using recking pos!.");
    }
    geometry_msgs::PoseStamped _pose;
    if( matched_mea_rfs.size() < 3 ){
      ROS_ERROR("WARNING!!!pfLocalize::calGlobalPosThread.matched_mea_rfs size:%d,no enough matched rfs.Using recking pos!.",matched_mea_rfs.size());
      _pose.header.frame_id=map_frame;
      _pose.header.stamp = ros::Time::now();
      _pose.pose.position.x = recking_pos.x;
      _pose.pose.position.y = recking_pos.y;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(recking_pos.theta),
                            _pose.pose.orientation);
      global_pos_pub.publish(_pose);
    }
    else{
      int res = getOptimizeTriangle(cur_scan_pos,matched_mea_rfs,best);
      if( res > 0){
        //template rfs cord in map
         VecPosition v_a = cur_map_rfs[best[0].first];
         VecPosition v_b = cur_map_rfs[best[1].first];
         VecPosition v_c = cur_map_rfs[best[2].first];
         //the dist bet template rfs and measured rfs
         double ra = mea_rfs[best[0].second].getMagnitude();
         double rb = mea_rfs[best[1].second].getMagnitude();
         double rc = mea_rfs[best[2].second].getMagnitude();
         //已知道反光板绝对位置，和当前车子到三个反光板的距离，求车子位置（三个圆的最优估计交点）
         CalThreeArc cal(v_a,v_b,v_c,ra,rb,rc);
         int suc = cal.getCrossPoint(cord_result);
         if(suc > 0)
         {
           //已知道反光板绝对坐标，定位位置，求定位角度
           //（根据每个实测反光板相对位置数据，可以得到相对夹角
           // 又知道反光板绝对位置与定位位置关系，可以测得反光板在绝对坐标系下的角度，
           // 从而反推出车子角度（反光板绝对角度－相对夹角　＝　车子绝对角度　）
           angle_result = getOptimizeAngle(best,cord_result);
           geometry_msgs::Pose2D pos;
           ///cord_result是scan in map 故需要转换成base in map
           ///map2base = map2scan * inv(base2scan)
           tf::Transform tmp_tf(tf::createQuaternionFromYaw(angle_result),tf::Vector3(cord_result.getX(),cord_result.getY(),0) );///map2base
           tf::StampedTransform tf_map2scan (tmp_tf,ros::Time(0),///chq!!!这个时间参数很重要，设置不当会导致transformPose转换失败
                                            map_frame,scan_frame);///map2base
           tf::StampedTransform tf_map2base;
           tf_map2base.mult(tf_map2scan, tf_base2scan.inverse());
           pos.x = tf_map2base.getOrigin().getX();
           pos.y = tf_map2base.getOrigin().getY();
           pos.theta = tf::getYaw(tf_map2base.getRotation() );

           global_pos = pos;
           _re_deadrecking = true;
           // global_pos_pub.publish(pos);
           _pose.header.stamp = ros::Time::now();
           _pose.pose.position.x = global_pos.x;
           _pose.pose.position.y = global_pos.y;
           tf::quaternionTFToMsg(tf::createQuaternionFromYaw(global_pos.theta),
                                 _pose.pose.orientation);
           global_pos_pub.publish(_pose);

         }
      }
      else
      {
        ROS_INFO("calGlobalPosThread.no good rfs which score beyond the thread.do not cal.");
        _pose.header.frame_id=map_frame;
        _pose.header.stamp = ros::Time::now();
        _pose.pose.position.x = recking_pos.x;
        _pose.pose.position.y = recking_pos.y;
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(recking_pos.theta),
                              _pose.pose.orientation);
        global_pos_pub.publish(_pose);
        //global_pos_pub.publish(recking_pos);

      }
    }
    tim.end();
    double dt = tim.getTime();
    ROS_INFO("calGolbalPosThread.waster time(ms):%.6f",dt);
   // r.sleep();
    ros::spinOnce();
  //}
}


}


