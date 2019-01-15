#include "pf_slam.h"
namespace pf_slam_space {
using namespace pf_localization_space;
pf_slam::pf_slam(){
  ros::NodeHandle private_nh("~");
  private_nh.param<double>("scan_range_max",scan_range_max,6);//rad
  nh.param<double>("triangle_side_min_len",triangle_side_min_len,2);
  nh.param<double>("triangle_grouping_thread",triangle_grouping_thread,20);
  nh.param<double>("new_rfs_dist",new_rfs_dist,1);
  nh.param<double>("new_rfs_angle_deg",new_rfs_angle_deg,10);
  nh.param<int>("history_update_size",history_update_size_thread,10);
  nh.param<double>("history_rfs_reject_dist",history_rfs_reject_dist,0.2);
  nh.param<int>("min_his_rfs_avg_size",min_his_rfs_avg_size,7);
  nh.param<double>("min_his_rfs_update_dist",his_rfs_update_thread,0.001);
  mapping_start = false;
  init_mapping = true;
  ///timer_updatehistory= nh.createTimer(ros::Duration(0.10), boost::bind(&pf_slam::updateHistoryRfsThread,this));
  sub_mapping_task_start = nh.subscribe("pf_mapping_start",1,&pf_slam::callBackMappingStart,this);
  sub_mapping_task_cancel = nh.subscribe("pf_mapping_cancel",1,&pf_slam::callBackMappingCancel,this);
  sub_mapping_task_stop = nh.subscribe("pf_mapping_stop",1,&pf_slam::callBackMappingStop,this);
  map_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("pf_map",1);
  global_pos_pub = nh.advertise< geometry_msgs::PoseStamped >("pf_slam_pose",1);
  ROS_INFO_STREAM("pf_slam.construct.init done!");
}

pf_slam::~pf_slam(){
  for(int i =0 ;i < history_rfs.size();i++){
    delete history_rfs[i]->pos_array;
  }
}
void pf_slam::callBackMappingStart(const std_msgs::Int16 if_start){
  if(if_start.data){
    mapping_start = true;
  }
  else{
    ROS_WARN_STREAM("pf_slam::callBackMappingStart.the callback value is: " <<if_start.data <<"do not start mapping!" );
  }
}

void pf_slam::callBackMappingCancel(const std_msgs::Int16 if_cancel){
  //if(!if_cancel)return;
  mapping_start = false;
  init_mapping = true;
  _abs_map_rfs.clear();
  _triangle_template.clear();
  _pair_rfs_dist.clear();
  ordered_pair_rfs_dist.clear();
  for(int i =0 ;i < history_rfs.size();i++){
    delete history_rfs[i]->pos_array;
  }
  history_rfs.clear();
  stop();

}

void pf_slam::callBackMappingStop(const std_msgs::String fname){
  //save map
  fstream out;
  std::string str_name = fname.data;
  out.open(str_name.c_str(), ios::out|ios::trunc);
  mapping_start = false;
  ///1 out history_rfs
  out<<"[landmarks]\n";
  out <<"index\tx(mm)\ty(mm)\tvariance(mm^2)\thistory_use_num\n";
  for(int i =0 ;i < history_rfs.size();i++){
    rfs_history_cord* &cur_his_rfs_cord = history_rfs[i];
    VecPosition cur_p;
    cur_his_rfs_cord->getAgvPos(cur_p);
    out <<i<<"\t"<< (int)(cur_p.getX()*1000) << "\t" << (int)(cur_p.getY()*1000)  <<"\t"<< (int)(cur_his_rfs_cord->dx()*1e6)<<"\t" <<cur_his_rfs_cord->size()<<"\n";
  }
  ///2 out _triangle_template
  std::map<std::set<int>,comparedata> temp_triangle_rfs = _triangle_template;
  std::map<std::set<int>,comparedata>::iterator itr = temp_triangle_rfs.begin();
  out<<"[triangle_template_data]\n";
  out <<"index1\tindex2\tindex3\tlength(mm)\tvariance(mm^2)\tpi\n";
  for(;itr != temp_triangle_rfs.end(); itr++){
    std::set<int> temp_set = (*itr).first;
    std::set<int>::iterator itr_set_index = temp_set.begin();//in the set contained the avg_rfs_index then we do next

    if(itr_set_index != temp_set.end() && temp_set.size() == 3){
      for( std::set<int>::iterator itr_set = temp_set.begin();itr_set != temp_set.end(); itr_set++){
        out<<*itr_set<<"\t";
      }
      comparedata pre_cp = itr->second;
      out<<(int)(pre_cp.getLen()*1e3)<<"\t"<<(int)(pre_cp.getDx()*1e6)<<"\t"<<pre_cp.getP()<<"\n";
    }
    else{
      continue;
    }
  }

  ///3 out ordered_pair_rfs_dist
  std::deque<order_pair_rfs_para> cur_order_pair_deque=ordered_pair_rfs_dist;//all the pair rfs ordered by largest to smallest
  order_pair_rfs_para cur_rfs_pair;
  std::set<int> temp_set;
  out<<"[ordered_pair_lmks_dist_data]\n";
  out <<"index1\tindex2\tdist(mm)\n";
  //std::deque<order_pair_rfs_para>::iterator itr_pair = cur_order_pair_deque.begin();
   for(int i = 0 ;i < cur_order_pair_deque.size();i++){
    cur_rfs_pair = cur_order_pair_deque[i];
    temp_set = cur_rfs_pair.pair();
    std::set<int>::iterator itr_index = temp_set.begin();
    if(itr_index!=temp_set.end() && temp_set.size() == 2){
      for( std::set<int>::iterator itr_set = temp_set.begin();itr_set != temp_set.end(); itr_set++){
        out << *itr_set<<"\t";
      }
      out <<(int)(cur_rfs_pair.dist()*1e3)<<"\n";
    }

   }

   out.close();
   ROS_INFO("pf_slam.save map done!");
}

void pf_slam::addingNewRfs(geometry_msgs::Pose2D loc_pos, std::vector<VecPosition> new_rfs){
 // boost::mutex::scoped_lock l(addrfs_mut);
  VecPosition add;
  VecPosition v_pos(loc_pos.x,loc_pos.y);

  ///first add pos the map rfs
  for(int i = 0; i < new_rfs.size();i++){
    add = new_rfs[i];
    add.rotate(Rad2Deg(loc_pos.theta));
    add += v_pos;
    _abs_map_rfs.push_back(add);
    rfs_history_cord* his_data = new rfs_history_cord;
    his_data->setAvgPos(add);
    his_data->addPos(add);
    history_rfs.push_back(his_data);
    ROS_INFO("pf_slam::addingNewRfs.cur loc_pos(%.6f,%.6f).add pos:(%.6f,%.6f)",loc_pos.x,loc_pos.y,add.getX(), add.getY());
  }

  ///second to  updateTriangleTemplate
  std::deque<VecPosition> cur_abs_rfs = _abs_map_rfs;//
  int first_new_index = cur_abs_rfs.size() - new_rfs.size();
  int size = cur_abs_rfs.size();
  double i_j_dist,i_k_dist,j_k_dist;
  double len,mean,dx,max_side,min_side,p;
  Line a;
  for(int i = first_new_index ; i < size;i++){//i:first new push back rfs index,search loop from cur_index to back
    VecPosition v_i = cur_abs_rfs[i];
    for(int j = 0; j < i-1;j++ ){//from [0 - i-1)
      i_j_dist = v_i.getDistanceTo(cur_abs_rfs[j]);
      if( i_j_dist > triangle_grouping_thread || i_j_dist < triangle_side_min_len  ){
        if(i_j_dist < triangle_side_min_len)
          ROS_WARN_STREAM("WARNIGN! pf_slam::updateTriangleTemplate.dist bet rfs index " << i << " and " << j << " do not fit in thread.continue");
        continue;
      }
      for(int k = j+1; k < i; k++ ){//from [i+1 -> i)
        i_k_dist = cur_abs_rfs[k].getDistanceTo(v_i);
        if( i_k_dist < triangle_grouping_thread &&i_k_dist >= triangle_side_min_len ){
          a = Line::makeLineFromTwoPoints(v_i,cur_abs_rfs[j]);
          double dist = a.getDistanceWithPoint(cur_abs_rfs[k]);
          if( dist > EPSILON ){//如果不共线，则可以构成三角形
            int arr[3]={i,j,k};
            set<int> set_index(arr,arr+3);
            j_k_dist = cur_abs_rfs[j].getDistanceTo(cur_abs_rfs[k]);
            if(j_k_dist < triangle_side_min_len ){
              ROS_WARN_STREAM("WARIGN! pfLocalize::createTriangleTemplate.dist bet rfs index " << j << " and " << k << " do not fit in thread.continue");
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
          ROS_WARN_STREAM("WARIGN! pfLocalize::createTriangleTemplate.dist bet rfs index " << i << " and " << k << " do not fit in thread.continue");
      }
      }
    }
  }
  ///thirdly to update data to base localization
  setMapRfsData(_abs_map_rfs);
  setTriangleTemplateData(_triangle_template);

}


void pf_slam::updateRfsPara(int avg_rfs_index,VecPosition avg_rfs){
 double i_j_dist,i_k_dist,j_k_dist;
 double len,mean,dx,max_side,min_side,p;
 Line a;

 std::map<std::set<int>,comparedata> temp_triangle_rfs = _triangle_template;
 std::map<std::set<int>,comparedata>::iterator itr = temp_triangle_rfs.begin();
 _abs_map_rfs[avg_rfs_index] = avg_rfs;
 ///first to update the triangle para
 for(;itr != temp_triangle_rfs.end(); itr++){
   std::set<int> temp_set = (*itr).first;
   std::set<int>::iterator itr_have_index = temp_set.find(avg_rfs_index);//in the set contained the avg_rfs_index then we do next
   int index[3];
   int cnt = 0;
   VecPosition v_pos[3];
   if(itr_have_index != temp_set.end() && temp_set.size() == 3){
     for( std::set<int>::iterator itr_set = temp_set.begin();itr_set != temp_set.end(); itr_set++){
       index[cnt] = *itr_set;
       v_pos[cnt] = _abs_map_rfs[*itr_set];
       cnt++;
     }
   }
   else{
     continue;
   }
   i_j_dist = v_pos[0].getDistanceTo(v_pos[1]);
   i_k_dist = v_pos[0].getDistanceTo(v_pos[2]);
   j_k_dist = v_pos[1].getDistanceTo(v_pos[2]);
   //disable duo to that is a little change and saving time
   /*
   //if the new dist is not in limit ,then erase it from _triangle_template
   i_j_dist = v_pos[0].getDistanceTo(v_pos[1]);
   if(i_j_dist > triangle_grouping_thread || i_j_dist < triangle_side_min_len){
     _triangle_template.erase(temp_set);
     return;
   }
   i_k_dist = v_pos[0].getDistanceTo(v_pos[2]);
   if(i_k_dist > triangle_grouping_thread || i_k_dist < triangle_side_min_len){
     _triangle_template.erase(temp_set);
     return;
   }
   j_k_dist = v_pos[1].getDistanceTo(v_pos[2]);
   if(j_k_dist > triangle_grouping_thread || j_k_dist < triangle_side_min_len){
     _triangle_template.erase(temp_set);
     return;
   }
   -------*/
   a = Line::makeLineFromTwoPoints( v_pos[0],v_pos[1] );
   double dist = a.getDistanceWithPoint( v_pos[2] );
   if( dist > EPSILON ){//如果不共线，则可以构成三角形
     len = i_j_dist+i_k_dist+j_k_dist;
     mean = len/3.0;
     dx = (pow(i_j_dist-mean,2)+pow(i_k_dist-mean,2)+pow(j_k_dist-mean,2))/3;
     max_side = max(max(i_j_dist,i_k_dist),j_k_dist);
     min_side = min(min(i_j_dist,i_k_dist),j_k_dist);
     p = max_side/min_side;
     comparedata cp(len,dx,p);
     comparedata pre_cp = _triangle_template[temp_set];
     ROS_INFO("pf_slam::updateRfsPara.update index:%d pre compdata(%.4f,%.4f,%.4f) new(%.4f,%.4f,%.4f)",avg_rfs_index,pre_cp.getLen(),pre_cp.getDx(),pre_cp.getP(),cp.getLen(),cp.getDx(),cp.getP());
     _triangle_template[temp_set] = cp;//update para
   }
   else{
     _triangle_template.erase(temp_set);//erase by key
   }

 }

 ///second to update the pair rfs data

 std::deque<order_pair_rfs_para> cur_order_pair_deque=ordered_pair_rfs_dist;//all the pair rfs ordered by largest to smallest
 order_pair_rfs_para cur_rfs_pair;
 std::set<int> temp_set;
 //std::deque<order_pair_rfs_para>::iterator itr_pair = cur_order_pair_deque.begin();
  for(int i = 0 ;i < cur_order_pair_deque.size();i++){
   cur_rfs_pair = cur_order_pair_deque[i];
   temp_set = cur_rfs_pair.pair();
   std::set<int>::iterator itr_have_index = temp_set.find(avg_rfs_index);
   int index[2];
   int cnt = 0;
   VecPosition v_pos[2];
   if(itr_have_index!=temp_set.end() && temp_set.size() == 2){
     for( std::set<int>::iterator itr_set = temp_set.begin();itr_set != temp_set.end(); itr_set++){
       index[cnt] = *itr_set;
       v_pos[cnt] = _abs_map_rfs[*itr_set];
       cnt++;
     }
     i_j_dist = v_pos[0].getDistanceTo(v_pos[1]);
     order_pair_rfs_para cur_rfs_pair_temp(temp_set,i_j_dist);
     order_pair_rfs_para pre_pair = ordered_pair_rfs_dist[i];
     ROS_INFO("pf_slam::updateRfsPara.update index:%d pre pair dist(%.4f) new(%.4f)",avg_rfs_index,pre_pair.dist(),cur_rfs_pair_temp.dist());

     ordered_pair_rfs_dist[i] = cur_rfs_pair_temp;
   }else{
     continue;
   }

 }

  setMapRfsData(_abs_map_rfs);
  setTriangleTemplateData(_triangle_template);
  //third update ordered
  sortPairRfsDist();

}
void pf_slam::sortPairRfsDist(){
    std::deque<order_pair_rfs_para> cur_order_pair_deque=ordered_pair_rfs_dist;//all the pair rfs ordered by largest to smallest
    order_pair_rfs_para cur_rfs_pair;

    ///插入排序1
    for(int i = 1 ;i < cur_order_pair_deque.size();i++){
      for(int j = i; j > 0 && cur_order_pair_deque[j-1].dist() < cur_order_pair_deque[j].dist();j--){
        cur_rfs_pair = cur_order_pair_deque[j];
        cur_order_pair_deque[j] = cur_order_pair_deque[j-1];
        cur_order_pair_deque[j-1] = cur_rfs_pair;
        //ROS_WARN_STREAM("pf_mapping.sortPairRfsDist:cur_order_pair_deque(j)["<<j<<"] dist:" <<cur_order_pair_deque[j].dist() );
        //ROS_WARN_STREAM("pf_mapping.sortPairRfsDist:cur_order_pair_deque(j-1)["<<j-1<<"] dist:" <<cur_order_pair_deque[j-1].dist() );
      }

    }
    #if 0
    ///插入排序2 more fast
      for(int i = 1 ;i < cur_order_pair_deque.size();i++){
        int j = i;
        cur_rfs_pair = cur_order_pair_deque[j];
        for(; j > 0 && cur_order_pair_deque[j-1].dist() < cur_rfs_pair.dist();j--){
          cur_order_pair_deque[j] = cur_order_pair_deque[j-1];
         }
        cur_order_pair_deque[j] = cur_rfs_pair;
      }
   #endif
    ordered_pair_rfs_dist = cur_order_pair_deque;
}

void pf_slam::insertSortRfs(std::vector<VecPosition> new_rfs){
  std::deque<VecPosition> cur_abs_rfs = _abs_map_rfs;//
  double i_j_dist;
  int first_new_index = cur_abs_rfs.size()-new_rfs.size();
  //fistly to insert
  for(int i = first_new_index;i < cur_abs_rfs.size();i++){
    VecPosition add = cur_abs_rfs[i];
    for(int j = 0 ; j < i;j++){//只yu比其索引低的做比较，这样下一个新的自然会 comp to cur 索引
      i_j_dist = add.getDistanceTo(cur_abs_rfs[j]);
      if( i_j_dist <= triangle_grouping_thread && i_j_dist >= triangle_side_min_len  ){
        int arr[2]={i,j};
        std::set<int> set_index(arr,arr+2);//The range used is [first,last)
        order_pair_rfs_para cur_rfs_pair(set_index,i_j_dist);
        ordered_pair_rfs_dist.push_back(cur_rfs_pair);
        //std::cout << "pf_slam::insertSortRfs.ordered_pair_rfs_dist: i: " <<i <<" j"<< j << cur_rfs_pair;
      }
    }
  }
  //secondly to sort
  sortPairRfsDist();

}

void pf_slam::getNewRfs(geometry_msgs::Pose2D loc_pos, std::vector<VecPosition>& mea_rfs){
 std::vector<VecPosition> cur_mea_rfs = mea_rfs;
 std::deque<VecPosition> cur_abs_rfs = _abs_map_rfs;//
 std::vector<int> cur_candidate_match_rfs;//
 double dist;
 bool dis_match,ang_match;
 mea_rfs.clear();
 VecPosition v_loc_pos(loc_pos.x,loc_pos.y);
 //filter the map rfs which in cur scan view range
 for(int i = 0;i < cur_abs_rfs.size();i++){
   // if(dist = v_pos.getDistanceTo(cur_abs_rfs[i]) < scan_range_max){
      cur_candidate_match_rfs.push_back(i);
    //}
 }
// cur_candidate_match_rfs = cur_abs_rfs;
 int mea_size = cur_mea_rfs.size();
 int cand_size = cur_candidate_match_rfs.size();
 if( !cand_size ){
   ROS_ERROR("pf_slam::getNewRfs. cur_candidate_match_rfs is empty.return!");
   return;
 }
 VecPosition rel_rfs;
 double delta;
 double cmp_dist[mea_size][cand_size],cmp_angle[mea_size][cand_size];

 for(int i = 0; i < mea_size; i++){
   bool matched = false;
   for(int j = 0; j < cand_size; j++){
     rel_rfs = cur_abs_rfs[cur_candidate_match_rfs[j]];
     rel_rfs -= v_loc_pos;
     rel_rfs.rotate(-Rad2Deg(loc_pos.theta));
     cmp_dist[i][j] = rel_rfs.getDistanceTo(cur_mea_rfs[i]);
     delta = Deg2Rad( rel_rfs.getDirection()-cur_mea_rfs[i].getDirection() );
     cmp_angle[i][j] = fabs( atan2Deg( sin(delta),cos(delta) ) );
     dis_match = cmp_dist[i][j] <= new_rfs_dist;
     ang_match = cmp_angle[i][j] <= new_rfs_angle_deg;
     if( dis_match && ang_match ){
        matched = true;
        break;
      }
    }
    if( !matched ){//no cur measured rfs in map rfs
      VecPosition abs_mea_pos = cur_mea_rfs[i];
      abs_mea_pos.rotate(Rad2Deg(loc_pos.theta));
      abs_mea_pos += v_loc_pos;
      ROS_INFO("pf_slam.getNewRfs.no matched map rfs to cur mea rfs.abs_mea_pos:(%.4f,%.4f).loc_pos:(%.4f,%.4f)",abs_mea_pos.getX(),abs_mea_pos.getY(), loc_pos.x,loc_pos.y );
      for(int k = 0; k < cand_size; k++ ){
        rel_rfs = cur_abs_rfs[cur_candidate_match_rfs[k]];
        ROS_INFO("comp abs_map_pos(%.4f,%.4f).dist:%.4f,angle(deg):%.4f",rel_rfs.getX(),rel_rfs.getY(),cmp_dist[i][k],cmp_angle[i][k]);
      }
      mea_rfs.push_back(cur_mea_rfs[i]);
      //ROS_INFO("pf_slam::getNewMatchedRfs.measued rfs index:%d,rel pos:(%.3f,%.3f) is new rfs!",i,cur_mea_rfs[i].getX(),cur_mea_rfs[i].getY());
    }

 }

}
void pf_slam::pubMarkerRfs(){

visualization_msgs::Marker rfs_marker;
visualization_msgs::Marker text_marker;
visualization_msgs::Marker loc_pos_marker;
visualization_msgs::MarkerArray marker_array;
rfs_marker.header.stamp = ros::Time::now();
loc_pos_marker.header.stamp = rfs_marker.header.stamp;
rfs_marker.header.frame_id = "map";
loc_pos_marker.header.frame_id = rfs_marker.header.frame_id;
rfs_marker.type = visualization_msgs::Marker::POINTS;
loc_pos_marker.type = visualization_msgs::Marker::ARROW;
rfs_marker.ns = "map_nmspace";
rfs_marker.action = visualization_msgs::Marker::ADD;
// Set the scale of the marker -- 1x1x1 here means 1m on a side
rfs_marker.scale.x = 0.05;
rfs_marker.scale.y = 0.05;
rfs_marker.scale.z = 0.05;
// Set the color -- be sure to set alpha to something non-zero!
//DarkOrchid	153 50 204
rfs_marker.color.r = 153;
rfs_marker.color.g = 50;
rfs_marker.color.b = 204;
rfs_marker.color.a = 0.3;
rfs_marker.lifetime = ros::Duration(1000);
geometry_msgs::Point p;
geometry_msgs::Pose pose;
if(_abs_map_rfs.size() ){
  for(int i=0;i < _abs_map_rfs.size();i++){
    rfs_marker.id = i;
    //在相对于激光头的坐标空间
    p.x = _abs_map_rfs[i].getX();
    p.y = _abs_map_rfs[i].getY();
    p.z = 0;

    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = 0;
    ////fort text type,Only scale.z is used. scale.z specifies the height of an uppercase "A".
    text_marker = rfs_marker;
    text_marker.ns = "rfs_text_space";
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
    rfs_marker.points.push_back(p);
  }

  geometry_msgs::Pose2D loc_pos = getReckingPos();
  pose.position.x = loc_pos.x;
  pose.position.y = loc_pos.y;
  pose.position.z = 0;
  //tf::Quaternion qua = tf::createQuaternionFromRPY(0,0,loc_pos.theta);
  //geometry_msgs::Quaternion geo_qua;
  //tf::quaternionTFToMsg(qua,geo_qua);
  //pose.orientation = geo_qua;
  pose.orientation =tf::createQuaternionMsgFromYaw(loc_pos.theta);


  loc_pos_marker.color.r = 153;
  loc_pos_marker.color.g = 0;
  loc_pos_marker.color.b = 0;
  loc_pos_marker.color.a = 0.3;
  loc_pos_marker.scale.x = 0.1;//scale.x 是箭头长度，scale.y是箭头宽度和scale.z是箭头高度。
  loc_pos_marker.scale.y = 0.05;
  loc_pos_marker.scale.z = 0.00;
  loc_pos_marker.pose = pose;
  marker_array.markers.push_back(loc_pos_marker);
  marker_array.markers.push_back(rfs_marker);
  map_marker_pub.publish(marker_array);
}


}
void pf_slam::pubGlobalPos( geometry_msgs::Pose2D pos ){
  geometry_msgs::PoseStamped _pose;
  _pose.header.frame_id=getMapFrame();
  _pose.header.stamp = ros::Time::now();
  _pose.pose.position.x = pos.x;
  _pose.pose.position.y = pos.y;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(pos.theta),
                        _pose.pose.orientation);
  global_pos_pub.publish(_pose);
}

void pf_slam::calGlobalPosThread(){

  if(!mapping_start){
    ROS_DEBUG("wait for running.pf_slam do not start cal GlobalPos Thread");
    return;
  }

  double angle_result;
  VecPosition cord_result;
  std::vector<std::pair<int,int> > best,matched_mea_rfs;
  std::vector<VecPosition> cur_map_rfs,cur_mea_rfs,cur_mea_rfs_mapping;
  geometry_msgs::Pose2D loc_pos,cur_recking_pos;
  double comp_dt;
  ///get newest data
  {
    boost::mutex::scoped_lock l(temp_mut);
    cur_mea_rfs = getMeasuredRfs();
    if( cur_mea_rfs.size() < 3 ){
      ROS_WARN("pf_slam::slamThread.init thread.no enough rfs data .rfs num:%d,do nothing return;",cur_mea_rfs.size());
      ROS_WARN("pf_slam::slamThread.please move the car to the position where excitting enough rfs;");
      return;
    }
    cur_recking_pos = getReckingPos();
    loc_pos = cur_recking_pos;//update cur recking pos for calculating
    if(loc_pos.x != loc_pos.x){//check nan
      ROS_ERROR("pf_slam::slamThread.loc data invalid.return!");
      return;
    }
  }
  ///compensate data time delay
  {
    boost::mutex::scoped_lock l(cal_mut);
    comp_dt = compensateTimeStop();//the compensate_timer will be restarted by callBackScan fun when the new rfs be caled
    comp_dt /= 1000.0;
    ROS_INFO("pf_slam::slamThread.the rfs used delay time:%.3f(ms)",comp_dt*1000);
    compensateRfsUsedDelay(cur_mea_rfs,comp_dt);
    UpdateCurMeaRfs(cur_mea_rfs);
    cur_mea_rfs_mapping = cur_mea_rfs;
  }
  ///if have not
  if( init_mapping ){
    {
    boost::mutex::scoped_lock l(addrfs_mut);
    addingNewRfs(loc_pos,cur_mea_rfs_mapping);
    insertSortRfs(cur_mea_rfs_mapping);
    //setMapRfsData(_abs_map_rfs);
    //setTriangleTemplateData(_triangle_template);
    }
    run();//run base class thread such as dead recking
    init_mapping = false;
    return;
  }
  ///get newest map data (after mapping inited !!!)
  {
    boost::mutex::scoped_lock l(cal_mut);
    cur_map_rfs = getCurMapRfs();
    pubMarkerRfs();
  }
  ///due to we start a timer so we can not easily return out this func until we stop this timer
  static CountTime tim;
  tim.begin();
  ///get the matched rfs correspond to map rfs
  getMatchedMeaRfs(loc_pos,cur_mea_rfs,matched_mea_rfs);//pair type: var first map rfs index,second measured rfx index
  if( matched_mea_rfs.size() < 3 ){
    ROS_ERROR("ERROR!!!pf_slam.slam_thread.matched_mea_rfs size:%d,no enough matched rfs.Using recking pos!.",matched_mea_rfs.size());
    pubGlobalPos(loc_pos);
  }
  else{
    int res = getOptimizeTriangle(matched_mea_rfs,best);
    if( res > 0){
      //template rfs cord in map
       VecPosition v_a = cur_map_rfs[best[0].first];
       VecPosition v_b = cur_map_rfs[best[1].first];
       VecPosition v_c = cur_map_rfs[best[2].first];
       //the dist bet template rfs and measured rfs
       double ra = cur_mea_rfs[best[0].second].getMagnitude();
       double rb = cur_mea_rfs[best[1].second].getMagnitude();
       double rc = cur_mea_rfs[best[2].second].getMagnitude();
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
         //在建图模式下，定位误差较大，在全局定位发生较大跳跃时，不使用全局定位结果
         double de_x = fabs(cord_result.getX() - loc_pos.x);
         double de_y = fabs(cord_result.getY() - loc_pos.y);
         if(de_x> 0.04 || de_y > 0.04){
           ROS_INFO("pf_slam.calGlobalPos.big delta happened!");
         }
         else{
           loc_pos.x = cord_result.getX();
           loc_pos.y = cord_result.getY();
           loc_pos.theta = angle_result;
         }
         setGlobalPos(loc_pos);
         setReReckingFlag(true);
         pubGlobalPos(loc_pos);
         ROS_INFO("pf_slam cal global pos suc!global pos(%.6f,%.6f,%.6f),pre recking pos(%.6f,%.6f,%.6f)",
                  loc_pos.x,loc_pos.y,loc_pos.theta,cur_recking_pos.x,cur_recking_pos.y,cur_recking_pos.theta);
         ///for slam mapping
         getNewRfs(loc_pos,cur_mea_rfs_mapping);///!!!只有初始和有全局定位结果时，slam才是准确的!!!!
         if(cur_mea_rfs_mapping.size())
         {
           boost::mutex::scoped_lock l(addrfs_mut);
           addingNewRfs(loc_pos,cur_mea_rfs_mapping);
           insertSortRfs(cur_mea_rfs_mapping);
         }
         /// update history
         updateHistoryRfsThread(loc_pos,cur_mea_rfs,matched_mea_rfs);
         ///

       }else{
         pubGlobalPos(loc_pos);
         ROS_ERROR("pf_slam.cal cross point error!pre recking pos(%.6f,%.6f,%.6f)",cur_recking_pos.x,cur_recking_pos.y,cur_recking_pos.theta);
       }
    }
    else
    {
      ROS_INFO("pf_slam.calGlobalPosThread.no good rfs which score beyond the thread.do not cal.");
       pubGlobalPos( loc_pos );
    }
  }

  tim.end();
  double dt = tim.getTime();
  ROS_INFO("pf_slam.calGolbalPosThread.waster time(ms):%.6f",dt);
  ros::spinOnce();
  //}
}
void pf_slam::updateHistoryRfsThread( geometry_msgs::Pose2D loc_pos,std::vector<VecPosition> mea_rfs,std::vector<std::pair<int,int> > matched_mea_rfs){
  //moving average!!!
  if( !mapping_start ){
    ROS_INFO("pf_slam::updateHistoryRfsThread.mapping not start.return");
    return;
  }
 ///std::vector<std::pair<int,int> > matched_mea_rfs;//map rfs index<->measured rfs index
 /// geometry_msgs::Pose2D loc_pos = getLocPos();
 /// std::vector<VecPosition> mea_rfs = getMeasuredRfs();
 /// getMatchedMeaRfs(loc_pos,mea_rfs,matched_mea_rfs);
  int map_rfs_index;
  VecPosition v_pos(loc_pos.x,loc_pos.y);
  for(int i = 0 ; i < matched_mea_rfs.size();i++){
    map_rfs_index = matched_mea_rfs[i].first;
    VecPosition add=mea_rfs[matched_mea_rfs[i].second];
    add.rotate(Rad2Deg(loc_pos.theta));
    add += v_pos;

    rfs_history_cord* &cur_his_rfs_cord = history_rfs[map_rfs_index];
    std::list<VecPosition> &pos_arr=cur_his_rfs_cord->getPosArray();
    int cur_his_size = pos_arr.size();

    if( cur_his_size < history_update_size_thread)
    {//if do not meet upate condition,only to add cur pos to his data
      cur_his_rfs_cord->addPos(add);
    }
    else{//otherwise we average the cord data
      double sum_x = 0.;double sum_y = 0.;
      std::list<VecPosition>::iterator itr = pos_arr.begin();
      std::list<VecPosition> filter_his_rfs;
      for( ; itr !=pos_arr.end(); itr++){
        sum_x += itr->getX();
        sum_y += itr->getY();
      }
      sum_x/=pos_arr.size();
      sum_y/=pos_arr.size();

      itr = pos_arr.begin();
      VecPosition avg(sum_x,sum_y);
      //filter the pos which the dist to avg_pos is too large
      for( ; itr !=pos_arr.end(); itr++){
        if(fabs(avg.getDistanceTo(*itr)) > history_rfs_reject_dist)continue;
        filter_his_rfs.push_back(*itr);
      }
      //judge the less size
      if( filter_his_rfs.size() < min_his_rfs_avg_size )
      {//if too little,just update the data but not update the new avg pos
        pos_arr = filter_his_rfs;
        cur_his_rfs_cord->setUseNum(filter_his_rfs.size());
        //then do nothing
      }
      else{//otherwise we try to update the avg cord pos
        sum_x = 0.;
        sum_y = 0.;
        itr = filter_his_rfs.begin();
        for( ; itr !=filter_his_rfs.end(); itr++){
          sum_x += itr->getX();
          sum_y += itr->getY();
        }
        sum_x/=filter_his_rfs.size();
        sum_y/=filter_his_rfs.size();

        VecPosition new_avg(sum_x,sum_y);
        VecPosition delta ;
        double var = 0;
        itr = filter_his_rfs.begin();
        for( ; itr !=filter_his_rfs.end(); itr++){
          delta = *itr - new_avg;
          var += pow(delta.getX(),2)+pow(delta.getY(),2);
        }
        var/=filter_his_rfs.size();
        cur_his_rfs_cord->setCurDx(var);//record cur newest dx
        double cur_dx = cur_his_rfs_cord->dx();
        double delta_dx = cur_dx - var;
        if( delta_dx > his_rfs_update_thread*his_rfs_update_thread ){
          //if the new var is better enough ,lastly we update the data
          cur_his_rfs_cord->setAvgPos(new_avg);
          cur_his_rfs_cord->setDx(var);
          cur_his_rfs_cord->setUseNum(filter_his_rfs.size());
         {
          boost::mutex::scoped_lock l(addrfs_mut);
          updateRfsPara(map_rfs_index,new_avg);//
          //update the new pos to abs_map_rfs
         }

        }
       cur_his_rfs_cord->pos_array->pop_front();//delete the oldest one

      }
    }
  }
}



}
