#include "pf_mapping.h"
#include <fstream>
#include <tf/transform_datatypes.h>
namespace pf_mapping_space {

  pf_mapping::pf_mapping(){
    ros::NodeHandle private_nh("~");
    private_nh.param<double>("init_pos_x",init_x,0.0);
    private_nh.param<double>("init_pos_y",init_y,0.0);
    private_nh.param<double>("init_pos_angle",init_theta,0.0);//rad
    private_nh.param<double>("scan_range_max",scan_range_max,6);//rad

    nh.param<double>("triangle_side_min_len",triangle_side_min_len,2);
    nh.param<double>("triangle_grouping_thread",triangle_grouping_thread,20);

    nh.param<double>("new_rfs_dist",new_rfs_dist,1);
    nh.param<double>("new_rfs_angle_deg",new_rfs_angle_deg,10);
    nh.param<int>("history_update_size",history_update_size_thread,10);
    nh.param<double>("history_rfs_reject_dist",history_rfs_reject_dist,0.5);
    nh.param<int>("min_his_rfs_avg_size",min_his_rfs_avg_size,7);
    nh.param<double>("min_his_rfs_update_dist",his_rfs_update_thread,0.01);
    mapping_start = false;
    init_mapping = true;
    timer_mapping= nh.createTimer(ros::Duration(0.01), boost::bind(&pf_mapping::mappingThread,this));
    timer_updatehistory= nh.createTimer(ros::Duration(0.10), boost::bind(&pf_mapping::updateHistoryRfsThread,this));
    sub_mapping_task_start = nh.subscribe("pf_mapping_start",1,&pf_mapping::callBackMappingStart,this);
    sub_mapping_task_cancel = nh.subscribe("pf_mapping_cancel",1,&pf_mapping::callBackMappingCancel,this);
    sub_mapping_task_stop = nh.subscribe("pf_mapping_stop",1,&pf_mapping::callBackMappingStop,this);
    map_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("pf_map",1);
  }
  pf_mapping::~pf_mapping(){
    for(int i =0 ;i < history_rfs.size();i++){
      delete history_rfs[i]->pos_array;
    }
  }

  void pf_mapping::callBackMappingStart(const std_msgs::Int16 if_start){
    if(if_start.data){
      mapping_start = true;
    }
    else{
      ROS_WARN_STREAM("pf_mapping::callBackMappingStart.the callback value is: " <<if_start.data <<"do not start mapping!" );
    }
  }

  void pf_mapping::callBackMappingCancel(const std_msgs::Int16 if_cancel){
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
    localization.stop();

  }

  void pf_mapping::callBackMappingStop(const std_msgs::String fname){
    //save map
    fstream out;
    std::string str_name = fname.data;
    out.open(str_name.c_str(), ios::out|ios::trunc);


    mapping_start = false;

}

  void pf_mapping::addingNewRfs(geometry_msgs::Pose2D loc_pos, std::vector<VecPosition> new_rfs){
   // boost::mutex::scoped_lock l(addrfs_mut);
    VecPosition add;
    VecPosition v_pos(loc_pos.x,loc_pos.y);
    for(int i = 0; i < new_rfs.size();i++){
      add = new_rfs[i];
      add.rotate(Rad2Deg(loc_pos.theta));
      add += v_pos;
      _abs_map_rfs.push_back(add);
      rfs_history_cord* his_data = new rfs_history_cord;
      his_data->setAvgPos(add);
      his_data->addPos(add);
      history_rfs.push_back(his_data);
    }
  }

  void pf_mapping::updateTriangleTemplate(std::vector<VecPosition> new_rfs){
    std::deque<VecPosition> cur_abs_rfs = _abs_map_rfs;//
    int first_new_index = cur_abs_rfs.size()-new_rfs.size();
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
            ROS_WARN_STREAM("WARNIGN! pf_mapping::updateTriangleTemplate.dist bet rfs index " << i << " and " << j << " do not fit in thread.continue");
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

  }
  void pf_mapping::updateRfsPara(int avg_rfs_index,VecPosition avg_rfs){
   double i_j_dist,i_k_dist,j_k_dist;
   double len,mean,dx,max_side,min_side,p;
   Line a;

   std::map<std::set<int>,comparedata> temp_triangle_rfs = _triangle_template;
   std::map<std::set<int>,comparedata>::iterator itr = temp_triangle_rfs.begin();
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
         if(itr_have_index == itr_set)//if the set value is equal to avg_rfs_index then do update the avg_rfs to new v_pos
           v_pos[cnt] =  avg_rfs;
         else
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
       ROS_INFO("pf_mapping::updateRfsPara.update index:%d pre compdata(%.4f,%.4f,%.4f) new(%.4f,%.4f,%.4f)",avg_rfs_index,pre_cp.getLen(),pre_cp.getDx(),pre_cp.getP(),cp.getLen(),cp.getDx(),cp.getP());
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
         if(itr_have_index == itr_set)//if the set value is equal to avg_rfs_index then do update the avg_rfs to new v_pos
           v_pos[cnt] =  avg_rfs;
         else
           v_pos[cnt] = _abs_map_rfs[*itr_set];
         cnt++;
       }
       i_j_dist = v_pos[0].getDistanceTo(v_pos[1]);
       order_pair_rfs_para cur_rfs_pair_temp(temp_set,i_j_dist);
       order_pair_rfs_para pre_pair = ordered_pair_rfs_dist[i];
       ROS_INFO("pf_mapping::updateRfsPara.update index:%d pre pair dist(%.4) new(%.4f)",avg_rfs_index,pre_pair.dist(),cur_rfs_pair_temp.dist());

       ordered_pair_rfs_dist[i] = cur_rfs_pair_temp;
     }else{
       continue;
     }
   }
    //third update ordered
    sortPairRfsDist();



 }
  void pf_mapping::sortPairRfsDist(){
      std::deque<order_pair_rfs_para> cur_order_pair_deque=ordered_pair_rfs_dist;//all the pair rfs ordered by largest to smallest
      order_pair_rfs_para cur_rfs_pair;
      for(int i = 1 ;i < cur_order_pair_deque.size();i++){
        for(int j = i; j > 0 && cur_order_pair_deque[j-1].dist() < cur_order_pair_deque[j].dist();j--){
          cur_rfs_pair = cur_order_pair_deque[j];
          cur_order_pair_deque[j] = cur_order_pair_deque[j-1];
          cur_order_pair_deque[j-1] = cur_rfs_pair;
          ROS_WARN_STREAM("pf_mapping.sortPairRfsDist:cur_order_pair_deque(j)["<<j<<"] dist:" <<cur_order_pair_deque[j].dist() );
          ROS_WARN_STREAM("pf_mapping.sortPairRfsDist:cur_order_pair_deque(j-1)["<<j-1<<"] dist:" <<cur_order_pair_deque[j-1].dist() );
        }

      }
      ordered_pair_rfs_dist = cur_order_pair_deque;
  }

  void pf_mapping::insertSortRfs(std::vector<VecPosition> new_rfs){
    std::deque<VecPosition> cur_abs_rfs = _abs_map_rfs;//
    double i_j_dist;
    int first_new_index = cur_abs_rfs.size()-new_rfs.size();
    //fistly to insert
    for(int i = first_new_index;i < cur_abs_rfs.size();i++){
      VecPosition add = cur_abs_rfs[i];
      for(int j = 0 ; j < i;j++){//只yu比其索引低的做比较，这样下一个新的自然会 comp to cur 索引
        i_j_dist = add.getDistanceTo(cur_abs_rfs[j]);
        if( i_j_dist < triangle_grouping_thread && i_j_dist >= triangle_side_min_len  ){
          int arr[2]={i,j};
          std::set<int> set_index(arr,arr+2);//The range used is [first,last)
          order_pair_rfs_para cur_rfs_pair(set_index,i_j_dist);
         //comment temp --s
         // std::cout << "pf_mapping::insertSortRfs.insert pair set:(";
         // for (std::set<int>::iterator it=set_index.begin(); it!=set_index.end(); ++it)
         //   std::cout << *it << ',';
         // std::cout << ")\n dist: " << i_j_dist;

          ordered_pair_rfs_dist.push_back(cur_rfs_pair);

        }
      }
    }

    //secondly to sort
    sortPairRfsDist();

  }

  void pf_mapping::getNewRfs(geometry_msgs::Pose2D loc_pos, std::vector<VecPosition>& mea_rfs){
   std::vector<VecPosition> cur_mea_rfs = mea_rfs;
   mea_rfs.clear();
   std::deque<VecPosition> cur_abs_rfs = _abs_map_rfs;//
   std::vector<int> cur_candidate_match_rfs;//
   double dist;
   VecPosition v_loc_pos(loc_pos.x,loc_pos.y);
   //filter the map rfs which in cur scan view range
   for(int i = 0;i < cur_abs_rfs.size();i++){
     // if(dist = v_pos.getDistanceTo(cur_abs_rfs[i]) < scan_range_max){
        cur_candidate_match_rfs.push_back(i);
      //}
   }
  // cur_candidate_match_rfs = cur_abs_rfs;
   int cand_size = cur_candidate_match_rfs.size();
   if( !cand_size ){
     ROS_ERROR("pf_mapping::getNewRfs. cur_candidate_match_rfs is empty.return!");
     return;
   }
   VecPosition rel_rfs;
   double delta;
   double cmp_dist[cand_size],cmp_angle[cand_size];

   for(int i = 0;i < cur_mea_rfs.size();i++){
     bool matched = false;
     for(int j = 0;j < cand_size;j++){
       rel_rfs = cur_abs_rfs[cur_candidate_match_rfs[j]];
       rel_rfs -= v_loc_pos;
       rel_rfs.rotate(-Rad2Deg(loc_pos.theta));
       cmp_dist[i] = rel_rfs.getDistanceTo(cur_mea_rfs[i]);
       delta = Deg2Rad( rel_rfs.getDirection()-cur_mea_rfs[i].getDirection() );
       cmp_angle[i] = fabs( atan2Deg( sin(delta),cos(delta) ) );
       if(cmp_dist[i] < new_rfs_dist && cmp_angle[i]  < new_rfs_angle_deg){
          matched = true;
          break;
        }
      }
      if(!matched){//no cur measured rfs in map rfs
        VecPosition abs_mea_pos=cur_mea_rfs[i];
        abs_mea_pos.rotate(Rad2Deg(loc_pos.theta));
        abs_mea_pos += v_loc_pos;
        ROS_INFO("pf_mapping.getNewRfs.no matched.comp data to abs_mea_pos:(%.4f,%.4f).loc_pos:(%.4f,%.4f)",abs_mea_pos.getX(),abs_mea_pos.getY(), loc_pos.x,loc_pos.y );
        for(int k = 0;k < cand_size;k++ ){
          rel_rfs = cur_abs_rfs[cur_candidate_match_rfs[k]];
          ROS_INFO("comp abs_map_pos(%.4f,%.4f).dist:%.4f,angle(deg):%.4f",rel_rfs.getX(),rel_rfs.getY(),cmp_dist[k],cmp_angle[k]);
        }
        mea_rfs.push_back(cur_mea_rfs[i]);
        //ROS_INFO("pf_mapping::getNewMatchedRfs.measued rfs index:%d,rel pos:(%.3f,%.3f) is new rfs!",i,cur_mea_rfs[i].getX(),cur_mea_rfs[i].getY());
      }

   }

  }
 void pf_mapping::pubMarkerRfs(){

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

    geometry_msgs::Pose2D loc_pos = localization.getReckingPos();
    pose.position.x = loc_pos.x;
    pose.position.y = loc_pos.y;
    pose.position.z = 0;
    tf::Quaternion qua = tf::createQuaternionFromRPY(0,0,loc_pos.theta);
    geometry_msgs::Quaternion geo_qua;
    tf::quaternionTFToMsg(qua,geo_qua);
    pose.orientation = geo_qua;


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
  void pf_mapping::mappingThread(){
    std::vector<VecPosition> cur_mea_rfs;
    //std::vector<int> v_index;
    geometry_msgs::Pose2D loc_pos;

    if(!mapping_start){
      ROS_INFO("pf_mapping::mappingThread.mapping not start.return");
      return;
    }
    pubMarkerRfs();
    if(init_mapping){
      cur_mea_rfs = localization.getMeasuredRfs();
      if( cur_mea_rfs.size() < 3 ){
        ROS_WARN("pf_mapping::mappingThread.init thread.no enough rfs data .rfs num:%d,do nothing return;",cur_mea_rfs.size());
        ROS_WARN("pf_mapping::mappingThread.please move the car to the position where excitting enough rfs;");
        return;
      }
      localization.setInitPos(init_x,init_y,init_theta);
      loc_pos = localization.getReckingPos();
      {
      boost::mutex::scoped_lock l(addrfs_mut);
      addingNewRfs(loc_pos,cur_mea_rfs);
      updateTriangleTemplate(cur_mea_rfs);
      insertSortRfs(cur_mea_rfs);
      localization.setMapRfsData(_abs_map_rfs);
      localization.setTriangleTemplateData(_triangle_template);
      }
      localization.run();
      init_mapping = false;
      return;
    }
    cur_mea_rfs = localization.getMeasuredRfs();
    loc_pos = localization.getReckingPos();
    if( cur_mea_rfs.size() < 3 ){
      ROS_WARN("pf_mapping::mappingThread.cur no enough rfs data .rfs num:%d,do nothing return;",cur_mea_rfs.size());
      ROS_WARN("pf_mapping::mappingThread.please move the car to the position where excitting enough rfs;");
      return;
    }
    if(loc_pos.x!=loc_pos.x){//forbid nan
      ROS_ERROR("pf_mapping::mappingThread.loc data invalid.return!");
      return;
    }
    getNewRfs(loc_pos,cur_mea_rfs);
    std::deque<order_pair_rfs_para>::iterator itr_d = ordered_pair_rfs_dist.begin();
    ROS_INFO("pf_mapping::insertrfs.display orderedpairrfsdist:");
    /*
     for(;itr_d!=ordered_pair_rfs_dist.end();itr_d++){
       std::set<int> temp_set=(*itr_d).pair();
       std::set<int>::iterator itr_set = temp_set.begin();
      std::cout << "size: " << ordered_pair_rfs_dist.size() << " pair:(";
       for(;itr_set!=temp_set.end();itr_set++){
         std::cout << " " << *itr_set;
       }
     std::cout << ") dist: " <<(*itr_d).dist()<< std::endl ;
    }
   */
    {
    if(cur_mea_rfs.empty())return;
    boost::mutex::scoped_lock l(addrfs_mut);
    addingNewRfs(loc_pos,cur_mea_rfs);
    updateTriangleTemplate(cur_mea_rfs);
    insertSortRfs(cur_mea_rfs);

    localization.setMapRfsData(_abs_map_rfs);
    localization.setTriangleTemplateData(_triangle_template);
    }
  }

  void pf_mapping::updateHistoryRfsThread(){
    //moving average!!!
    if( !mapping_start ){
      ROS_INFO("pf_mapping::updateHistoryRfsThread.mapping not start.return");
      return;
    }
    std::vector<std::pair<int,int> > matched_mea_rfs;//map rfs index<->measured rfs index
    geometry_msgs::Pose2D loc_pos = localization.getReckingPos();
    std::vector<VecPosition> mea_rfs = localization.getMeasuredRfs();
    localization.getMatchedMeaRfs(mea_rfs,matched_mea_rfs);
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
            _abs_map_rfs[map_rfs_index] = new_avg;
            localization.setTriangleTemplateData(_triangle_template);
           }

          }
         cur_his_rfs_cord->pos_array->pop_front();//delete the oldest one

        }
      }
    }
  }
}
