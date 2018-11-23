#include "filter_rfs_center.h"
// NB: a penalty will be applied if one of the constraints is > 0
// using the default adaptation to constraint(s) method
using namespace filter_rfs_center_space;
 FilterRfsCenter::FilterRfsCenter(){
   //ros::NodeHandle nh_("~");

   _judge_by_dist=true;//直接通过距离判断反光板
   _echo=100;
  _rf_radius=0.025;
  _step=0.0005;
   _err=0.0011;
    ROS_INFO_STREAM("pf_filter.construct.init done!");
}

 FilterRfsCenter::FilterRfsCenter(bool judge_by_dist,double reflector_radius,int echo_thread,double step,double err_thread):
   _judge_by_dist(judge_by_dist),_echo(echo_thread),_step(step),_err(err_thread),_rf_radius(reflector_radius){
 }


void FilterRfsCenter::getReflectorsCenter(const std::vector<sensor_msgs::LaserScan> & raw_scan,
                                    std::vector<VecPositionDir> & v_reflectors){
  std::vector<VecPositionDir> v_reflectors_center;
  groupingScan(raw_scan,_reflectors_group);
  getInitCenter(_reflectors_group,v_reflectors_center);
  getRelativePointClouds(_reflectors_group,v_reflectors_center,_relative_pointclounds);
  getOptimizedCenter(v_reflectors_center,_relative_pointclounds,v_reflectors);
}

void FilterRfsCenter::groupingScan(const std::vector<sensor_msgs::LaserScan>& raw_scan,
                  std::vector<std::list<scanCluster> > & v_reflectors_group){
  //average the dist
  v_reflectors_group.clear();
  sensor_msgs::LaserScan avegrage_scan;
  std::list<scanCluster> one_group_scan;
  avegrage_scan = raw_scan[0];
  double sum_dist =.0;
  double sum_echo =.0;
  double dist_min = raw_scan[0].range_min;
  double dist_max = raw_scan[0].range_max;

  scan_num = raw_scan[0].ranges.size();
  int size_v = raw_scan.size();
  //平均化几个连续激光束
  if(size_v >= 2){
    for(int i = 0 ;i < scan_num;i++){//laser dist data index
      sum_dist = .0;
      sum_echo = .0;
      for(int j = 0; j < size_v; j++)//laser index
      {
        sum_dist+=raw_scan[j].ranges[i];
        sum_echo+=raw_scan[j].intensities[i];
      }
      avegrage_scan.ranges[i] = sum_dist/size_v;
      avegrage_scan.intensities[i] = sum_echo/size_v;
    }
  }

  //filter by echo_thread
  //todo 这里要考虑靠近的时候，可能中间有间隔无强度信号，但是实际下一个有强度数据仍然是当前的反光板的问题
  //但是对于很远处，可能会存在只要间隔就是另一个反光板
  int diff_cnt = 0;
  bool last_is_diff= false;
  for(int i = 0; i < scan_num ; i++){
    double angle = avegrage_scan.angle_min + i * avegrage_scan.angle_increment;
    double dist = avegrage_scan.ranges[i];
    //超过强度阈值　当前组插入新数据点
    bool is_rf;
    if(_judge_by_dist){
      is_rf =(dist<dist_max &&dist>dist_min )?true:false;
    }
    else
    {
      is_rf =( avegrage_scan.intensities[i] > _echo)?true:false;
    }
    if(is_rf){

      last_is_diff  = false;
     //TODO we do not need compsate every scan beam ,for saving time,only need to update every rfs center!!!
      //compensateScanMoveDist(&angle,&dist);
      one_group_scan.push_back(scanCluster(angle,dist,avegrage_scan.intensities[i]));
    }
    else{
      //如果上次也低于阈值,继续累计
      if(last_is_diff){
       /* if(avegrage_scan.ranges[i]<10.0) *///在过滤模式下　强度值小于阈值的距离数据已经无效
          diff_cnt++;
      }
      //否则　重新累计
      else{
        last_is_diff  = true;
        diff_cnt = 0;
        diff_cnt++;
      }
      //如果连续两次是低于阈值，则步本簇数据采样完成
      if( diff_cnt >= 2 ){
        if(one_group_scan.size()){
          v_reflectors_group.push_back(one_group_scan);
          one_group_scan.clear();
        }
      }
      else{
        continue;
      }

    }
  }
  //ROS_INFO("pf_filter.grouping scan.group num: %d",v_reflectors_group.size());

}


void FilterRfsCenter::trimmingScanGroup(std::vector<std::list<scanCluster> > & v_reflectors_group){
  std::vector<std::list<scanCluster> > v_rf ;
  // step o
  for(int i = 0;i < v_reflectors_group.size();i++){
    std::list<scanCluster> tmp = v_reflectors_group[i];
    std::list<scanCluster> new_tmp;
    if(tmp.size() > sampe_limit_par1){
      //remove max min angle,max min dist;//waster time
      //update m[i];
    }
     std::list<scanCluster>::iterator it = tmp.begin();
     int k = 0;
     int p = 0;
    if(tmp.size() > sampe_limit_par2){
      double sum_dist = 0.0;
      double sum_angle = 0.0;
       int interv = tmp.size()/10;
       for(; k < 10;i++){
         for(;p < interv;p++){
           std::list<scanCluster>::iterator it_tmp =it++;
           sum_dist+=(it_tmp)->dist();
           sum_angle+=(it_tmp)->dist();
         }
         new_tmp.push_back(scanCluster(sum_angle/interv,sum_dist/interv,(--it)->echo()));//any echo is fit due to we do not use it from now
       }
    }
    v_rf.push_back(new_tmp);
  }
  v_reflectors_group = v_rf;
}

void FilterRfsCenter::getInitCenter(const std::vector<std::list<scanCluster> > & v_reflectors_group,
                   std::vector<VecPositionDir>& v_reflectors_center){
  v_reflectors_center.clear();
  int cnt;
  double sum_dist ;
  double sum_angle ;
  for(int i = 0 ;i <v_reflectors_group.size();i++ ){
    std::list<scanCluster> tmp = v_reflectors_group[i];
    std::list<scanCluster>::iterator it = tmp.begin();
    cnt = 0;
    sum_dist = 0.0;
    sum_angle = 0.0;
    for(;it!=tmp.end();it++){
      sum_dist += it->dist();
      sum_angle += it->angle();
      cnt++;
    }
    if(cnt)sum_dist /= cnt;
    if(cnt)sum_angle /= cnt;
    double new_dist = (sum_dist + 0.78*_rf_radius);
    double c_x = new_dist * cos(sum_angle);
    double c_y = new_dist * sin(sum_angle);
    v_reflectors_center.push_back(VecPositionDir(c_x,c_y,sum_angle));//store the angle for cal the relative pcl in getRelativePointClouds fun
  }
  //ROS_INFO_STREAM("filterscan.v_rfs_center.size():" << v_reflectors_center.size() );
}

void FilterRfsCenter::getRelativePointClouds(const std::vector<std::list<scanCluster> > & v_reflectors_group,
                            const std::vector<VecPositionDir>& v_reflectors_center,
                            std::vector<std::list<VecPositionDir> >& relative_pointclounds){
  int cnt = v_reflectors_group.size();
  double reverse_angle_deg =0.0;
  relative_pointclounds.clear();
  VecPositionDir tmp_center;
  std::list<scanCluster> tmp ;
  std::list<scanCluster>::iterator it;
  std::list<VecPositionDir> new_tmp;
  for(int i = 0; i < cnt; i++){
    new_tmp.clear();
    tmp = v_reflectors_group[i];
    tmp_center = v_reflectors_center[i];
    reverse_angle_deg = VecPositionDir::normalizeAngle( Rad2Deg( tmp_center.angle() - M_PI ) );
    it = tmp.begin();
    for( ;it!=tmp.end(); it++ ){
      VecPosition vecp(it->dist(),Rad2Deg(it->angle()),POLAR);
      VecPosition p(vecp);
      VecPosition np = p.globalToRelative(tmp_center,reverse_angle_deg);// the detail describered in Geometry.cpp
      VecPositionDir new_p(np,reverse_angle_deg);//this angle restored for calculating the abs cord next step
      new_tmp.push_back(new_p);
    }
    relative_pointclounds.push_back(new_tmp);
  }

}
void FilterRfsCenter::getFourDirValue( const std::list<VecPositionDir> &relative_pointclounds,
                       VecPositionDir cen,double *array){

  std::list<VecPositionDir>::const_iterator it = relative_pointclounds.begin();
  double cen_x = cen.getX();
  double cen_y = cen.getY();
  for(;it!=relative_pointclounds.end();it++){
    array[1] += pow( ( sqrt( pow( ( it->getX() - (cen_x-_step) ),2) + pow( (it->getY() - cen_y ),2 ) ) - _rf_radius),2);
    array[2] += pow( ( sqrt( pow( ( it->getX() - (cen_x+_step) ),2) + pow( (it->getY() - cen_y ),2 ) ) - _rf_radius),2);
    array[3] += pow( ( sqrt( pow( ( it->getX() - cen_x ),2 ) + pow( (it->getY() - (cen_y+_step ) ),2 ) ) - _rf_radius),2);
    array[4] += pow( ( sqrt( pow( ( it->getX() - cen_x ),2 ) + pow( (it->getY() - (cen_y-_step ) ),2 ) ) - _rf_radius),2);
  }
}
void FilterRfsCenter::move_center( std::list<VecPositionDir> & relative_pointclounds,
                 const double x,const double y,const double _step,int &kx,int &ky,double& sum){
  //双向搜索

    double sum_err_ml = 0.0;
    double sum_err_mr = 0.0;
    double sum_err_mu = 0.0;
    double sum_err_md = 0.0;
    double sum_err_cent = 0.0;
    kx = 0;
    ky = 0;
    int i = 0;
    std::list<VecPositionDir>::iterator it = relative_pointclounds.begin();
    for(;it!=relative_pointclounds.end();it++){
      sum_err_cent += pow((sqrt( pow( (it->getX() - x),2)+ pow( (it->getY() - y),2)) - _rf_radius),2);
        sum_err_ml += pow((sqrt( pow( (it->getX() - (x - _step)),2)+ pow( (it->getY() - y),2)) - _rf_radius),2);
        sum_err_mr += pow((sqrt( pow( (it->getX() - (x + _step)),2)+ pow( (it->getY() - y),2)) - _rf_radius),2);
        sum_err_mu += pow((sqrt( pow( (it->getX() - x),2) + pow( (it->getY() - (y+_step)),2)) - _rf_radius),2);
        sum_err_md += pow((sqrt( pow( (it->getX() - x),2) + pow( (it->getY() - (y-_step)),2)) - _rf_radius),2);
      i++;
    }
    if(i){
      sum_err_cent /= i;
      sum_err_ml /= i;
      sum_err_mr /= i;
      sum_err_mu /= i;
      sum_err_md /= i;
    }

    double array[5]={sum_err_cent,sum_err_ml,sum_err_mr,sum_err_mu,sum_err_md};
    int index = getMin(array,sum);

    if(index== 1)kx=-1;
    else if(index == 2)kx= 1;
    else if(index == 3)ky= 1;
    else if(index == 4)ky=-1;

}

void FilterRfsCenter::getOptimizedCenter(const std::vector<VecPositionDir>& v_pre_center,
                        const std::vector<std::list<VecPositionDir> > &relative_pointclounds,
                         std::vector<VecPositionDir>& v_opt_center
                        ){
  v_opt_center.clear();
  std::vector<VecPositionDir> v_center;
  std::vector<VecPositionDir> v_new_center;
  int len_group = relative_pointclounds.size();
  double x = 0;
  double y = 0;
  double v = 0.0;
  int kx = 1;
  int ky = 1;
  for(int i =0;i < len_group;i++){
    x = 0;
    y = 0;
    v = 0.0;
    kx = 1;
    ky = 1;
    std::list<VecPositionDir> tmp = relative_pointclounds[i];
    //method 1 min four dir err
    bool need_move = ( kx != 0 || ky != 0 );
    //bool not_boundary = (fabs(x)<=dia && fabs(y)<=dia);
    double err_bef = 99999;
    while(need_move){//如果仍然有偏移系数且未超出搜索范围，则继续搜索
      move_center(tmp,x,y,_step,kx,ky,v);
      //如果前后迭代偏差(unit mm(not mm^2))小于一个步长（很小）　或者误差已经小于指定阈值，则退出循环
      if( fabs(v - err_bef) < _step*_step || v < _err *_err)
      {
        // ROS_INFO("pf_filtercenter.arrive to precision.end this search loop");
        break;
      }
      else
        err_bef = v;

      x += kx*_step;
      y += ky*_step;
      need_move = (kx!=0 || ky!=0);
      ///not_boundary = (fabs(x)<=dia && fabs(y)<=dia);
    }
    v_center.push_back(VecPositionDir(x,y,v));//为了存储方便and后续处理，最后一个存得是累计和
  }
  //以当前计算中心点(x0,y0)为中心，分别计算x和y方向的中心偏移量,确定最终的中心
  double offset_x = 0;
  double offset_y = 0;
  for(int i = 0; i < len_group; i++ ){
    double array[5];
    array[0] = (v_center[i]).angle();//angle var store the var err in last step
    getFourDirValue(relative_pointclounds[i],v_center[i],array);
    offset_x = 0;
    offset_y = 0;
    double c1 = array[1]+array[2]-2*array[0] ;
    double c2 = array[3]+array[4]-2*array[0] ;
    if( c1 != 0 )
      offset_x=  _step*0.5*fabs( (array[1]-array[2])  / c1 );
    if( c2 != 0 )
      offset_y=  _step*0.5*fabs( (array[3]-array[4]) / c2 );
    double new_step=offset_x > offset_y ? offset_x:offset_y;
    if(new_step > _step)
      v_new_center.push_back(v_center[i]);
    else {
      double new_err = 0;
      double new_x = 0;
      double new_y = 0;
      const std::list<VecPositionDir> tmp = relative_pointclounds[i];
      std::list<VecPositionDir>::const_iterator it = tmp.begin();
      new_x = (v_center[i].getX() + new_step);
      new_y = (v_center[i].getY() + new_step);
      double x_var = 0;
      double y_var = 0;
      int cnt = 0;
      for(;it!=tmp.end();it++){
        x_var =  (it->getX() - new_x);
        y_var =  (it->getY() - new_y);
        new_err = pow( ( sqrt( x_var * x_var + y_var * y_var ) - _rf_radius ),2);
        cnt++;
      }
      if(cnt)new_err /= cnt;
      if( new_step <= _err ){
        if( new_err< (v_center[i]).angle() )
          v_new_center.push_back(VecPositionDir(new_x,new_y,new_err));
        else
          v_new_center.push_back(v_center[i]);
      }
     else{

        v_new_center.push_back(v_center[i]);
      #if 0
         if(new_err< (v_new_center[i]).angle() )v_new_center.push_back(VecPositionDir(new_x,new_y,new_err));
         else {
           //loop to move
         }
      #endif
     }
    }

   }

  //when we get the optimized center group in relative cordinate ,now we transform them to sick head cord
  for(int i = 0; i < len_group; i++ ){
    ROS_INFO("pf_scanfilter.movecenter rf index:%d,err(mm)(sqrt):%.5f",i,1000*sqrt(fabs(v_new_center[i].angle())));
    VecPositionDir new_relative_cen ( v_new_center[i]);
    VecPositionDir new_origin_cen( v_pre_center[i] );
    VecPosition tmp_p = new_relative_cen.relativeToGlobal(new_origin_cen,v_pre_center[i].angle());
    VecPositionDir new_p( tmp_p );// the detail describered in Geometry.cpp
    v_opt_center.push_back(new_p);

  }


}
