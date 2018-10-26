#include "pf_filtercenter/filterscan.h"
#include "GALGO_Lib/Galgo.hpp"
// objective class example
template <typename T>
class ScanObjective
{
public:
   // objective function example : Rosenbrock function
   // minimizing f(x,y) = (1 - x)^2 + 100 * (y - x^2)^2
   static std::vector<T> Objective(const std::vector<T>& x)
   {
     std::list<VecPositionDir>::iterator it = _static_relative_pcls.begin();
     T obj;
     static T var_r = _static_rf_radius*1000*_static_rf_radius*1000;
     int i = 0;
     for(;it!=_static_relative_pcls.end();it++){
       T sum=(it->getX()*1000 - x[0])*(it->getX()*1000 - x[0]);
       obj += -fabs( sum - var_r);
       sum=(it->getY()*1000 - x[1])*(it->getY()*1000 - x[1]);
       obj += -fabs( sum - var_r);
       i++;
     }
     obj /= i;
     //// NB: GALGO maximize by default so we will maximize -f(x,y)
      return {obj};
   }
   //std::vector<T> ScanConstraint(const std::vector<T>& x)
   //{
   //   return {x[0]*x[1]+x[0]-x[1]+1.5,10-x[0]*x[1]};
   //}
   static std::list<VecPositionDir> _static_relative_pcls;
   static T  _static_rf_radius;
private:


};
///!!!!!!!!!!!!!!!!!! we must init it
template<typename T>
T ScanObjective<T>::_static_rf_radius = 0;
///!!!!!!!!!!!!!!!!!
template<typename T>
std::list<VecPositionDir> ScanObjective<T>::_static_relative_pcls ;

// NB: a penalty will be applied if one of the constraints is > 0
// using the default adaptation to constraint(s) method

 filterScan::filterScan(){
  ;
}

filterScan::filterScan(double reflector_radius,int echo_thread,double step,double err_thread):
  _echo(echo_thread),_step(step),_err(err_thread){
  _rf_radius =reflector_radius;
  ;
}

void filterScan::getReflectorsCenter(const std::vector<sensor_msgs::LaserScan> & raw_scan,
                                    std::vector<VecPositionDir> & v_reflectors){
  std::vector<VecPositionDir> v_reflectors_center;
  groupingScan(raw_scan,_reflectors_group);
  getInitCenter(_reflectors_group,v_reflectors_center);
  getRelativePointClouds(_reflectors_group,v_reflectors_center,_relative_pointclounds);
  getOptimizedCenter(v_reflectors_center,_relative_pointclounds,v_reflectors);
}

void filterScan::groupingScan(const std::vector<sensor_msgs::LaserScan>& raw_scan,
                  std::vector<std::list<scanCluster> > & v_reflectors_group){
  //average the dist
  v_reflectors_group.clear();
  sensor_msgs::LaserScan avegrage_scan;
  std::list<scanCluster> one_group_scan;
  avegrage_scan = raw_scan[0];
  double sum_dist =.0;
  double sum_echo =.0;
  int length = raw_scan[0].ranges.size();
  int size_v = raw_scan.size();
  //average the vector scan data
  if(size_v >= 2){
    for(int i = 0 ;i < length;i++){//laser dist data index
      sum_dist =.0;
      sum_echo = .0;
      for(int j = 0 ; j < size_v;j++)//laser index
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
  for(int i = 0; i < length ; i++){
    double angle = avegrage_scan.angle_min + i * avegrage_scan.angle_increment;
    if(avegrage_scan.intensities[i] > _echo){
      one_group_scan.push_back(scanCluster(angle,avegrage_scan.ranges[i],avegrage_scan.intensities[i]));
    }
    else{
      if(one_group_scan.size()){
        v_reflectors_group.push_back(one_group_scan);
        one_group_scan.clear();
      }
    }
  }
  ROS_INFO("pf_filter.grouping scan.group num: %d",v_reflectors_group.size());

}
void filterScan::trimmingScanGroup(std::vector<std::list<scanCluster> > & v_reflectors_group){
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

void filterScan::getInitCenter(const std::vector<std::list<scanCluster> > & v_reflectors_group,
                   std::vector<VecPositionDir>& v_reflectors_center){
  v_reflectors_center.clear();
  for(int i = 0 ;i <v_reflectors_group.size();i++ ){
    std::list<scanCluster> tmp = v_reflectors_group[i];
    std::list<scanCluster>::iterator it = tmp.begin();
    double sum_dist = 0.0;
    double sum_angle = 0.0;
    int cnt = 0;
    for(;it!=tmp.end();it++){
      sum_dist+=it->dist();
      sum_angle+=it->angle();
      cnt++;
    }
    sum_dist /= cnt;
    sum_angle /= cnt;
    double new_dist = (sum_dist + 0.78*_rf_radius);
    double c_x = new_dist * cos(sum_angle);
    double c_y = new_dist * sin(sum_angle);
    v_reflectors_center.push_back(VecPositionDir(c_x,c_y,sum_angle));//store the angle for cal the relative pcl in getRelativePointClouds fun
  }
  std::cout << "filterscan.v_rfs_center.size():" << v_reflectors_center.size()<<std::endl;
}

void filterScan::getRelativePointClouds(const std::vector<std::list<scanCluster> > & v_reflectors_group,
                            const std::vector<VecPositionDir>& v_reflectors_center,
                            std::vector<std::list<VecPositionDir> >& relative_pointclounds){
  int cnt = v_reflectors_group.size();
  relative_pointclounds.clear();
  std::list<scanCluster> tmp ;
  VecPositionDir tmp_center;
  std::list<scanCluster>::iterator it;
  double reverse_angle_deg =0.0;
   std::list<VecPositionDir> new_tmp;
  for(int i = 0;i < cnt;i++){
    new_tmp.clear();
    tmp = v_reflectors_group[i];
    tmp_center = v_reflectors_center[i];
    reverse_angle_deg = VecPositionDir::normalizeAngle(Rad2Deg( tmp_center.angle()-M_PI));
    it = tmp.begin();
    for(;it!=tmp.end();it++){
     VecPosition vecp(it->dist(),Rad2Deg(it->angle()),POLAR);
     VecPosition p(vecp);
     VecPosition np = p.globalToRelative(tmp_center,reverse_angle_deg);// the detail describered in Geometry.cpp
     VecPositionDir new_p(np,reverse_angle_deg);//this angle restored for calculating the abs cord next step
     new_tmp.push_back(new_p);
    }
    relative_pointclounds.push_back(new_tmp);
  }

}
void filterScan::getFourDirValue( const std::list<VecPositionDir> &relative_pointclounds,
                       VecPositionDir cen,double *array){
#if 0
  std::list<VecPositionDir>::const_iterator it = relative_pointclounds.begin();
  double var_r = _rf_radius * _rf_radius;
  for(;it!=relative_pointclounds.end();it++){
    array[1] += fabs( pow( (it->getX() - (cen.getX()-_step)),2) - var_r);
    array[1] += fabs( pow( (it->getY() - cen.getY()),2) - var_r);

    array[2] += fabs( pow( (it->getX() - (cen.getX()+_step)),2) - var_r);
    array[2] += fabs( pow( (it->getY() - cen.getY()),2) - var_r);

    array[3] += fabs( pow( (it->getX() - cen.getX()),2) - var_r);
    array[3] += fabs( pow( (it->getY() - (cen.getY()+_step)),2) - var_r);

    array[4] += fabs( pow( (it->getX() - cen.getX()),2) - var_r);
    array[4] += fabs( pow( (it->getY() - (cen.getY()-_step)),2) - var_r);
  }
#endif
}
void filterScan::move_center( std::list<VecPositionDir> & relative_pointclounds,
                 const double x,const double y,const double _step,int &kx,int &ky,double& sum){
  //双向搜索

    double sum_err_ml =0.0;
    double sum_err_mr =0.0;
    double sum_err_mu =0.0;
    double sum_err_md =0.0;
    double sum_err_cent =0.0;
    kx = 0;
    ky = 0;
    std::list<VecPositionDir>::iterator it = relative_pointclounds.begin();
    double var_r = _rf_radius * _rf_radius;
    int i = 0;
    for(;it!=relative_pointclounds.end();it++){
      sum_err_cent += fabs( pow( (it->getX() - x),2) - var_r);
      sum_err_cent += fabs( pow( (it->getY() - y),2) - var_r);

      sum_err_ml += fabs( pow( (it->getX() - (x-_step)),2) - var_r);
      sum_err_ml += fabs( pow( (it->getY() - y),2) - var_r);

      sum_err_mr += fabs( pow( (it->getX() - (x+_step)),2) - var_r);
      sum_err_mr += fabs( pow( (it->getY() - y),2) - var_r);

      sum_err_mu += fabs( pow( (it->getX() - x),2) - var_r);
      sum_err_mu += fabs( pow( (it->getY() - (y+_step)),2) - var_r);

      sum_err_md += fabs( pow( (it->getX() - x),2) - var_r);
      sum_err_md += fabs( pow( (it->getY() - (y-_step)),2) - var_r);
      i++;
    }
    sum_err_cent/=i;
    sum_err_ml/=i;
    sum_err_mr/=i;
    sum_err_mu/=i;
    sum_err_md/=i;

    double array[5]={sum_err_cent,sum_err_ml,sum_err_mr,sum_err_mu,sum_err_md};
    int index = getMin(array,sum);

    if(index== 1)kx=-1;
    else if(index == 2)kx= 1;
    else if(index == 3)ky= 1;
    else if(index == 4)ky=-1;

}

void filterScan::getOptimizedCenter(const std::vector<VecPositionDir>& v_pre_center,
                        const std::vector<std::list<VecPositionDir> > &relative_pointclounds,
                         std::vector<VecPositionDir>& v_opt_center
                        ){
v_opt_center.clear();
  std::vector<VecPositionDir> v_center;
   std::vector<VecPositionDir> v_new_center;
  int len_group = relative_pointclounds.size();
  for(int i =0;i < len_group;i++){
    double x = 0;
    double y = 0;
    double v =0.0;
    int kx = 1;
    int ky = 1;

    std::list<VecPositionDir> tmp = relative_pointclounds[i];
    ///method 0 ga alogrithm
    galgo::Parameter<double> par1({-_rf_radius*1000,_rf_radius*1000});
    galgo::Parameter<double> par2({-_rf_radius*1000,_rf_radius*1000});
    // initiliazing genetic algorithm
    //ScanObjective<double>::_static_rf_radius = _rf_radius;
    //ScanObjective<double>::_static_relative_pcls = tmp;
    ScanObjective<double> t;
    t._static_relative_pcls = tmp;
    t._static_rf_radius = _rf_radius;

    galgo::GeneticAlgorithm<double> ga( ScanObjective<double>::Objective,100,1000,false,par1,par2);
    ga.covrate = 0.8;
    ga.mutrate = 0.05;
    ga.precision = 1;
    // setting constraints
    //ga.Constraint = MyObj<double>::MyConstraint;
    //ga.tolerance = -0.05*0.05;//terminal condition to stop the algorithm
    //ga.precision = 2;//number of decimals for outputting results
    ga.run();
    galgo::CHR<double> result(ga.result()) ;
    std::vector<double> para = result.get()->getParam();
    std::vector<double> err = result.get()->getResult();
    //v_new_center.push_back(VecPositionDir(para[0]/1000,para[1]/1000,err[0]/1000));
    ROS_INFO("pf_scanfilter.ga rf index:%d,err(mm)(sqrt):%.5f",i,sqrt(fabs(err[0])));

    //method 1 min four dir err
    bool need_move = (kx!=0 || ky!=0);
    double dia = 2.0*_rf_radius;
    bool not_boundary = (fabs(x)<=dia && fabs(y)<=dia);
    while(need_move /*&& not_boundary*/){//如果仍然有偏移系数且未超出搜索范围，则继续搜索
      move_center(tmp,x,y,_step,kx,ky,v);
      x += kx*_step;
      y += ky*_step;
      need_move = (kx!=0 || ky!=0);
      ///not_boundary = (fabs(x)<=dia && fabs(y)<=dia);
    }
    v_center.push_back(VecPositionDir(x,y,v));//为了存储方便and后续处理，最后一个存得是累计和
  }
  //以当前计算中心点(x0,y0)为中心，分别计算x和y方向的中心偏移量,确定最终的中心
  for(int i = 0; i < len_group; i++ ){
    double array[5];
    array[0] = (v_center[i]).angle();//angle var store the var err in last step
    getFourDirValue(relative_pointclounds[i],v_center[i],array);
    double offset_x=0;
    double offset_y=0;
    if((array[1]+array[2]-2*array[0])!=0)
      offset_x= _step*0.5*fabs((array[1]-array[2])/(array[1]+array[2]-2*array[0]));
    if((array[3]+array[4]-2*array[0])!=0)
      offset_y= _step*0.5*fabs((array[3]-array[4])/(array[3]+array[4]-2*array[0]));
    double new_step=offset_x>offset_y?offset_x:offset_y;
    if(new_step > _step)v_new_center.push_back(v_center[i]);
    else {
      double new_err =0;
      double new_x=0;
      double new_y=0;
      const std::list<VecPositionDir> tmp = relative_pointclounds[i];
      std::list<VecPositionDir>::const_iterator it = tmp.begin();
      double var_r = _rf_radius * _rf_radius;
      new_x = (v_center[i].getX()+new_step);
      new_y = (v_center[i].getY()+new_step);
      for(;it!=tmp.end();it++){
        new_err += fabs( pow( (it->getX() - new_x),2) - var_r);
        new_err += fabs( pow( (it->getY() - new_y),2) - var_r);
      }
      if(new_step <= _err){
        if(new_err< (v_center[i]).angle() )
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
    ROS_INFO("pf_scanfilter.moveventer rf index:%d,err(mm)(sqrt):%.5f",i,1000*sqrt(fabs(v_new_center[i].angle())));
    VecPositionDir new_relative_cen ( v_new_center[i]);
    VecPositionDir new_origin_cen( v_pre_center[i] );
    VecPosition tmp_p = new_relative_cen.relativeToGlobal(new_origin_cen,v_pre_center[i].angle());
    VecPositionDir new_p( tmp_p );// the detail describered in Geometry.cpp
    v_opt_center.push_back(new_p);

  }


}
