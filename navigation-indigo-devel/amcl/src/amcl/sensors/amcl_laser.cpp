/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
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
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>

#include "amcl/sensors/amcl_laser.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLLaser::AMCLLaser(size_t max_beams, map_t* map) : AMCLSensor(), 
                 max_samples(0), max_obs(0),
                 temp_obs(NULL)
{
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

AMCLLaser::~AMCLLaser()
{
  if(temp_obs){
  for(int k=0; k < max_samples; k++){
    delete [] temp_obs[k];
  }
  delete []temp_obs;
  }
}

void 
AMCLLaser::SetModelBeam(double z_hit,
                        double z_short,
                        double z_max,
                        double z_rand,
                        double sigma_hit,
                        double lambda_short,
                        double chi_outlier)
{
  this->model_type = LASER_MODEL_BEAM;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
}

void 
AMCLLaser::SetModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double max_occ_dist)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  map_update_cspace(this->map, max_occ_dist);
}

void 
AMCLLaser::SetModelLikelihoodFieldProb(double z_hit,
               double z_rand,
               double sigma_hit,
               double max_occ_dist,
               bool do_beamskip,
               double beam_skip_distance,
               double beam_skip_threshold,
               double beam_skip_error_threshold)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->do_beamskip = do_beamskip;
  this->beam_skip_distance = beam_skip_distance;
  this->beam_skip_threshold = beam_skip_threshold;
  this->beam_skip_error_threshold = beam_skip_error_threshold;
  map_update_cspace(this->map, max_occ_dist);
}


////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLLaser::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  if (this->max_beams < 2)
    return false;
  printf("AMCLLaser::UpdateSensor.this->model_type(0:beam 1:field 2:prob):%d\n",this->model_type);
  // Apply the laser sensor model
  if(this->model_type == LASER_MODEL_BEAM)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModel, data);  
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModelProb, data);  
  else
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLLaser::BeamModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  ///直接在单一激光射线方向上比较测量障碍物与理论地图障碍物的距离差的大小　作为　权重评价
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, data->range_max);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      //测量障碍物与地图障碍物位置距离(pow(z,2))越近，概率分配越大
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      // 地图障碍物在测量障碍物后方，认为测量障碍物可能是人等随机障碍物，分配概率大些
      if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      // 如果测量距离等于激光最大测量范围值，加入Z_MAX概率权重
      if(obs_range == data->range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if(obs_range < data->range_max)
        pz += self->z_rand * 1.0/data->range_max;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}
double AMCLLaser::evaluateOnePose(AMCLLaserData*  data, pf_vector_t pose){
  int i, j, step;
  double obs_range, obs_bearing;
  double p;
  double z, pz;
  AMCLLaser *self;
  self = (AMCLLaser*) data->sensor;
  double worst =  -999999;
  pf_vector_t hit;
  pf_vector_t map2base = pose;
  // Take account of the laser pose relative to the robot
  //map2base * base2laser = map2laser.
  pose = pf_vector_coord_add(self->laser_pose, map2base);///相对激光头[head]位置转到绝对激光位置
  //printf("AMCLLaser::evaluateOnePose.map2base:(%.6f,%.6f,%.6f).base2laser:(%.6f,%.6f,%.6f).map2laser:(%.6f,%.6f,%.6f)\n",
  //    map2base.v[0],map2base.v[1],map2base.v[2],
  //    self->laser_pose.v[0],self->laser_pose.v[1],self->laser_pose.v[2],
  //    pose.v[0],pose.v[1],pose.v[2]) ;
 p = 0;


  step = (data->range_count - 1) / (self->max_beams - 1);

  // Step size must be at least 1
  if(step < 1)
    step = 1;
  // 开始通过利用与最近物体的欧氏距离计算激光模型似然的算法，
  // 对所有特征（激光数据）进行遍历
  //printf("evaluate one scan.scan size:%d,step:%d\n",data->range_count,step);
  ///!!!step 间隔过大　不利于收敛（大部分数据没用上），故这里设为１(but influence speed)
  static bool cnt = 0;
  if(!cnt){
    printf("evaluate one scan.data->range_max:%.3f\n",data->range_max);
    cnt = true;
  }
  
  double max_err_bound_thread  = 0.05;
  double min_err_bound_thread  = 0.05;
  int mi, mj;
  for (i = 0; i < data->range_count; i += step)
  {
    //printf("AMCLLaser::evaluateOnePose.loop start \n");
    obs_range = data->ranges[i][0];
    obs_bearing = data->ranges[i][1];
    // This model ignores max range readings
    // 似然域测量模型简单地将最大距离读数丢弃
    if(obs_range >= data->range_max - max_err_bound_thread || obs_range <= min_err_bound_thread  )
      continue;

    // Check for NaN
    if(obs_range != obs_range)
      continue;

    hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);///相对激光点[endpoint]位置转到绝对激光点位置
    hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

    mi = MAP_GXWX(self->map, hit.v[0]);
    mj = MAP_GYWY(self->map, hit.v[1]);

    // Part 1: Get distance from the hit to closest obstacle.
    // Off-map penalized as max distance
    if(!MAP_VALID(self->map, mi, mj)){
     // printf("AMCLLaser::evaluateOnePose.obs cord outbound.use min prob value\n");
      z = self->map->max_occ_dist;
    }
    else{
     // printf("AMCLLaser::evaluateOnePose.obs cord outbound.use prob value\n");
      z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
    }
    p += -z;

  }
//  if(p >= 0 ){
//    printf("hyoou!\n");
//  }
  return p;
}
double AMCLLaser::LikelihoodFieldModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  ///使用最近邻搜索，make sure 测量障碍物点的占用概率，作为权重策略
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  printf("AMCLLaser::LikelihoodFieldModel.set->sample_count:%d."
         "laser data->range_count:%d.laser max_beams:%d\n",
         set->sample_count,
         data->range_count,
         self->max_beams);
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;//遍历每个粒子，这是粒子对应的位姿，是经运动模型更新后先验位姿

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);///相对激光头[head]位置转到绝对激光位置

    p = 1.0;

    // Pre-compute a couple of things
    double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;//测量噪声的方差chq
    double z_rand_mult = 1.0/data->range_max;

    step = (data->range_count - 1) / (self->max_beams - 1);

    // Step size must be at least 1
    if(step < 1)
      step = 1;
    // 开始通过利用与最近物体的欧氏距离计算激光模型似然的算法，
    // 对所有特征（激光数据）进行遍历
    //printf("AMCLLaser::LikelihoodFieldModel.step:%d\n",step);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      // 似然域测量模型简单地将最大距离读数丢弃
      if(obs_range >= data->range_max)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);///相对激光点[endpoint]位置转到绝对激光点位置
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj))
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
      //将正态分布与均匀分布混合后得到的似然结果
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

double AMCLLaser::LikelihoodFieldModelProb(AMCLLaserData *data, pf_sample_set_t* set)
{
  ///与上一个LikelihoodFieldModel模型相比，多了激光（人群 and so on ）概率剔除处理
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;
  //ceil 是天花板的意思，有向上的意思 返回大于或者等于指定表达式的最小整数
  //floo 是地面，地板的意思，有下面的意思，所以，此函数是向下取整
  //round 大约，环绕，在某某四周，附近的意思，所以，可以取其大约的意思，在函数中是四舍五入
  step = ceil((data->range_count) / static_cast<double>(self->max_beams));
  
  // Step size must be at least 1
  if(step < 1)
    step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
  double z_rand_mult = 1.0/data->range_max;

  double max_dist_prob = exp(-(self->map->max_occ_dist * self->map->max_occ_dist) / z_hit_denom);

  //Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  //prevents correct particles from getting down weighted because of unexpected obstacles 
  //such as humans 

  bool do_beamskip = self->do_beamskip;
  double beam_skip_distance = self->beam_skip_distance;
  double beam_skip_threshold = self->beam_skip_threshold;
  
  //we only do beam skipping if the filter has converged 
  if(do_beamskip && !set->converged){
    do_beamskip = false;
  }

  //we need a count the no of particles for which the beam agreed with the map 
  int *obs_count = new int[self->max_beams]();

  //we also need a mask of which observations to integrate (to decide which beams to integrate to all particles) 
  bool *obs_mask = new bool[self->max_beams]();
  
  int beam_ind = 0;
  
  //realloc indicates if we need to reallocate the temp data structure needed to do beamskipping 
  bool realloc = false; 

  if(do_beamskip){
    if(self->max_obs < self->max_beams){
      realloc = true;
    }

    if(self->max_samples < set->sample_count){
      realloc = true;
    }

    if(realloc){
      self->reallocTempData(set->sample_count, self->max_beams);     
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples, self->max_obs);
    }
  }
  ///遍历整个粒子集合，重新计算权重
  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;
    /// !!!chq every sample check loop reset the  beam_ind
    beam_ind = 0;

    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {//range_count = laser range.size() chq
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max){
        continue;
      }

      // Check for NaN
      if(obs_range != obs_range){
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      // 测量到地图边界　惩罚为最大距离对应的概率
      if(!MAP_VALID(self->map, mi, mj)){
        pz += self->z_hit * max_dist_prob;
       }
      else{
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
        if(z < beam_skip_distance){
          obs_count[beam_ind] += 1;//
        }
        pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }
       
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0); 
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings
            
      if(!do_beamskip){
        log_p += log(pz);
      }
      else{
         self->temp_obs[j][beam_ind] = pz;
      }
    }
    if(!do_beamskip){
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }
  
  if(do_beamskip){
    int skipped_beam_count = 0; 
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
      if((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold){
        obs_mask[beam_ind] = true;
      }
      else{
        obs_mask[beam_ind] = false;
        skipped_beam_count++;
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map 
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to 
    //the right solution
    bool error = false; 
    ///如果剔除的激光数量超过error_skip阈值，认为收敛到错误位置
    if(skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold)){
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold));
      error = true; 
    }

    for (j = 0; j < set->sample_count; j++)
    {
      sample = set->samples + j;
      pose = sample->pose;

      log_p = 0;

      for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
        if(error || obs_mask[beam_ind]){
          log_p += log(self->temp_obs[j][beam_ind]);
        }
      }
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }

  delete [] obs_count; 
  delete [] obs_mask;
  return(total_weight);
}

void AMCLLaser::reallocTempData(int new_max_samples, int new_max_obs){
  if(temp_obs){
    for(int k=0; k < max_samples; k++){
      delete [] temp_obs[k];
    }
    delete []temp_obs; 
  }
  max_obs = new_max_obs; 
  max_samples = fmax(max_samples, new_max_samples); 

  temp_obs = new double*[max_samples]();
  for(int k=0; k < max_samples; k++){
    temp_obs[k] = new double[max_obs]();
  }
}
