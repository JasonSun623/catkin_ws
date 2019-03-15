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
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "amcl/pf/pf.h"
#include "amcl/pf/pf_pdf.h"
#include "amcl/pf/pf_kdtree.h"


// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t *pf, int k);

// Re-compute the cluster statistics for a sample set
static void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set);


// Create a new filter
pf_t *pf_alloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               pf_init_model_fn_t random_pose_fn, void *random_pose_data)
{
  int i, j;
  //chq pf_t :not only included the type pf_sample_set_t
  //but also included para like min_samples and max_samples.
  //it is a type included entire info about pf for amcl
  pf_t *pf;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  
  srand48(time(NULL));

  pf = calloc(1, sizeof(pf_t));

  pf->random_pose_fn = random_pose_fn;
  pf->random_pose_data = random_pose_data;

  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile(标准正态分位数) for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5; 
  
  pf->current_set = 0;
  for (j = 0; j < 2; j++)
  {
    set = pf->sets + j;
    set->sample_count = max_samples;
    //calloc Allocate NMEMB : max_samples elements of SIZE bytes:sizeof(pf_sample_t) each, all initialized to 0
    set->samples = calloc(max_samples, sizeof(pf_sample_t));//set->samples: a pf_samplet_t ptr type

    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
    }

    // HACK: is 3 times max_samples enough?
    set->kdtree = pf_kdtree_alloc(3 * max_samples);

    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;

  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  //set converged to 0
  pf_init_converged(pf);

  return pf;
}

// Free an existing filter
void pf_free(pf_t *pf)
{
  int i;
  
  for (i = 0; i < 2; i++)
  {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);
  
  return;
}

// Initialize the filter using a guassian
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  pf_pdf_gaussian_t *pdf;
  
  set = pf->sets + pf->current_set;
  
  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;
  ///disable by chq

  pdf = pf_pdf_gaussian_alloc(mean, cov);

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = pf_pdf_gaussian_sample(pdf);

    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }
  #if 0
  double search_radius = 5;
  for (i = 0; i < set->sample_count; i++){
    for(;;)
    {
      double x = mean[0]+drand48()*2*search_radius - search_radius;
      double y = mean[1]+drand48()*2*search_radius - search_radius;

      // Check that it's a free cell
      int i,j;
      i = MAP_GXWX(map,x);
      j = MAP_GYWY(map,y);
      if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
        break;
    }
  }
  #endif
  pf->w_slow = pf->w_fast = 0.0;

  pf_pdf_gaussian_free(pdf);
    
  // Re-compute cluster statistics
  pf_cluster_stats(pf, set); 

  //set converged to 0
  pf_init_converged(pf);

  return;
}

pf_vector_t pf_get_one_guassian_sample(pf_t *pf, pf_vector_t mean, pf_matrix_t cov){
  pf_vector_t pose;
  pf_pdf_gaussian_t *pdf;
  pdf = pf_pdf_gaussian_alloc(mean, cov);
  pose = pf_pdf_gaussian_sample(pdf);
  return pose;
}
void resetOneCluster(pf_t *pf,pf_vector_t pose){
  pf_sample_set_t *set;
  pf_sample_t *sample;
  //pf有两个集合，选择当前在用的集合
  set = pf->sets + pf->current_set;
  pf_kdtree_clear(set->kdtree);
  set->sample_count = 1;

  sample = set->samples;
  sample->weight = 1.0;
  sample->pose = pose;
  pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  pf->w_slow = pf->w_fast = 0.0;
   pf_cluster_stats(pf, set);
  pf_init_converged(pf);
}

void clustingPfSet(pf_t *pf,bool if_set_uniform_w){
  pf_sample_set_t *set;
  pf_sample_t *sample;
  //pf有两个集合，选择当前在用的集合
  set = pf->sets + pf->current_set;
  int size = set->sample_count;
  if(!size)return;
  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  // Compute the new sample poses
  int i = 0;
  for (; i < size; i++)
  {
    sample = set->samples+i;
    if(if_set_uniform_w)sample->weight = 1.0/(double)size;
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }
  pf->w_slow = pf->w_fast = 0.0;
  // Re-compute cluster statistics
  //printf("pf.resetAndClustingPfSet.pf_cluster_stats\n");
  pf_cluster_stats(pf, set);
  //printf("pf.resetAndClustingPfSet.pf_init_converged\n");
  //set converged to 0
  pf_init_converged(pf);
}
void resetAndClustingPfSet(pf_t *pf,int size,pf_vector_t* rand_pos_set, bool clusting){
  int i;


  //printf("pf.resetAndClustingPfSet.start...");
  //printf("pf.resetAndClustingPfSet.size:%d\n",size);
  if(!size)return;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  //pf有两个集合，选择当前在用的集合
  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = size;
  // Compute the new sample poses
  double weight = 1.0 / size;
  for (i = 0; i < size; i++)
  {
    // Add sample to histogram
    //printf("pf.resetThePfSet.pf_kdtree_insert.v_mean[%d]:(%.6f,%.6f,%.6f) w:%.6f\n",i,v_mean[i].v[0],v_mean[i].v[1],v_mean[i].v[2],weight);
    sample = set->samples + i;
    sample->weight = weight;
    sample->pose = rand_pos_set[i];
    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);

  }
  pf->w_slow = pf->w_fast = 0.0;
  if(!clusting)return;
  // Re-compute cluster statistics
  //printf("pf.resetAndClustingPfSet.pf_cluster_stats\n");
  pf_cluster_stats(pf, set);
  //printf("pf.resetAndClustingPfSet.pf_init_converged\n");
  //set converged to 0
  pf_init_converged(pf);
}


void resetThePfSet(pf_t *pf, pf_vector_t* v_mean){
  int i;
  int max_count = pf->max_samples;
  if(!max_count)return;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = max_count;
  // Compute the new sample poses
  double weight = 1.0 / max_count;
  for (i = 0; i < max_count; i++)
  {
    // Add sample to histogram
    //printf("pf.resetThePfSet.pf_kdtree_insert.v_mean[%d]:(%.6f,%.6f,%.6f) w:%.6f\n",i,v_mean[i].v[0],v_mean[i].v[1],v_mean[i].v[2],weight);
    sample = set->samples + i;
    sample->weight = weight;
    sample->pose = v_mean[i];
    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);

  }
  pf->w_slow = pf->w_fast = 0.0;
  // Re-compute cluster statistics
  //printf("pf.resetThePfSet.pf_cluster_stats\n");
  pf_cluster_stats(pf, set);
 // printf("pf.resetThePfSet.pf_init_converged\n");
  //set converged to 0
  pf_init_converged(pf);

}


// Initialize the filter using some model
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = (*init_fn) (init_data);
    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);
  
  //set converged to 0
  pf_init_converged(pf);

  return;
}
//init model by split map
//chq 将地图拆分成很多小块，每块执行一次粒子聚合，每块得到分值最高粒子，然后再进行一次聚合。保证每一块都有建议解
//void pf_init_model(pf_t *pf, void *init_data){

//}

void pf_init_converged(pf_t *pf){
  pf_sample_set_t *set;
  set = pf->sets + pf->current_set;
  set->converged = 0; 
  pf->converged = 0; 
}

int pf_update_converged(pf_t *pf)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;

  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;
  
  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;
    if(fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold || 
       fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold){
      set->converged = 0; 
      pf->converged = 0; 
      return 0;
    }
  }
  set->converged = 1; 
  pf->converged = 1; 
  return 1; 
}

// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn, void *action_data)
{
  pf_sample_set_t *set;

  set = pf->sets + pf->current_set;

  (*action_fn) (action_data, set);
  
  return;
}


#include <float.h>
// Update the filter with some new sensor observation
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn, void *sensor_data)
{
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;

  set = pf->sets + pf->current_set;

  // Compute the sample weights
  total = (*sensor_fn) (sensor_data, set);
  printf("pf_update_sensor.total:%.9f\n",total);
  if (total > 0.0)
  {
    // Normalize weights
    double w_avg=0.0;
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      w_avg += sample->weight;
      sample->weight /= total;
    }
    // Update running averages of likelihood of samples (Prob Rob p258)
    // 更新短期似然平均与长期似然平均
    w_avg /= set->sample_count;
    if(pf->w_slow == 0.0)
      pf->w_slow = w_avg;
    else
      pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);
    if(pf->w_fast == 0.0)
      pf->w_fast = w_avg;
    else
      pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    //printf("w_avg: %e slow: %e fast: %e\n", 
           //w_avg, pf->w_slow, pf->w_fast);
  }
  else
  {
    // Handle zero total
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }

  return;
}


// Resample the distribution
void pf_update_resample(pf_t *pf)
{
  int i;
  double total;
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b;

  //double r,c,U;
  //int m;
  //double count_inv;
  double* c;

  double w_diff;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  c = (double*)malloc(sizeof(double)*(set_a->sample_count+1));
  c[0] = 0.0;
  for(i=0;i<set_a->sample_count;i++)
    c[i+1] = c[i]+set_a->samples[i].weight;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);
  
  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;
  ///chq
  /// prob robotics cn.195
  /// w_fast 短期似然  w_slow 长期似然
  /// if w_fast is better than w_slow (or equal to) then do not add the random sample prob
  /// otherwise add the prob by their ratio .
  w_diff = 1.0 - pf->w_fast / pf->w_slow;///( 0 < alpha_slow < alpha_fast)
  if(w_diff < 0.0)
    w_diff = 0.0;
  //printf("w_diff: %9.6f\n", w_diff);

  // Can't (easily) combine low-variance sampler with KLD adaptive
  // sampling, so we'll take the more traditional route.
  /*
  // Low-variance resampler, taken from Probabilistic Robotics, p110
  count_inv = 1.0/set_a->sample_count;
  r = drand48() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  m = 0;
  */
  while(set_b->sample_count < pf->max_samples)
  {
    sample_b = set_b->samples + set_b->sample_count++;
    /// chq We want the random particle to be distributed around the best particles,
    //  instead of be spread too far, so we need to rewrite the random position generator.
    if(drand48() < w_diff){//生成一个新的位置
      ///sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);//chq disabled

      /// chq following method has a disadvantage
      // that once upon the loc err bet real pos and loc pos is beyond the limit search radius
      // there would never be a chance to recovery to real pos
      ///(so it can not resolve the kidnapped problem )
      // because we only generate random recovery particles in limit area
      ///But the advantage are :
      //1:the loc reliability will be strong becaused it never will fly to other way in limit speed
      //(speed <= search_radius / loop_peirod)
      //2:if the pos is lost in limit fast speed,it can be recovery by resampling --
      //--only if it don't be move too far by  hanging in the air
      //(once upon the loc reliability  below to thread,the w_diff para will be increased,
      //then the random num particles will be increased )
      //it will be benifit be reloc
       pf_vector_t temp = (pf->limit_random_pose_fn)(pf,pf->random_pose_data);
      if((temp.v[0] ||temp.v[2] ||temp.v[3]))
        sample_b->pose =temp;
    }
    else
    {
      // Can't (easily) combine low-variance sampler with KLD adaptive
      // sampling, so we'll take the more traditional route.
      /*
      // Low-variance resampler, taken from Probabilistic Robotics, p110
      U = r + m * count_inv;
      while(U>c)
      {
        i++;
        // Handle wrap-around by resetting counters and picking a new random
        // number
        if(i >= set_a->sample_count)
        {
          r = drand48() * count_inv;
          c = set_a->samples[0].weight;
          i = 0;
          m = 0;
          U = r + m * count_inv;
          continue;
        }
        c += set_a->samples[i].weight;
      }
      m++;
      */

      // Naive discrete event sampler
      double r;
      r = drand48();
      for(i=0;i<set_a->sample_count;i++)
      {
        if((c[i] <= r) && (r < c[i+1]))
          break;
      }
      assert(i<set_a->sample_count);

      sample_a = set_a->samples + i;//轮盘赌方式选择一个老数据

      assert(sample_a->weight > 0);

      // Add sample to list
      sample_b->pose = sample_a->pose;
    }

    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count))
      break;
  }
  
  // Reset averages, to avoid spiraling off into complete randomness.
  if(w_diff > 0.0)
    pf->w_slow = pf->w_fast = 0.0;

  //fprintf(stderr, "\n\n");

  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++)
  {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }
  
  // Re-compute cluster statistics
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  pf->current_set = (pf->current_set + 1) % 2;// after resampled ,更新为 set: set_b

  pf_update_converged(pf);

  free(c);
  return;
}


// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int pf_resample_limit(pf_t *pf, int k)
{
  double a, b, c, x;
  int n;

  if (k <= 1)
    return pf->max_samples;

  a = 1;
  b = 2 / (9 * ((double) k - 1));
  c = sqrt(2 / (9 * ((double) k - 1))) * pf->pop_z;
  x = a - b + c;

  n = (int) ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

  if (n < pf->min_samples)
    return pf->min_samples;
  if (n > pf->max_samples)
    return pf->max_samples;
  
  return n;
}


// Re-compute the cluster statistics for a sample set
// 计算粒子集合的聚类统计特性
// 对属于同一kdtree类别的样本进行累计期望，并求均值，作为该cluster的均值。　同时求所有样本的期望和均值作为set集合期望均值
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set)
{
  int i, j, k, cidx;
  pf_sample_t *sample;
  pf_cluster_t *cluster;
  
  // Workspace
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  pf_kdtree_cluster(set->kdtree);
  
  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++)
  {
    cluster = set->clusters + i;
    cluster->count = 0;
    cluster->weight = 0;
    cluster->mean = pf_vector_zero();
    cluster->cov = pf_matrix_zero();

    for (j = 0; j < 4; j++)
      cluster->m[j] = 0.0;//mean
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->c[j][k] = 0.0;//covariance
  }

  // Initialize overall filter stats
  count = 0;
  weight = 0.0;
  set->mean = pf_vector_zero();
  set->cov = pf_matrix_zero();
  for (j = 0; j < 4; j++)
    m[j] = 0.0;
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      c[j][k] = 0.0;
  
  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    //printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2]);

    // Get the cluster label for this sample
    // 获得该样本的所属类标签
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count)
      continue;
    if (cidx + 1 > set->cluster_count)
      set->cluster_count = cidx + 1;
    
    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    //属于同一　cluster index 的样本　会累计权重
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);
   //不区分cluster　index的总累计
    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    /// 属于同一　cluster index 的样本　会累计cov
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
      {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
  }

  // Normalize
  /// 对每个cluster的位置和协方差　按照权重归一化
  for (i = 0; i < set->cluster_count; i++)
  {
    cluster = set->clusters + i;
        
    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);

    cluster->cov = pf_matrix_zero();

    // Covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
          cluster->mean.v[j] * cluster->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                                         cluster->m[3] * cluster->m[3]));

    //printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count, cluster->weight,
           //cluster->mean.v[0], cluster->mean.v[1], cluster->mean.v[2]);
    //pf_matrix_fprintf(cluster->cov, stdout, "%e");
  }

  // Compute overall filter stats
  /// 所有样本数据归一化
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));

  return;
}


// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var)
{
  int i;
  double mn, mx, my, mrr;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  
  set = pf->sets + pf->current_set;

  mn = 0.0;
  mx = 0.0;
  my = 0.0;
  mrr = 0.0;
  
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    mn += sample->weight;
    mx += sample->weight * sample->pose.v[0];
    my += sample->weight * sample->pose.v[1];
    mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
    mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
  }

  mean->v[0] = mx / mn;
  mean->v[1] = my / mn;
  mean->v[2] = 0.0;

  *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));

  return;
}


// Get the statistics for a particular cluster.
int pf_get_cluster_stats(pf_t *pf, int clabel, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov)
{
  pf_sample_set_t *set;
  pf_cluster_t *cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count)
    return 0;
  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}


