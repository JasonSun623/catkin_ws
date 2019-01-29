/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <fstream>
#include <iostream>
using namespace std;
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
/**
 * @brief Map generation node.
 */
class MapGenerator 
{

  public:
    MapGenerator(const std::string& mapname) : mapname_(mapname), saved_map_(false)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);

     ///step 1---------------------------- save to pgm format
      std::string mapdatafile = mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }
      fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] == 0) { //occ [0,0.1)
            fputc(254, out);
          } else if (map->data[i] == +100) { //occ (0.65,1]
            fputc(000, out);
          } else { //occ [0.1,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);


      std::string mapmetadatafile = mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

       */
      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Step 1 Save pgm map Done!\n");

      ///-----------------step 2 save to .map format added by chq
      std::string map_file = mapname_ + ".map";
      ROS_INFO("Writing map occupancy data to %s", map_file.c_str());
      ofstream out_map;
      out_map.open(map_file.c_str(), ios::out|ios::trunc);
      if (!out_map)
      {
        ROS_ERROR("Couldn't save map file to %s", map_file.c_str());
        return;
      }
      double origin[3] = {
        map->info.origin.position.x ,
        map->info.origin.position.y ,
        map->info.origin.position.z
      };
      int h = map->info.height;
      int w = map->info.width;
      double res = map->info.resolution;
      std::vector<pair<int,int> > mPoints,mUnKnownPoints;
      //首先整理数据，障碍物数据　与　位置数据　分别各归为一类
      int v;
      int w_x,w_y;
      //这里的索引对应的就是地图数据索引　而不是　图像索引
      //so 00 refer to lef down cord data
      //w h refer to topup cord data
      for(int j = 0; j < h; j++)
      {
        for (int i = 0; i < w; i++)
        {
           v = map->data[MAP_IDX( w,i, j)];
           w_x = (int)(1000.0*(origin[0] + i * res));//origin[0-2] refer to  the low -left 2d pos(*1000 refer to  m to mm)
           w_y = (int)(1000.0*(origin[1] + j * res));
           if (v == 0) { //occ [0,0.1)
             //blank do nothing
           } else if (v == +100) { //occ (0.65,1]
             mPoints.push_back( std::make_pair(w_x,w_y) );
           }
           ///!-------为了节省内存，不保存unknowndata--------!////
           ///else { //occ [0.1,0.65]
           ///  mUnKnownPoints.push_back( std::make_pair(w_x,w_y) );
           ///}
        }
      }
      //cal max pos to store
      w_x = (int)(1000.0*(origin[0] + w * res));//origin[0-2] refer to  the low -left 2d pos(*1000 refer to  m to mm)
      w_y = (int)(1000.0*(origin[1] + h * res));
      //然后分开存储
      //store base info
      int allsize = mPoints.size()+mUnKnownPoints.size();
      out_map << "2D-Map" << std::endl;
      out_map << "MinPos: " << (int)(origin[0]*1000) << " " << (int)(origin[1]*1000) << std::endl;
      out_map << "MaxPos: " << w_x  << " " << w_y << std::endl;
      out_map << "NumPoints: " << allsize << std::endl;
      out_map << "Resolution: "<< res*1000 << std::endl;//m to mm
      out_map <<"DATA"<<std::endl;
      for(int i = 0 ; i<mPoints.size();i++ ){
        out_map << mPoints[i].first << " " << mPoints[i].second<<std::endl;
      }
      out_map <<"UNKNOWN DATA"<<std::endl;
      for(int i = 0 ; i < mUnKnownPoints.size(); i++ ){
        out_map << mUnKnownPoints[i].first << " " << mUnKnownPoints[i].second<<std::endl;
      }
      ROS_INFO("Step 2 Save .map format map Done!\n");
      saved_map_ = true;
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    bool saved_map_;

};

#define USAGE "Usage: \n" \
              "  map_saver -h\n"\
              "  map_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "map";

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  if(*mapname.rbegin() == '/')
    mapname += "map";
  
  MapGenerator mg(mapname);

  while(!mg.saved_map_ && ros::ok())
    ros::spinOnce();

  return 0;
}


