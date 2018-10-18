/*
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/* Author: Brian Gerkey */

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include "map_server/AbstractMap/SimpleGridMap.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <list>
#include <map>
#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapServer
{
  public:
  int  SplitString(const string &input,const char delimiter,std::vector<string> &results) {
      string text = input;
      results.clear();
      while (1) {
        int pos = (int) text.find(delimiter);

        if (pos == 0) {
          text = text.substr(1);
          continue;
        }
        if (pos < 0) {
          results.push_back(text);
          break;
        }

        string word = text.substr(0, pos);

        text = text.substr(pos + 1);
        results.push_back(word);
      }
      return 0;
    }
  void showMapInfo(SimpleGridMap &map){

    goal_marker_pub =  n.advertise<visualization_msgs::Marker>("visualization_goal_marker",1,true);
    dock_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_dock_marker",1,true);
    robothome_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_robothome_marker",1,true);
    //forbiddenarea_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_forbiddenarea_marker",1,true);
    forbiddenarea_array_marker_pub = n.advertise<visualization_msgs::MarkerArray>("forbiddenarea_MarkerArray", 1,true);
    text_array_marker_pub = n.advertise<visualization_msgs::MarkerArray>("text_MarkerArray", 1,true);
    forbiddenline_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_forbiddenline_marker",1,true);
    uint32_t shape = visualization_msgs::Marker::POINTS;
    uint32_t text_shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
    visualization_msgs::Marker text_marker;
    visualization_msgs::Marker goal_marker;
    visualization_msgs::Marker dock_marker;
    visualization_msgs::Marker robothome_marker;
    visualization_msgs::Marker forbiddenarea_marker;
    visualization_msgs::MarkerArray forbiddenarea_markerArray;
    visualization_msgs::MarkerArray text_MarkerArray;
    visualization_msgs::Marker forbiddenline_marker;
    int sum = 0;

    
    ROS_INFO("map_server.pub dock_points_ size:%d",(map.dock_points_).size() );
    if( map.dock_points_.size() ){
      dock_marker.points.clear();
      for(int i=0;i < (map.dock_points_).size();i++){
        
        OrientPos pos  = ((map.dock_points_)[i]).pos();
        string name =  ((map.dock_points_)[i]).name();
         //name =  "show time";
        sum+=1;
        dock_marker.id = sum;
        dock_marker.header.frame_id="/map";
        dock_marker.header.stamp = ros::Time::now();
        dock_marker.ns = "map_dock_space";
        // Set the marker type
        dock_marker.type = shape;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        dock_marker.action = visualization_msgs::Marker::ADD;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        dock_marker.scale.x = 0.5;
        dock_marker.scale.y = 0.5;
        dock_marker.scale.z = 0.5;
        // Set the color -- be sure to set alpha to something non-zero!
        //DarkOrchid	153 50 204
        dock_marker.color.r = 153;
        dock_marker.color.g = 50;
        dock_marker.color.b = 204;
        dock_marker.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = 0;

        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0;

        dock_marker.points.push_back(p);
        sum+=1;
        text_marker = dock_marker;
        text_marker.ns = "text_space";
        text_marker.color.r = 255;
        text_marker.color.g = 0;
        text_marker.color.b = 0;
        text_marker.color.a = 1;
        text_marker.id = sum;
        text_marker.type = text_shape;
        text_marker.text = name;
        text_marker.pose = pose;
        text_MarkerArray.markers.push_back(text_marker);
        ROS_INFO("map_server.pub_dock pos:(%f,%f)",pos.x(),pos.y() );
        dock_marker.lifetime = ros::Duration(1000000);
      }
        dock_marker_pub.publish(dock_marker);
    }
    
    ROS_INFO("map_server.pub goals size:%d",(map.goals_).size() );
    if((map.goals_).size() ){
      goal_marker.points.clear();
      for(int i=0;i < (map.goals_).size();i++){
        
        OrientPos pos  = ((map.goals_)[i]).pos();
        string name =  ((map.goals_)[i]).name();
        sum+=1;
        goal_marker.id = sum;
        goal_marker.header.frame_id="/map";
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.ns = "map_goal1_space";
        // Set the marker type
        goal_marker.type = shape;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        goal_marker.action = visualization_msgs::Marker::ADD;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        goal_marker.scale.x = 0.5;
        goal_marker.scale.y = 0.5;
        goal_marker.scale.z = 0.5;
        // Set the color -- be sure to set alpha to something non-zero!
        //SeaGreen1	84 255 159
        //AntiqueWhite3	205 192 176
        //Orange	255 165 0
        //        //PaleGreen1 154 255 154 for robothome
        goal_marker.color.r = 154;
        goal_marker.color.g = 255;
        goal_marker.color.b = 154;
        goal_marker.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = 0;
        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0;
        goal_marker.points.push_back(p);
        sum+=1;
        text_marker = goal_marker;
        text_marker.ns = "text_space";
        text_marker.color.r = 255;
        text_marker.color.g = 0;
        text_marker.color.b = 0;
        text_marker.color.a = 1;
        text_marker.id = sum;
        text_marker.type = text_shape;
        text_marker.text = name;
        text_marker.pose = pose;
        text_MarkerArray.markers.push_back(text_marker);
        ROS_INFO("map_server.pub_dock pos:(%f,%f)",pos.x(),pos.y() );
        goal_marker.lifetime = ros::Duration(1000000);
      }
      goal_marker_pub.publish(goal_marker);
    }
    

    ROS_INFO("map_server.pub robot_homes_ size:%d",(map.robot_homes_).size() );
    if((map.robot_homes_).size()){
      robothome_marker.points.clear();
      for(int i=0;i < (map.robot_homes_).size();i++){
        OrientPos pos  = ((map.robot_homes_)[i]).pos();
        string name =  ((map.robot_homes_)[i]).name();
        sum+=1;
        robothome_marker.id = sum;
        robothome_marker.header.frame_id="/map";
        robothome_marker.header.stamp = ros::Time::now();
        robothome_marker.ns = "map_robothome_space";
        // Set the marker type
        robothome_marker.type = shape;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        robothome_marker.action = visualization_msgs::Marker::ADD;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        robothome_marker.scale.x = 0.5;
        robothome_marker.scale.y = 0.5;
        robothome_marker.scale.z = 0.5;
        // Set the color -- be sure to set alpha to something non-zero!

         //Orange	255 165 0 for robothome
        robothome_marker.color.r = 255;
        robothome_marker.color.g = 165;
        robothome_marker.color.b = 0;
        robothome_marker.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = 0;
        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0;
        robothome_marker.points.push_back(p);
        sum+=1;
        text_marker = robothome_marker;
        text_marker.ns = "text_space";
        text_marker.color.r = 255;
        text_marker.color.g = 0;
        text_marker.color.b = 0;
        text_marker.color.a = 1;
        text_marker.id = sum;
        text_marker.type = text_shape;
        text_marker.text = name;
        text_marker.pose = pose;
        text_MarkerArray.markers.push_back(text_marker);
        ROS_INFO("map_server.pub goal_ pos:(%f,%f)",pos.x(),pos.y() );
        robothome_marker.lifetime = ros::Duration(1000000);
      }
      robothome_marker_pub.publish(robothome_marker);
    }
    

    shape = visualization_msgs::Marker::LINE_STRIP;
    forbiddenarea_marker.type = shape;
    ROS_INFO("map_server.pub forbidden_area_ size:%d",(map.forbidden_area_).size() );
    if( (map.forbidden_area_).size() ){
      forbiddenarea_markerArray.markers.clear();    
      for(int i=0;i < (map.forbidden_area_).size();i++){
        forbiddenarea_marker.points.clear();
        FreeRegion1 reg  = ((map.forbidden_area_)[i]).regin();
        string icon = ((map.forbidden_area_)[i]).name();
        OrientPos cen_pos = ((map.forbidden_area_)[i]).pos();
        std::vector<VecPosition> list = reg.list();
        sum += 1;
        forbiddenarea_marker.header.stamp = ros::Time::now();
        forbiddenarea_marker.id = sum;
        forbiddenarea_marker.header.frame_id="/map";
        forbiddenarea_marker.header.stamp = ros::Time::now();
        forbiddenarea_marker.ns = "map_forbiddenarea_space";
        // Set the marker type
        forbiddenarea_marker.type = shape;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        forbiddenarea_marker.action = visualization_msgs::Marker::ADD;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        //Line strips also have some special handling for scale:
        //only scale.x is used and it controls the width of the line segments.
        forbiddenarea_marker.scale.x =0.1;

        // Set the color -- be sure to set alpha to something non-zero!
        //VioletRed4	139 34 82
        forbiddenarea_marker.color.r = 139;
        forbiddenarea_marker.color.g = 34;
        forbiddenarea_marker.color.b = 82;
        forbiddenarea_marker.color.a = 1.0;

        geometry_msgs::Point p;
        p.z = 0;
        
        geometry_msgs::Pose pose;
        pose.position.x = 0.5*(list[0].getX()+list[2].getX());
        pose.position.y = 0.5*(list[0].getY()+list[2].getY());
        pose.position.z = 0;
        for(int j =0; j < list.size();j++){
          p.x = list[j].getX();
          p.y = list[j].getY();
          forbiddenarea_marker.points.push_back(p);
        }
        //add first to end line
        p.x = list[0].getX();
        p.y = list[0].getY();
        forbiddenarea_marker.points.push_back(p);
        sum+=1;
        text_marker = forbiddenarea_marker;
        text_marker.ns = "text_space";
        text_marker.scale.x = 0.5;
        text_marker.scale.y = 0.5;
        text_marker.scale.z = 0.5;
        text_marker.color.r = 255;
        text_marker.color.g = 0;
        text_marker.color.b = 0;
        text_marker.color.a = 1;
        text_marker.id = sum;
        text_marker.type = text_shape;
        text_marker.text = icon;
        text_marker.pose = pose;
        text_MarkerArray.markers.push_back(text_marker);
        forbiddenarea_marker.lifetime =  ros::Duration(1000000);
        forbiddenarea_markerArray.markers.push_back(forbiddenarea_marker);
      }
      forbiddenarea_array_marker_pub.publish(forbiddenarea_markerArray);
    }
    
    //Line lists use the points member of the visualization_msgs/Marker message.
    //It will draw a line between each pair of points, so 0-1, 2-3, 4-5, ...
    shape = visualization_msgs::Marker::LINE_LIST;
    ROS_INFO("map_server.pub forbidden_line_ size:%d",(map.forbidden_line_).size() );
    if((map.forbidden_line_).size()){
      forbiddenline_marker.points.clear();
      for(int i=0;i < (map.forbidden_line_).size();i++){
        OrientPos s_pos,e_pos;
        ((map.forbidden_line_)[i]).getPairPos(s_pos,e_pos);
        string icon = ((map.forbidden_line_)[i]).name();
        sum += 1;
        forbiddenline_marker.header.stamp = ros::Time::now();
        forbiddenline_marker.id = sum;
        forbiddenline_marker.header.frame_id="/map";
        forbiddenline_marker.ns = "map_forbidden_line_space";
        // Set the marker type
        forbiddenline_marker.type = shape;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        forbiddenline_marker.action = visualization_msgs::Marker::ADD;
        //Line strips also have some special handling for scale:
        //only scale.x is used and it controls the width of the line segments.
        forbiddenline_marker.scale.x = 0.1;
        // DarkGoldenrod1	255 185 15
        forbiddenline_marker.color.r = 255;
        forbiddenline_marker.color.g = 185;
        forbiddenline_marker.color.b = 15;
        forbiddenline_marker.color.a = 1;
        geometry_msgs::Point p;
        p.z = 0;
        //add first to end line
        p.x = s_pos.x();
        p.y = s_pos.y();
        forbiddenline_marker.points.push_back(p);
        p.x = e_pos.x();
        p.y = e_pos.y();
        
        geometry_msgs::Pose pose;
        pose.position.x = 0.5*(s_pos.x()+e_pos.x());
        pose.position.y = 0.5*(s_pos.y()+e_pos.y());
        pose.position.z = 0;

        forbiddenline_marker.points.push_back(p);
        sum+=1;
        text_marker = forbiddenline_marker;
        text_marker.ns = "text_space";
        text_marker.scale.x = 0.5;
        text_marker.scale.y = 0.5;
        text_marker.scale.z = 0.5;
        text_marker.color.r = 255;
        text_marker.color.g = 0;
        text_marker.color.b = 0;
        text_marker.color.a = 1;
        text_marker.id = sum;
        text_marker.type = text_shape;
        text_marker.text = icon;
        text_marker.pose = pose;
        text_MarkerArray.markers.push_back(text_marker);
        forbiddenline_marker.lifetime = ros::Duration(1000000);
      }
      forbiddenline_marker_pub.publish(forbiddenline_marker); 
    }
     text_array_marker_pub.publish(text_MarkerArray); 
  }

    /** Trivial constructor */
    MapServer(const std::string& fname, double res)
    { 
      std::string mapfname = "";
      double origin[3];
      int negate;
      double occ_th, free_th;
      MapMode mode = TRINARY;
      std::string frame_id;
      ros::NodeHandle private_nh("~");
      
      private_nh.param("frame_id", frame_id, std::string("map"));
      std::vector<std::string> sep_tmp;
      SplitString(fname,'.',sep_tmp);
      if(sep_tmp.size() < 2){
        ROS_ERROR("Map_server File Format Error!file name: %s.", fname.c_str());
        exit(-1);
      }
      else{
        //read .map map
        //default conf
        negate = 0;
        res = 0.05;
        occ_th = 0.65;
        free_th = 0.196;

        if(sep_tmp[sep_tmp.size()-1] == "map"){

          SimpleGridMap map_;
          map = map_;
          map.LoadFromFile(fname,&map_resp_,res);
          
         }



        //read image map
        else{
          deprecated = (res != 0);
          //method 1 read map info from yaml file
          if (!deprecated) {
            //mapfname = fname + ".pgm";
            //std::ifstream fin((fname + ".yaml").c_str());
            std::ifstream fin(fname.c_str());
            if (fin.fail()) {
              ROS_ERROR("Map_server could not open %s.", fname.c_str());
              exit(-1);
            }
          #ifdef HAVE_NEW_YAMLCPP
            // The document loading process changed in yaml-cpp 0.5.
            YAML::Node doc = YAML::Load(fin);
          #else
            YAML::Parser parser(fin);
            YAML::Node doc;
            parser.GetNextDocument(doc);
          #endif
            try {
              doc["resolution"] >> res;
            } catch (YAML::InvalidScalar) {
              ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
              exit(-1);
            }
            try {
              doc["negate"] >> negate;
            } catch (YAML::InvalidScalar) {
              ROS_ERROR("The map does not contain a negate tag or it is invalid.");
              exit(-1);
            }
            try {
              doc["occupied_thresh"] >> occ_th;
            } catch (YAML::InvalidScalar) {
              ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
              exit(-1);
            }
            try {
              doc["free_thresh"] >> free_th;
            } catch (YAML::InvalidScalar) {
              ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
              exit(-1);
            }
            try {
              std::string modeS = "";
              doc["mode"] >> modeS;

              if(modeS=="trinary")
                mode = TRINARY;
              else if(modeS=="scale")
                mode = SCALE;
              else if(modeS=="raw")
                mode = RAW;
              else{
                ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
                exit(-1);
              }
            } catch (YAML::Exception) {
              ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
              mode = TRINARY;
            }
            try {
              doc["origin"][0] >> origin[0];
              doc["origin"][1] >> origin[1];
              doc["origin"][2] >> origin[2];
            } catch (YAML::InvalidScalar) {
              ROS_ERROR("The map does not contain an origin tag or it is invalid.");
              exit(-1);
            }
            try {
              doc["image"] >> mapfname;
              // TODO: make this path-handling more robust
              if(mapfname.size() == 0)
              {
                ROS_ERROR("The image tag cannot be an empty string.");
                exit(-1);
              }
              if(mapfname[0] != '/')
              {
                // dirname can modify what you pass it
                char* fname_copy = strdup(fname.c_str());
                mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
                free(fname_copy);
              }
            } catch (YAML::InvalidScalar) {
              ROS_ERROR("The map does not contain an image tag or it is invalid.");
              exit(-1);
            }
          }
          //method 2 read map info from launched configure param
          else {
            private_nh.param("negate", negate, 0);//whether the white/black free/occupied semantics should be reversed (interpretation of thresholds is unaffected)
            private_nh.param("occupied_thresh", occ_th, 0.65);//big than this is considered as occupied
            private_nh.param("free_thresh", free_th, 0.196);//low than this is considered as free
            mapfname = fname;
            origin[0] = origin[1] = origin[2] = 0.0;// The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation (yaw=0 means no rotation). Many parts of the system currently ignore yaw.
          }

          ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
          map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);

        }
      }


      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

      service = n.advertiseService("static_map", &MapServer::mapCallback, this);
      //pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1,

      // Latched publisher for metadata
      metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      metadata_pub.publish( meta_data_message_ );
      
      // Latched publisher for data
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
      map_pub.publish( map_resp_.map );
      if(sep_tmp[sep_tmp.size()-1] == "map"){
        showMapInfo(map);
      }

    }

  private:
    ros::NodeHandle n;
    SimpleGridMap map;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::Publisher goal_marker_pub;
    ros::Publisher text_array_marker_pub;
    ros::Publisher dock_marker_pub;
    ros::Publisher robothome_marker_pub;
    ros::Publisher forbiddenarea_marker_pub;
    ros::Publisher forbiddenarea_array_marker_pub;
    ros::Publisher forbiddenline_marker_pub;
    ros::ServiceServer service;
    bool deprecated;

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  if(argc != 3 && argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  if (argc != 2) {
    ROS_WARN("Using deprecated map server interface. Please switch to new interface.");
  }
  std::string fname(argv[1]);
  double res = (argc == 2) ? 0.0 : atof(argv[2]);
  /*
  std::list<std::list<std::map<int,string > > > llmp;
  std::map<int,string> tmp_map;
  tmp_map[0]="zero";
  std::list<std::map<int,string > > tmp_lmp;
  tmp_lmp.push_back(tmp_map);
  llmp.push_back(tmp_lmp);
  (*( (llmp.begin())->begin()))[0] ="one";
  std::cout << " cout." <<  (( ( (llmp.begin())->begin()))->begin())->second << std::endl;
  */
  try
  {
    MapServer ms(fname, res);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

