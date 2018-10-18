#ifndef _SURO_STD_SIMPLE_GRID_MAP_taotao_20110901_H
#define _SURO_STD_SIMPLE_GRID_MAP_taotao_20110901_H


#include <utility>
#include <vector>
#include "AbstractMap.h"
#include "Geometry.h"
#include "map_server/image_loader.h"

typedef std::pair<int, int> GridPos;
typedef std::pair<double, double> WorldPos;

class FreeRegion1 {
public:
  FreeRegion1(){}
  FreeRegion1(std::vector<VecPosition> &vp);
  ~FreeRegion1() {}
  bool IsInside(VecPosition &pos);
  std::vector<VecPosition> list(){return plist_;}
private:
  // void MakeRect();
private:
    // 四个角点按逆时针顺序
  std::vector<VecPosition> plist_;
  int num_;
};
class OrientPos{
public:
  OrientPos(){_x=0.0;_y=0.0;_angle_deg=0.0;}
  OrientPos(double dx,double dy,double a = 0.0,std::string name = ""):_x(dx),_y(dy),_angle_deg(a),_icon(name){
  }
  double x(){return _x;}
  double y(){return _y;}
  double angle(){return _angle_deg;}
  std::string name(){return _icon;}
private:
  double _x;
  double _y;
  double _angle_deg;
  std::string _icon;
};

enum item_type{POINT_ITEM,AREA_ITEM};

class shape_item{
  public:
  shape_item(){
    ;
  }
  shape_item(OrientPos cen_pos,item_type tp, std::string nm=""):
  center_pos(cen_pos),_type(tp), icon(nm)
  {
    
  }
  shape_item(OrientPos cen_pos, item_type tp, std::string nm, OrientPos cor_pos, OrientPos opp_pos, FreeRegion1 reg):
  center_pos(cen_pos),icon(nm),_type(tp),corner_pos(cor_pos),oppsite_pos(opp_pos),_reg(reg)
  {
    
  }
  OrientPos pos(){return center_pos;}
  void getPairPos(OrientPos & cor,OrientPos &opp){
     cor = corner_pos;
     opp = oppsite_pos;
    }
  std::string name(){return icon;}
  item_type type(){return _type;}
  FreeRegion1 regin(){return _reg;}
  private:
  OrientPos center_pos;
  OrientPos corner_pos;
  OrientPos oppsite_pos;
  std::string icon;
  item_type _type; 
  FreeRegion1 _reg;

};
typedef std::pair<OrientPos,OrientPos> LineNode;
//////////////////////////////////////////////////////////////////////////
//栅格地图：每个栅格存储一个char代表该栅格是否是障碍
//重要函数：
//GetWidthByGrid() 获得栅格总列数
//GetHeightByGrid() 获得栅格总行数

//GetStatusFast(int grid_x, int grid_y)以及GetStatusFast(const GridPos &grid_pos)函数可用于获取已知栅格坐标的栅格状态

//通过以上三个函数可以实现栅格遍历
//栅格坐标和世界坐标的转换函数：World2Grid和Grid2World
//获取和世界坐标点(x,y)距离最近的障碍点的世界坐标的函数：void GetNearestObstacle(double x , double y , double &x_obs , double &y_obs)

class SimpleGridMap : public AbstractMap
{
public:
  static const int kInitGridNum = 100;
  //
  static const char kTraverable = 0;
    //static const char kUnTraverable = 1;
  static const char kUnTraverable = 100;//chq
  static const char kUnKnown = -1;//chq
    //
  static const int kDirectionLeft = 1;
  static const int kDirectionRight = 2;
  static const int kDirectionUp = 4;
  static const int kDirectionDown = 8;

public:
  SimpleGridMap(int grid_size = 100);
  SimpleGridMap(const SimpleGridMap &other);
   void operator = (const SimpleGridMap &other);
  ~SimpleGridMap();
  void LoadFromFile(std::string file_name,nav_msgs::GetMap::Response* resp,double &res);
  void WriteFile();
  std::string GetMapTypeName() {
    return "SimpleGridMap";
  }
  inline int  SplitString(const string &input,const char delimiter,std::vector<string> &results) {
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

public:
  inline int GetWidthByGrid() {
    return grid_num_width_;
  }

  inline int GetHeightByGrid() {
    return grid_num_height_;
  }

  inline int GetGridSize() {
    return grid_size_;
  }

  char GetStatusFast(const GridPos &grid_pos);
  char GetStatusFast(int grid_x, int grid_y);
  void SetStatusFast(int grid_x, int grid_y, char value);

  inline bool CheckGridValid(const int &xGrid, const int &yGrid) {
    return xGrid >= 0 && xGrid < grid_num_width_ && yGrid >= 0 && yGrid < grid_num_height_;
  }

  GridPos World2Grid(const WorldPos &world_pos);
  void World2GridAccurate(double xWorld, double yWorld, double &xGrid, double &yGrid);
  WorldPos Grid2World(const GridPos &grid_pos);


public:
  void AddBoundary();

  void Erode(int size);

protected:
  void InsertPos(double xWorld, double yWorld, char status = kUnTraverable);
  void InitWithPos(double xWorld, double yWorld);
  int CheckExtent(GridPos gridCoord);
  void LeftExtent();
  void RightExtent();
  void UpExtent();
  void DownExtent();
  void Extent(int extentCode);



public://modify by chq for use astarfornavigation 2016 1213
  int grid_size_;  //网格大小
  int grid_num_width_; //横向网格数
  int grid_num_height_; //纵向网格数

  //锚点定义：网格坐标与世界坐标的转换关系不仅需要网格大小，还需要锚点，即需要标明世界坐标原点与网格坐标原点之间的关系。
  //这个关系用一个四元组表示（xGrid , yGrid , xWorld , yWorld）表示在网格坐标为（xGrid，yGrid）时，世界坐标为（xWorld , yWorld）
  //只要指定且仅要指定一个这样的四元组，就可确定网格与世界坐标的映射关系（当然仍需知道网格大小）
  //以下四个参数定义了一个这样的四元组
  int anchor_grid_x_;
  int anchor_grid_y_;
  double anchor_world_x_;
  double anchor_world_y_;
  std::vector<shape_item> forbidden_area_;
  std::vector<shape_item> forbidden_line_;
  std::vector<shape_item> robot_homes_;
  std::vector<shape_item> goals_;
  std::vector<shape_item> dock_points_;
  boost::shared_array<char> grid_data_; //网格数据
  bool b_empty_; //初始时地图为空，加入第一个点以后即不为空

public:

};

#endif
