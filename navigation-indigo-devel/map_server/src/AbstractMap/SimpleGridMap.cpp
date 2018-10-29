#include "map_server/AbstractMap/SimpleGridMap.h"
#include "map_server/AbstractMap/GridMapConstants.h"
#include "map_server/AbstractMap/Geometry.h"
#include <fstream>
#include <string>
#include <iostream>
#include <memory.h>
#include <ros/ros.h>
#include <tf/tf.h>
using namespace std;
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
FreeRegion1::FreeRegion1(std::vector<VecPosition> &vp) {
  plist_ = vp;
  // 对于矩形而言
  num_ = plist_.size();
  assert(4 == num_);
}


bool FreeRegion1::IsInside(VecPosition &pos) {
  /// 先做左后一条射线和第一条射线的叉积
   /* add by chq for -s */
   //avoid the forbidden area is a point(that is mean start point is equal to end point )
   //(if in this situation, the previous source code would return true ,that is a BUG)

  bool same = (plist_[0] == plist_[1]) && (plist_[1] == plist_[2]) && (plist_[2] == plist_[3]) ;
  if( same  && pos == plist_[0] )return true;
  if( same )return false;

    /* add by chq for -s */
  VecPosition v1 = plist_[num_ - 1] - pos;
  VecPosition vnxt = plist_[0] - pos;
  double cross_res = v1.getX() * vnxt.getY() - vnxt.getX() * v1.getY();
  int change_sign = 0;
  for( int i = 1; i < num_; ++i) {
       v1 = vnxt;
       vnxt = plist_[i] - pos;
       double res = v1.getX() * vnxt.getY() - vnxt.getX() * v1.getY();
       if ( res * cross_res < 0.0 ) {
            ++change_sign;
       }
       cross_res = res;
  }
  return (0 == change_sign ? true : false);
}


SimpleGridMap::SimpleGridMap(int grid_size):grid_size_(grid_size)
{
  grid_num_width_ = grid_num_height_ = kInitGridNum;
  grid_data_ = boost::shared_array<char>(new char[grid_num_width_ * grid_num_height_]);
  memset(grid_data_.get(), 0, sizeof(char) * grid_num_width_ * grid_num_height_);
  b_empty_ = true;
  map_name_ = "NoMap";
}

SimpleGridMap::SimpleGridMap(const SimpleGridMap &other)
{
  grid_num_width_ = other.grid_num_width_;
  grid_num_height_ = other.grid_num_height_;

  anchor_grid_x_ = other.anchor_grid_x_;
  anchor_grid_y_ = other.anchor_grid_y_;
  anchor_world_x_ = other.anchor_world_x_;
  anchor_world_y_ = other.anchor_world_y_;

  grid_size_ = other.grid_size_;
  b_empty_ = other.b_empty_;

  grid_data_ = boost::shared_array<char>(new char[grid_num_width_ * grid_num_height_]);
  memcpy(grid_data_.get() , other.grid_data_.get() , sizeof(char) * grid_num_width_ * grid_num_height_);

  map_name_ = other.map_name_;

}
 void SimpleGridMap::operator = (const SimpleGridMap &other){
  grid_num_width_ = other.grid_num_width_;
  grid_num_height_ = other.grid_num_height_;

  anchor_grid_x_ = other.anchor_grid_x_;
  anchor_grid_y_ = other.anchor_grid_y_;
  anchor_world_x_ = other.anchor_world_x_;
  anchor_world_y_ = other.anchor_world_y_;

  grid_size_ = other.grid_size_;
  b_empty_ = other.b_empty_;

  grid_data_ = boost::shared_array<char>(new char[grid_num_width_ * grid_num_height_]);
  memcpy(grid_data_.get() , other.grid_data_.get() , sizeof(char) * grid_num_width_ * grid_num_height_);

  map_name_ = other.map_name_;

 }
SimpleGridMap::~SimpleGridMap()
{

}

void SimpleGridMap::LoadFromFile(std::string file_name,nav_msgs::GetMap::Response* resp,double &res)
{
  std::ifstream ifs(file_name.c_str());
  if (ifs.fail()) {
     ROS_ERROR("Map_server could not open .map file: %s", file_name.c_str());
     exit(-1);
  }
  grid_size_ =res * 1000;//
  std::cout << "Load Simple Gird Map.set grid size by input para: " << grid_size_ <<std::endl;
  std::string str;
  std::vector<std::string> sep_tmp;
  bool findData = false;
  bool findUnKnownData = false;//存储非占用或自由的坐标数据（适配图片地图）
  //first to get resolution
  while(std::getline(ifs, str)){
    SplitString(str,' ',sep_tmp);
    if ( sep_tmp[0] == "Resolution:" ) {
      res = atof( sep_tmp[1].c_str() ) / 1000.0;//unit m to mm
      grid_size_ = res * 1000;///!!!!!!!!!!!!!!!!!!!!!!!!!
      std::cout << "Load Simple Gird Map.changing resolution to file conf(mm): " << grid_size_ <<std::endl;
      break;
   }
  }
  while ( std::getline(ifs, str) ){
          SplitString(str,' ',sep_tmp);
          if( findUnKnownData ) {
           if(str != "DATA" ){//if have not hit DATA
             //std::cout << "LoadSimpleGridMap.unknownpos: (" << sep_tmp[0] << "," << sep_tmp[1] <<std::endl;
             InsertPos( atoi(sep_tmp[0].c_str()),atoi(sep_tmp[1].c_str()),kUnKnown );// data
            }
           else{
             findUnKnownData = false;
             findData = true;
            }
          }else
          if( findData ){
            if(str != "UNKNOWN DATA" ) {//if have not hit UnKnown Data
              //std::cout << "LoadSimpleGridMap.insert pos: (" << sep_tmp[0] << "," << sep_tmp[1] <<std::endl;
              InsertPos( atoi(sep_tmp[0].c_str()),atoi(sep_tmp[1].c_str()),kUnTraverable );//unknown data
             }
            else{
             findData = false;
             findUnKnownData = true;
             }
          }else
          if (str == "UNKNOWN DATA")
          {
            std::cout << "LoadSimpleGridMap. find unknown data!" <<std::endl;
            findUnKnownData = true;
            findData = false;
            continue;
          }else
          if (str == "DATA" )
          {
            std::cout << "LoadSimpleGridMap. find data!" <<std::endl;
            findData = true;
            findUnKnownData = false;
            continue;
          }else
          if ( sep_tmp[0] == "Cairn:" ) {
            // the last 4 element is the rect's corners
            size_t k(0);
            while ( string::npos == sep_tmp[k].find("ICON")) {
                    ++k;
            }
            //  assert(k + 5 == sep_tmp.size() - 1);
            double px[4];
            size_t j = 0;
            for( size_t i = k + 2; i < sep_tmp.size(); ++i) {
                 istringstream s(sep_tmp[i]);
                 s >> px[j];
                 ++j;
               }
            // --------------------now navi do not use forbiddenarea 20170715--------S-------
            if( sep_tmp[1] == "ForbiddenArea" ) {
              // 根据角度把坐标转换回去
              double rect_a = 0.0; // rad
              std::vector<VecPosition> p_list;
              {
                istringstream s(sep_tmp[4]);
                s >> rect_a;
                rect_a = Deg2Rad(VecPosition::normalizeAngle(rect_a));
                // 对角点1
                double xx = px[0];
                double yy = px[1];
                double tx = cos(rect_a) * xx - sin(rect_a) * yy;
                double ty = sin(rect_a) * xx + cos(rect_a) * yy;
                p_list.push_back(VecPosition(tx/1000.0, ty/1000.0));
                // 逆时针点1
                xx = px[0];
                yy = px[3];
                tx = cos(rect_a) * xx - sin(rect_a) * yy;
                ty = sin(rect_a) * xx + cos(rect_a) * yy;
                p_list.push_back(VecPosition(tx/1000.0, ty/1000.0));
                // 对角点2
                xx = px[2];
                yy = px[3];
                tx = cos(rect_a) * xx - sin(rect_a) * yy;
                ty = sin(rect_a) * xx + cos(rect_a) * yy;
                p_list.push_back(VecPosition(tx/1000.0, ty/1000.0));
                // 逆时针点
                xx = px[2];
                yy = px[1];
                tx = cos(rect_a) * xx - sin(rect_a) * yy;
                ty = sin(rect_a) * xx + cos(rect_a) * yy;
                p_list.push_back(VecPosition(tx/1000.0, ty/1000.0));
             }
              //Rect rect(VecPosition(px[0], px[1]), VecPosition(px[2], px[3]));
              FreeRegion1 rect(p_list);
               
              double init_x(0.0), init_y(0.0), init_angle(0.0);
              istringstream sx(sep_tmp[2]);
              istringstream sy(sep_tmp[3]);
              istringstream sa(sep_tmp[4]);
              sx >> init_x;
              sy >> init_y;
              sa >> init_angle;
              OrientPos cen_pos(init_x/1000.0,init_y/1000.0,init_angle);
              OrientPos cor_pos(px[0]/1000.0,px[1]/1000.0,0);
              OrientPos opp_pos(px[2]/1000.0,px[3]/1000.0,0);
              shape_item item(cen_pos,AREA_ITEM,sep_tmp[7],cor_pos,opp_pos,rect);
              forbidden_area_.push_back(item);
             }else
            if( sep_tmp[1] == "ForbiddenLine" ){
                 double init_angle(0.0);
                 istringstream sa(sep_tmp[4]);
                 sa >> init_angle;

                 string name = sep_tmp[7];
                 OrientPos start(px[0]/1000.0, px[1]/1000.0, 0.0, name);
                 OrientPos end(px[2]/1000.0, px[3]/1000.0, 0.0, name);
                 OrientPos cen_pos(px[0]/1000.0,px[1]/1000.0,init_angle);
                 FreeRegion1 rect;//do nothing
                 shape_item item(cen_pos,AREA_ITEM,name,start,end,rect);
                 forbidden_line_.push_back(item);
            }else
            if( sep_tmp[1] == "Goal" ){
              double init_x(0.0), init_y(0.0), init_angle(0.0);

              istringstream sx(sep_tmp[2]);
              istringstream sy(sep_tmp[3]);
              istringstream sa(sep_tmp[4]);
              sx >> init_x;
              sy >> init_y;
              sa >> init_angle;
              string name = sep_tmp[7];
              OrientPos pose_(init_x/1000.0, init_y/1000.0, init_angle);

              ROS_INFO_STREAM( "simlpe grid map."<< "push back goal pos( " << pose_.x() << "," << pose_.y()<< ")" );
              shape_item item(pose_,POINT_ITEM,name);
              goals_.push_back(item);

            }else
            if( sep_tmp[1] == "RobotHome" ){
              double init_x(0.0), init_y(0.0), init_angle(0.0);

              istringstream sx(sep_tmp[2]);
              istringstream sy(sep_tmp[3]);
              istringstream sa(sep_tmp[4]);
              sx >> init_x;
              sy >> init_y;
              sa >> init_angle;
              string name = sep_tmp[7];
              OrientPos pose_(init_x/1000.0, init_y/1000.0, init_angle,name);
               shape_item item(pose_,POINT_ITEM,name);
              robot_homes_.push_back(item);
            }else
            if( sep_tmp[1] == "Dock" ){
              double init_x(0.0), init_y(0.0), init_angle(0.0);

              istringstream sx(sep_tmp[2]);
              istringstream sy(sep_tmp[3]);
              istringstream sa(sep_tmp[4]);
              sx >> init_x;
              sy >> init_y;
              sa >> init_angle;
              string name = sep_tmp[7];
              OrientPos pose_(init_x/1000.0, init_y/1000.0, init_angle,name);
               shape_item item(pose_,POINT_ITEM,name);
              dock_points_.push_back(item);
            }
    }
  }
  ifs.close();

  int grid_size = GetGridSize();
  int grid_width = GetWidthByGrid();
  int grid_height = GetHeightByGrid();
  std::cout << "map grid size: " << grid_size << " gird_width: " << grid_width << " grid_height: " << grid_height << std::endl;

 ///!!!获得左下角世界坐标数据 用于map_server 显示
 ///!!! 栅格坐标与世界坐标方向对应　即00 对应小ｘ小ｙ　左下角　width -1 height -1 对应右上角
  WorldPos pos_leftdown = Grid2World( GridPos(0,0)  );
  WorldPos pos_rightdown = Grid2World( GridPos(grid_width-1,0)  );
  WorldPos pos_leftup =Grid2World( GridPos(0,grid_height-1)  );
  WorldPos pos_rightup = Grid2World(GridPos(grid_width-1,grid_height-1)  );
  WorldPos pos_center = Grid2World( GridPos(grid_width/2,grid_height/2)  );

 printf("left up world pos (mm,mm)(x,y) :(%.4lf,%.4lf).accuracy:[%dmm] \n",pos_leftup.first, pos_leftup.second, grid_size);
 printf("right up world pos (mm,mm)(x,y) :(%.4lf,%.4lf).accuracy:[%dmm] \n",pos_rightup.first, pos_rightup.second, grid_size);
 printf("left down world pos(mm,mm)(x,y) :(%.4lf,%.4lf).accuracy:[%dmm] \n",pos_leftdown.first, pos_leftdown.second, grid_size);
 printf("right down world pos(mm,mm)(x,y) :(%.4lf,%.4lf).accuracy:[%dmm] \n",pos_rightdown.first, pos_rightdown.second, grid_size);
 printf("center grid world pos (mm,mm)(x,y) :(%.4lf,%.4lf).accuracy:[%dmm] \n",pos_center.first, pos_center.second, grid_size);
  std::cout << "LoadSimpleGridMap. close file. gird with,height:("
            <<grid_num_width_ << " " << grid_num_height_ <<")"
            <<"origin(mm) : " <<  pos_leftdown.first << " " << pos_leftdown.second << " "
            <<std::endl;
  // Copy the image data into the map structure
  resp->map.info.width = grid_num_width_;
  resp->map.info.height = grid_num_height_;
  resp->map.info.resolution = res;
  resp->map.info.origin.position.x = ((double)pos_leftdown.first)/1000.0;
  resp->map.info.origin.position.y = ((double)pos_leftdown.second)/1000.0;
  resp->map.info.origin.position.z = 0.0;
  tf::Quaternion q;
  q.setRPY(0,0,0);
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();
  // Allocate space to hold the data
  resp->map.data.resize(resp->map.info.width * resp->map.info.height);
  std::cout << "alloc the data into resp->map.data\n";
  for( int i = 0; i < grid_num_width_ * grid_num_height_; i++  ){
     //printf("alloc i:%d v:%d\n",i, (int)grid_data_[i]);
     resp->map.data[i] = grid_data_[i];
  }

}


void SimpleGridMap::WriteFile(void){
     int grid_width = 200;
     int grid_height = 200;

     grid_width = GetWidthByGrid();
     grid_height = GetHeightByGrid();

   std::cout << "define array "<< " grid_width: " << grid_width<< " grid_height: "  << grid_height<< std::endl;

    //int griddata[grid_width][grid_height];
    std::cout << "before define ofstream out"<<std::endl;
    ofstream out;
    std::cout << "after define ofstream out"<<std::endl;
    std::string grid_name = "./griddata.txt";

    std::cout << "before out.open"<<std::endl;
    out.open(grid_name.c_str(), ios::out|ios::trunc);
    std::cout << "after out.open"<<std::endl;

    if(!out) {
         std::cerr<<" FATAL: write map  file not exist!!"<<std::endl;
    }
     assert(out);
    ROS_INFO("SimpleGridMap grid data :");
     int temp;

    for(int j=0; j < grid_height; j++){
         for ( int i=0; i < grid_width; i++){
               // griddata[i][j] = GetStatusFast(i,j);
               temp = GetStatusFast(i,j);
                out << temp << "\t";
             }
          out << std::endl;
    }
    out.close();
    ROS_INFO("SimpleGridMap Write File End!");
}
///!!!grid pos (1,0) refer  to x = 1 y = 0 in grid cord not index x = 1,y = 0
///!!!so compapred grid a (1.0) to grid b(0,1), a.x > b.x && a.y < b.y
///!!!so the left down pos in the grid cord is that (0,0)
///!!!the top right pos in the grid cord is that (grid_num_width-1,grid_num_height-1)
//////////////////////////////////////////////////////////////////////////
char SimpleGridMap::GetStatusFast(const GridPos &grid_pos)
{
  return (grid_data_.get())[grid_pos.second * grid_num_width_ + grid_pos.first];
}

char SimpleGridMap::GetStatusFast(int grid_x, int grid_y)
{
  return (grid_data_.get())[grid_y * grid_num_width_ + grid_x];
}
//row
void SimpleGridMap::SetStatusFast(int grid_x, int grid_y, char value)
{
  (grid_data_.get())[grid_y * grid_num_width_ + grid_x] = value;
}

GridPos SimpleGridMap::World2Grid(const WorldPos &world_pos)
{
  double xWorld = world_pos.first;
  double yWorld = world_pos.second;
  double xOffsetWorld = xWorld - anchor_world_x_;
  double yOffsetWorld = yWorld - anchor_world_y_;

  if (xOffsetWorld >= 0)
    xOffsetWorld += grid_size_ / 2.0;
  else
    xOffsetWorld -= grid_size_ / 2.0;

  if (yOffsetWorld >= 0)
    yOffsetWorld += grid_size_ / 2.0;
  else
    yOffsetWorld -= grid_size_ / 2.0;

  int xOffsetGrid = int(xOffsetWorld / grid_size_);
  int yOffsetGrid = int(yOffsetWorld / grid_size_);

  return GridPos(anchor_grid_x_ + xOffsetGrid, anchor_grid_y_ + yOffsetGrid);
}

void SimpleGridMap::World2GridAccurate(double xWorld, double yWorld, double &xGrid, double &yGrid)
{
  double xOffsetWorld = xWorld - anchor_world_x_;
  double yOffsetWorld = yWorld - anchor_world_y_;

  double xOffsetGrid = xOffsetWorld / (double) grid_size_;
  double yOffsetGrid = yOffsetWorld / (double) grid_size_;

  xGrid = (double) anchor_grid_x_ + 0.5 + xOffsetGrid;
  yGrid = (double) anchor_grid_y_ + 0.5 + yOffsetGrid;

}

WorldPos SimpleGridMap::Grid2World(const GridPos &grid_pos)
{
  int xGridOffset = grid_pos.first - anchor_grid_x_;
  int yGridOffset = grid_pos.second - anchor_grid_y_;
  double xWorld = anchor_world_x_ + xGridOffset * grid_size_;
  double yWorld = anchor_world_y_ + yGridOffset * grid_size_;
  return WorldPos(xWorld, yWorld);
}

void SimpleGridMap::InsertPos(double xWorld, double yWorld, char status)
{
  if(b_empty_) {
    InitWithPos(xWorld, yWorld);
  }
  //
  GridPos gridCoord = World2Grid(WorldPos(xWorld, yWorld));
  //
  //std::cout << "simplegridmap.insert pos check extend ...\n";
  int extentCode = CheckExtent(gridCoord);
  while (extentCode > 0) {
    Extent(extentCode);
    gridCoord = World2Grid(WorldPos(xWorld, yWorld));
    extentCode = CheckExtent(gridCoord);
  }

  int memOffset = gridCoord.second * grid_num_width_ + gridCoord.first;
  grid_data_[memOffset] = status;
}

void SimpleGridMap::InitWithPos(double xWorld, double yWorld)
{
  if (!b_empty_)
    return;

  b_empty_ = false;
   //第一个数据作为世界坐标系下的原点
  anchor_world_x_ = xWorld;
  anchor_world_y_ = yWorld;
    //anchor_grid_x_ = grid_num_width_ / 2;
    //anchor_grid_y_ = grid_num_height_ / 2;
     //原点始终在中心（网格中心的世界坐标始终是文件地图数据里第一个点）
    anchor_grid_x_ = grid_num_width_ / 2;
    anchor_grid_y_ = grid_num_height_ / 2;
}
//检测栅格有没有越界
int SimpleGridMap::CheckExtent(GridPos gridCoord)
{
  //chq comment
  //网格坐标X不允许小于0，不允许超过grid_num_width
  //网格坐标y不允许小于0，不允许超过grid_num_height
  int left = gridCoord.first < 0 ? kDirectionLeft : 0;
  int right = gridCoord.first >= grid_num_width_ ? kDirectionRight : 0;
  int down = gridCoord.second < 0 ? kDirectionDown : 0;
  int up = gridCoord.second >= grid_num_height_ ? kDirectionUp : 0;
  return (left | right | down | up);
}

void SimpleGridMap::LeftExtent()
{
  anchor_grid_x_ += grid_num_width_;
  int oldWidth = grid_num_width_;
  grid_num_width_ *= 2;
   //分配新的内存给新数组
  boost::shared_array<char> new_data(new char[grid_num_width_ * grid_num_height_]);
  //初始化新数组
  memset(new_data.get(), 0, sizeof(char) * grid_num_width_ * grid_num_height_);
  /// previous
  //AX---------------
  //CX---------------

  //  now 现在为两倍oldWidth宽度
  //[][][][][][][][][]AX----------------
  //[][][][][][][][][]CX----------------

  char *desMem = new_data.get() + oldWidth;
  char *sourceMem = grid_data_.get();

  for (int i = 0; i < grid_num_height_; i++) {
    /* 将老数据归位到原有的位置 */
    memcpy(desMem, sourceMem, sizeof(char) * oldWidth);
    //新数据数组跳转一行
    desMem += grid_num_width_;
    //老数据数组也跳转一行
    sourceMem += oldWidth;
  }
  //for share_ptr 
  //reset() 相当于销毁当前所控制的对象
  //reset(T* p) 相当于销毁当前所控制的对象，然后接管p所指的对象
  //reset(T*, Deleter) 和上面一样
  grid_data_.reset();
  grid_data_ = new_data;
}

void SimpleGridMap::RightExtent()
{
  int oldWidth = grid_num_width_;
  grid_num_width_ *= 2;

  boost::shared_array<char> new_data(new char[grid_num_width_ * grid_num_height_]);
  memset(new_data.get(), 0, sizeof(char) * grid_num_width_ * grid_num_height_);

  char *desMem = new_data.get();
  char *sourceMem = grid_data_.get();

  for (int i = 0; i < grid_num_height_; i++) {
    memcpy(desMem, sourceMem, sizeof(char) * oldWidth);
    desMem += grid_num_width_;
    sourceMem += oldWidth;
  }

  grid_data_.reset();
  grid_data_ = new_data;
}

void SimpleGridMap::UpExtent()
{
  int oldHeight = grid_num_height_;
  grid_num_height_ *= 2;

  boost::shared_array<char> new_data(new char[grid_num_width_ * grid_num_height_]);
  memset(new_data.get(), 0, sizeof(char) * grid_num_width_ * grid_num_height_);

  memcpy(new_data.get(), grid_data_.get(), sizeof(char) * grid_num_width_ * oldHeight);

  grid_data_.reset();
  grid_data_ = new_data;
}

void SimpleGridMap::DownExtent()
{
  anchor_grid_y_ += grid_num_height_;
  int oldHeight = grid_num_height_;
  grid_num_height_ *= 2;

  boost::shared_array<char> new_data(new char[grid_num_width_ * grid_num_height_]);
  memset(new_data.get(), 0, sizeof(char) * grid_num_width_ * grid_num_height_);

  memcpy(new_data.get() + grid_num_width_ * oldHeight, grid_data_.get(), sizeof(char) * grid_num_width_ * oldHeight);

  grid_data_.reset();
  grid_data_ = new_data;
}

void SimpleGridMap::Extent(int extentCode)
{
  if ((extentCode & kDirectionLeft) > 0) {
    LeftExtent();
    return;
  }
  //
  if ((extentCode & kDirectionRight) > 0) {
    RightExtent();
    return;
  }
  //
  if ((extentCode & kDirectionUp) > 0) {
    UpExtent();
    return;
  }
  //是
  if ((extentCode & kDirectionDown) > 0) {
    DownExtent();
    return;
  }
}


void SimpleGridMap::AddBoundary()
{
  for(int i=0 ; i< grid_num_width_ ; i++)
  {
    SetStatusFast(i,0,kUnTraverable);
    SetStatusFast(i,grid_num_height_-1,kUnTraverable);
  }
  //
  for(int i=0 ; i< grid_num_height_ ; i++)
  {
    SetStatusFast(0,i,kUnTraverable);
    SetStatusFast(grid_num_width_-1,i,kUnTraverable);
  }
}

void SimpleGridMap::Erode(int size)
{
  std::vector<IntPoint> nowWave;
  std::vector<IntPoint> nextWave;
  //找到所有障碍物点 存储
  for (int y = 0; y < grid_num_height_; y++)
  {
    for (int x = 0; x < grid_num_width_; x++)
    {
      if( GetStatusFast(x,y) == kUnTraverable) {
        nextWave.push_back( IntPoint(x,y) );
      }
    }
  }
  //////////////////////////////////////////////////////////////////////////
  for( int repeat = 0 ; repeat < size ; repeat++ )
  {

    nowWave = nextWave;
    nextWave.clear();
    //在最新推移的点内遍历
    for( int i=0 ; i<nowWave.size() ; i++)
    {
      IntPoint nowPoint = nowWave[i];
      int xNext =  nowPoint.x;
      int yNext =  nowPoint.y;
      /*判断右方栅格是否超出边界，且判断是否为障碍物栅格；条件成立则将其设置为不可行区域，并将其放入列表中*/
      if( CheckGridValid(xNext+1 , yNext) && (GetStatusFast(xNext+1, yNext) != kUnTraverable) )
      {
        SetStatusFast(xNext+1 , yNext , kUnTraverable );
        /// 推移的点计入新的nextWare         ///
        nextWave.push_back( IntPoint(xNext+1 , yNext) );
      }
      /*判断左方栅格是否超出边界，且判断是否为障碍物栅格*/
      if( CheckGridValid(xNext-1 , yNext) && (GetStatusFast(xNext-1, yNext) != kUnTraverable) )
      {
        SetStatusFast(xNext-1 , yNext , kUnTraverable );
        nextWave.push_back( IntPoint(xNext-1 , yNext) );
      }
      /*判断上方栅格是否超出边界，且判断是否为障碍物栅格*/
      if( CheckGridValid(xNext, yNext+1) && (GetStatusFast(xNext, yNext+1) != kUnTraverable) )
      {
        SetStatusFast(xNext , yNext+1 , kUnTraverable );
        nextWave.push_back( IntPoint(xNext , yNext+1) );
      }
      /*判断下方栅格是否超出边界，且判断是否为障碍物栅格*/
      if( CheckGridValid(xNext , yNext-1) && (GetStatusFast(xNext, yNext-1) != kUnTraverable) )
      {
        SetStatusFast(xNext , yNext-1 , kUnTraverable );
        nextWave.push_back( IntPoint(xNext , yNext-1) );
      }
      /*判断右上方栅格是否超出边界，且判断是否为障碍物栅格*/
      if( CheckGridValid(xNext+1 , yNext+1) && (GetStatusFast(xNext+1, yNext+1) != kUnTraverable) )
      {
        SetStatusFast(xNext+1 , yNext+1 , kUnTraverable );
        nextWave.push_back( IntPoint(xNext+1 , yNext+1) );
      }
      /*判断右下方栅格是否超出边界，且判断是否为障碍物栅格*/
      if( CheckGridValid(xNext+1 , yNext-1) && (GetStatusFast(xNext+1, yNext-1) != kUnTraverable) )
      {
        SetStatusFast(xNext+1 , yNext-1 , kUnTraverable );
        nextWave.push_back( IntPoint(xNext+1 , yNext-1) );
      }
      /*判断左上方栅格是否超出边界，且判断是否为障碍物栅格*/
      if( CheckGridValid(xNext-1 , yNext+1) && (GetStatusFast(xNext-1, yNext+1) != kUnTraverable) )
      {
        SetStatusFast(xNext-1 , yNext+1 , kUnTraverable );
        nextWave.push_back( IntPoint(xNext-1 , yNext+1) );
      }
      /*判断左下方栅格是否超出边界，且判断是否为障碍物栅格*/
      if( CheckGridValid(xNext-1 , yNext-1) && (GetStatusFast(xNext-1, yNext-1) != kUnTraverable) )
      {
        SetStatusFast(xNext-1 , yNext-1 , kUnTraverable );
        nextWave.push_back( IntPoint(xNext-1 , yNext-1) );
      }

    }
  }
}


