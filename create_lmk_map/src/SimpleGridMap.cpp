#include "create_lmk_map/SimpleGridMap.h"
#include "create_lmk_map/GridMapConstants.h"
#include "create_lmk_map/Geometry.h"
#include <fstream>
#include <string>
#include <iostream>
#include <memory.h>
#include <ros/ros.h>
using namespace std;
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

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

SimpleGridMap::~SimpleGridMap()
{

}

void SimpleGridMap::LoadFromFile(std::string file_name)
{

	std::ifstream ifs;
	ifs.open(file_name.c_str());
	if(!ifs) {
        std::cerr<<"FATAL: map file: "<< file_name << " not exist!!"<<std::endl;
	}
	assert(ifs);
	map_name_ = file_name;

	std::string str;
	bool findData = false;

	while (ifs >> str) {
        std::cout << "cur str: " << str << std::endl;
        if (str == "globID") {
             std::cout << "cur suc to get : " << str  << " info "<< std::endl;
			findData = true;
            break;
		}

	}
   int cnt_break=2;
    while (ifs >> str) {
         std::cout << "cur jumg str: " << str << std::endl;
        if ( cnt_break++ == 9) {
            break;
        }
    }

	if(!findData) {
		std::cerr<<"FATAL: map file parse error!!"<<std::endl;
	}
	assert(findData);

	double x, y;
    std::cout << "ready to get (x,y)"<< std::endl;
    ifs >> str;
	while (ifs >> x) {
		ifs >> y;
         std::cout << "cur  insert (x,y) :( " << x << " , " << y << " ) "<< std::endl;
		InsertPos(x, y);

        int cnt_break=3;
        while (ifs >> str) {
            std::cout << "cur jump insert str: " << str << std::endl;
            if ( cnt_break++ == 7) {
                break;
            }
        }//end while jump


    }//end while (ifs >> x)
    std::cout << "Simple Grid Map .Read map suc!close file "<< std::endl;
	ifs.close();
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

//////////////////////////////////////////////////////////////////////////
char SimpleGridMap::GetStatusFast(const GridPos &grid_pos)
{
	return (grid_data_.get())[grid_pos.second * grid_num_width_ + grid_pos.first];
}

char SimpleGridMap::GetStatusFast(int grid_x, int grid_y)
{
	return (grid_data_.get())[grid_y * grid_num_width_ + grid_x];
}

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


