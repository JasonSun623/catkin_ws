#include <ros/ros.h>
#include "loadgridmap/AbstractMap/SimpleGridMap.h"
//typedef std::pair<int, int> GridPos;
//typedef std::pair<double, double> WorldPos;
int main(int argc, char ** argv){
    ros::init(argc, argv, "create_lmk_map");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");
    ros::Rate r(10);
    std::string map_name;//lmk地图
    int map_grid_size ;//栅格地图尺寸mm
    int erode_rf_size;//膨胀尺寸
    nh_ns.param<std::string>("map_name",map_name,"/home/hdros/catkin_ws/src/loadgridmap/map1.map");
    nh_ns.param("map_grid_size",map_grid_size,50);
    nh_ns.param("erode_rf_size",erode_rf_size,0);

    SimpleGridMap map(map_grid_size);
    std::cout << "main.load file ...\n";
    map.LoadFromFile(map_name);
    std::cout << "main.erode map ...\n";
    map.Erode(erode_rf_size);
    std::cout << "main. write map data... ...\n";
    map.WriteFile();
    //
    int grid_size = map.GetGridSize();
    int grid_width = map.GetWidthByGrid();
    int grid_height = map.GetHeightByGrid();
    std::cout << "map grid size: " << grid_size << " gird_width: " << grid_width << " grid_height: " << grid_height << std::endl;
    ///!!! 获得左下角世界坐标数据 用于map_server 显示
    ///!!! 栅格坐标与世界坐标方向对应　即00 对应小ｘ,小ｙ　->左下角　,width -1,height -1 ->右上角
    WorldPos pos_leftdown = Grid2World( GridPos(0,0)  );
    WorldPos pos_rightdown = Grid2World( GridPos(grid_width-1,0)  );
    WorldPos pos_leftup =Grid2World( GridPos(0,grid_height-1)  );
    WorldPos pos_rightup = Grid2World( GridPos(grid_width-1,grid_height-1)  );
    WorldPos pos_center = Grid2World( GridPos(grid_width/2,grid_height/2)  );

    printf("left up world pos (mm,mm)(x,y) :(%.4lf,%.4lf)\.accuracy:[%dmm] \n",pos_leftup.first, pos_leftup.second, grid_size);
    printf("right up world pos (mm,mm)(x,y) :(%.4lf,%.4lf)\.accuracy:[%dmm] \n",pos_rightup.first, pos_rightup.second, grid_size);
    printf("left down world pos(mm,mm)(x,y) :(%.4lf,%.4lf)\.accuracy:[%dmm] \n",pos_leftdown.first, pos_leftdown.second, grid_size);
    printf("right down world pos(mm,mm)(x,y) :(%.4lf,%.4lf)\.accuracy:[%dmm] \n",pos_rightdown.first, pos_rightdown.second, grid_size);
    printf("center grid world pos (mm,mm)(x,y) :(%.4lf,%.4lf)\.accuracy:[%dmm] \n",pos_center.first, pos_center.second, grid_size);
    /*while(nh.ok()){
        r.sleep();
        ros::spinOnce();
    }
    */
    return 0;
}
