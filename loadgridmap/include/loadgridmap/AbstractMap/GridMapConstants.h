#ifndef _SURO_GRID_MAP_CONSTANTS_taotao_2011_02_12_
#define _SURO_GRID_MAP_CONSTANTS_taotao_2011_02_12_

class GridMapConstants
{
public:
	static const int DIRECTION_LEFT = 1;
	static const int DIRECTION_RIGHT = 2;
	static const int DIRECTION_UP = 4;
	static const int DIRECTION_DOWN = 8;

	static const char COLOR_BLUE[3]; //= {0,0,255};
	static const char COLOR_WRITE[3];// = {255,255,255};
	static const char COLOR_BLACK[3]; //= {0,0,0};
	static const char COLOR_RED[3]; //= {255,0,0};

	static bool isColor(const char* data,char r, char g, char b){
		if(r == data[0] && g == data[1] && b == data[2])
			return true;
		return false;
	}

	static const char COLOR_UNKNOW_AREA[3]; //= {255,0,0};

	static const char MaxUnTraverableArea = 100;

	static const char MinUnTraverableArea = 50;

	static const char MaxTraverableArea = 49;

	static const char MinTraverableArea = 1;

	static const char UnExploreredArea = 0;

	static double UNKNOWN_DOUBLE_VALUE;
	static bool IsUnknownDoubleValue(double v);
	static bool IsTraverable( char v );
	static bool IsUnTraverable( char v );

	static const char everyWallAdd = 8;

	static const char everyExploredAdd = -2;
};




#endif
