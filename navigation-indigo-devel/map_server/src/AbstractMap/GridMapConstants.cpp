#include "map_server/AbstractMap/GridMapConstants.h"

double GridMapConstants::UNKNOWN_DOUBLE_VALUE = 1e100;

const char GridMapConstants::COLOR_BLUE[3] = {0,0,255};

const char GridMapConstants::COLOR_WRITE[3] = {255,255,255};

const char GridMapConstants::COLOR_BLACK[3] = {0,0,0};

const char GridMapConstants::COLOR_RED[3] = {255,0,0};

const char GridMapConstants::COLOR_UNKNOW_AREA[3] = {255,0,0};

bool GridMapConstants::IsUnknownDoubleValue(double v)
{
	return v > 1e99;
}

bool GridMapConstants::IsTraverable( char v )
{	
	return v >= MinTraverableArea && v <= MaxTraverableArea;
}

bool GridMapConstants::IsUnTraverable( char v )
{
	return v >= MinUnTraverableArea && v <= MaxUnTraverableArea;
}
