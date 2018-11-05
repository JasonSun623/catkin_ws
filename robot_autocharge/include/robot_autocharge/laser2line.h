#pragma once
#include <iostream>
#include <vector>
#include <boost/thread/mutex.hpp>
#include "point.h"

#define DIST_THRESHOLD		0.03
#define LINE_DENSITY		70
#define LASER_NUM			540
///#define MIN_POINT_NUM		3 chq disable
///#define MIN_LINE_DIST		0.03 chq disable

#define MIN_POINT_NUM		10
#define MIN_LINE_DIST		0.3


#define MAX_LINE2LINE_DIST	0.07
#define MAXK				572957795.13082320818620513080389	// tan(89.9999999)

struct segments
{
	int start;
	int end;
	double lbegin_x;
	double lbegin_y;
	double lend_x;
	double lend_y;
	double line_k;
	double line_b;
	double line_length;
};

struct edge
{
	double edge_small;
	double edge_large;
};

struct houghvector
{
	int row[1];
};

class Laser2Line
{
public:
	Laser2Line(void);
	~Laser2Line(void);

	PointList Laser_Data;	// laser data
	std::vector<segments>	laser_line;


	// set laser data
  void setLaserData(const PointList& or_LaserData);
	std::vector<segments> get_line();

	double point_point_dist(int s, int e);
	double point_line_dist(int t, int f, int l);
	double point2line_dist(Point p, Point p1, Point p2);
	void split(int first_index, int last_index);
	void merge(void);
	void hough(void);


	void line_fitting(int index, int lb, int le);
	void split_and_merge();
    double euclidianDist(Point p1,Point p2);//add by chq 


private:

	int dex;

	houghvector laser_hough[180];

};

