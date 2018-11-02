#include "../include/robot_autocharge/laser2line.h"
#include <ros/ros.h>
Laser2Line::Laser2Line(void)
{
}

Laser2Line::~Laser2Line(void)
{
}


// 获取原始激光数据
void Laser2Line::setLaserData(const PointList or_LaserData)
{
  dex = 0;
  Laser_Data.clear();
  Laser_Data = or_LaserData;
  laser_line.clear();
}

// 点到点的距离
double Laser2Line::point_point_dist(int s, int e)
{
  double dx = (Laser_Data[e].x-Laser_Data[s].x);
  double dy = (Laser_Data[e].y-Laser_Data[s].y);
  return sqrt(dx*dx+dy*dy);
}

// 点到线的距离
double Laser2Line::point_line_dist(int t, int f, int l)
{
  return point2line_dist(Laser_Data[t],Laser_Data[f],Laser_Data[l]);
  //double sdist = point_point_dist(f,l);
  //if(sdist==0)
  //  return point_point_dist(t,f);
  //else
  //{
  //  double thisu = ((Laser_Data[t].y-Laser_Data[f].y)*(Laser_Data[l].y-Laser_Data[f].y)+(Laser_Data[t].x-Laser_Data[f].x)*(Laser_Data[l].x-Laser_Data[f].x))/(sdist*sdist);
  //  OrientedPoint plumb_point;
  //  plumb_point.x = Laser_Data[f].x + thisu*(Laser_Data[l].x-Laser_Data[f].x);
  //  plumb_point.y = Laser_Data[f].y + thisu*(Laser_Data[l].y-Laser_Data[f].y);
  //  return sqrt((Laser_Data[t].x-plumb_point.x)*(Laser_Data[t].x-plumb_point.x) + (Laser_Data[t].y-plumb_point.y)*(Laser_Data[t].y-plumb_point.y));
  //}
}
//added by chq 
//点到点距离
double Laser2Line::euclidianDist(Point p1, Point p2){
 double dx =p1.x-p2.x; 
 double dy =p1.y-p2.y; 
 return sqrt( dx * dx + dy * dy );

}
// 点到线的距离
double Laser2Line::point2line_dist(Point p, Point p1, Point p2)
{
  double d = euclidianDist(p1,p2);
  if(d==0)
    return euclidianDist(p,p2);
  else
  {
    double u = ((p.y-p1.y)*(p2.y-p1.y)+(p.x-p1.x)*(p2.x-p1.x))/(d*d);
    OrientedPoint plumb_point;
    plumb_point.x = p1.x + u*(p2.x-p1.x);
    plumb_point.y = p1.y + u*(p2.y-p1.y);
    return sqrt( (p.x-plumb_point.x)*(p.x-plumb_point.x) + (p.y-plumb_point.y)*(p.y-plumb_point.y) );
  }
}

// 直线拟合
void Laser2Line::line_fitting(int index, int lb, int le)
{
  int num_of_points = le - lb + 1;

  double x_av = 0.0, y_av = 0.0;
  for(int i=lb; i<=le; i++)
  {
    x_av += Laser_Data[i].x;
    y_av += Laser_Data[i].y;
  }
  x_av = x_av/(double)num_of_points;
  y_av = y_av/(double)num_of_points;

  double numerator = 0.0, denominator = 0.0;
  double min_x = 99999, max_x = -99999, min_y = 99999, max_y = -99999;
  for(int i=lb; i<=le; i++)
  {
    double dx = (x_av - Laser_Data[i].x);
    double dy = (y_av - Laser_Data[i].y);
    numerator   += dy * dx;
    denominator += ( dy*dy - dx*dx );
    if( min_x > Laser_Data[i].x ){ min_x = Laser_Data[i].x; }
    if( max_x < Laser_Data[i].x ){ max_x = Laser_Data[i].x; }
    if( min_y > Laser_Data[i].y ){ min_y = Laser_Data[i].y; }
    if( max_y < Laser_Data[i].y ){ max_y = Laser_Data[i].y; }
  }
  numerator *= -2.0;

  double angle    = atan2( numerator, denominator ) / 2.0;
  double distance = ( x_av*cos(angle) ) + ( y_av*sin(angle) );
  if(angle<M_PI/4.0 && angle>-M_PI/4.0)
  {
    laser_line[index].lbegin_x = ( distance - (min_y)*sin(angle) ) / cos(angle);
    laser_line[index].lend_x = ( distance - (max_y)*sin(angle) ) / cos(angle);
    laser_line[index].lbegin_y = min_y;
    laser_line[index].lend_y = max_y;
  }
  else{
    laser_line[index].lbegin_y = ( distance - (min_x)*cos(angle) ) / sin(angle);
    laser_line[index].lend_y = ( distance - (max_x)*cos(angle) ) / sin(angle);
    laser_line[index].lbegin_x = min_x;
    laser_line[index].lend_x = max_x;
  }

  if(laser_line[index].lend_x-laser_line[index].lbegin_x == 0)
      laser_line[index].line_k = MAXK;
  else
    laser_line[index].line_k = (laser_line[index].lend_y-laser_line[index].lbegin_y)/(laser_line[index].lend_x-laser_line[index].lbegin_x);

  laser_line[index].line_b = laser_line[index].lend_y - laser_line[index].line_k*laser_line[index].lend_x;
  Point bg(laser_line[index].lbegin_x,laser_line[index].lbegin_y);
  Point ed(laser_line[index].lend_x,laser_line[index].lend_y);
  laser_line[index].line_length = euclidianDist(bg,ed);
}

// 拆分
void Laser2Line::split(int first_index, int last_index)
{
  if(first_index>=last_index)
    return;
  else
  {
    segments seg;
    seg.start = first_index;
    seg.end = last_index;
    if (last_index - first_index < 5)//如果点的数量小于５个，直接首尾连线作为线段 chq
    {
      ROS_INFO_STREAM("Laser2Line::split.first index: " << first_index << " last_index: " << last_index << " low  to 5.directly get one line");
      seg.lbegin_x = Laser_Data[seg.start].x;
      seg.lbegin_y = Laser_Data[seg.start].y;

      seg.lend_x = Laser_Data[seg.end].x;
      seg.lend_y = Laser_Data[seg.end].y;

      if (seg.lend_x - seg.lbegin_x == 0)
        seg.line_k = MAXK;
      else
        seg.line_k = (seg.lend_y - seg.lbegin_y) / (seg.lend_x - seg.lbegin_x);
      seg.line_b = seg.lend_y - seg.line_k*seg.lend_x;
      seg.line_length = point_point_dist(seg.start, seg.end);
      laser_line.push_back(seg);
    }
    //  return;
    else
    {
      double dis;
      double max_dist = 0;
      // check the distance of neighboured points
      for(int i=first_index; i<last_index-1; i++)
      {
        dis = point_point_dist(i, i+1);
        max_dist = dis>max_dist?dis:max_dist;
        if( dis > MAX_LINE2LINE_DIST )//如果点点距离超过距离阈值，认为是两条线段分离点，从此点分离 chq
        {
          ROS_INFO_STREAM("Laser2Line::split.cur index: " << i
                          << " next  index: " << i+1
                          << " point dis: " << dis
                          << " high than MAX_LINE2LINE_DIST" << MAX_LINE2LINE_DIST
                          <<".iter split from here");
          split(first_index,  i);
          split(i+1, last_index);
          return;
        }
      }

      max_dist = 0;
      int max_index = first_index+1;
      for(int i=first_index; i<last_index; ++i)//找到拐点 chq
      {
        double this_dist = point_line_dist(i,first_index,last_index);//dist bet point i and  line <first_index last_index>
        if(this_dist>max_dist)
        {
          max_dist = this_dist;
          max_index = i;
        }
      }
      if (max_dist > DIST_THRESHOLD)//大于阈值,从拐点分离
    //  if (max_dist > point_point_dist(first_index, last_index)/0.12 * DIST_THRESHOLD)
      {
        ROS_INFO_STREAM("Laser2Line::split.the max dist do line first to end : " << max_dist
                        << "bigger than DIST_THRESHOLD: "<< DIST_THRESHOLD
                        <<".iter split from here index: " << max_index );
        split(first_index,max_index);
        split(max_index,last_index);
      }
      else//小于阈值，首尾作为一条线段
      {
        ROS_INFO_STREAM("Laser2Line::split.the max dist do line first to end : " << max_dist
                        << "little than DIST_THRESHOLD: "<< DIST_THRESHOLD
                        <<" push this line directly!" );
        dex ++;
        seg.lbegin_x = Laser_Data[seg.start].x;
        seg.lbegin_y = Laser_Data[seg.start].y;
        seg.lend_x = Laser_Data[seg.end].x;
        seg.lend_y = Laser_Data[seg.end].y;
        if (seg.lend_x - seg.lbegin_x == 0)
          seg.line_k = MAXK;
        else
          seg.line_k = (seg.lend_y - seg.lbegin_y) / (seg.lend_x - seg.lbegin_x);
        seg.line_b = seg.lend_y - seg.line_k*seg.lend_x;
        seg.line_length = point_point_dist(seg.start, seg.end);
        laser_line.push_back(seg);
      //  printf("%d\n",dex);
      }
    }
  }
}


// 线段合并
void Laser2Line::merge(void)
{
//  printf("line: %d before merge\n", laser_line.size());
  for(int i=0; i<laser_line.size(); ++i)
  {
    bool erase_flag = false;
    // check number of points
    if (laser_line[i].end - laser_line[i].start + 1 < MIN_POINT_NUM)//chq若当前线段中的点小于个数阈值
    {
      ROS_INFO("Laser2Line::merge.this line : %d,point number too little in this line merge this line to next.num:%d\n",i, laser_line[i].end - laser_line[i].start + 1);
      erase_flag = true;//chq enable
    }

    // check the length of the line segment
    if (point_point_dist(laser_line[i].start, laser_line[i].end) < MIN_LINE_DIST)//chq若线段长度小于长度阈值
    {
      ROS_INFO("Laser2Line::merge.this line:%d, length too short %lf\n",i,point_point_dist(laser_line[i].start, laser_line[i].end));
      erase_flag = true;//chq enable
    }

    // check the line density
    if ((laser_line[i].end - laser_line[i].start) *1.0 / laser_line[i].line_length < LINE_DENSITY)//chq若个数/长度比小于线条密度阈值
    {
      //printf("line density too little %lf\n", (laser_line[i].end - laser_line[i].start) / laser_line[i].line_length);
      //erase_flag = true;//chq disabled
    }

    if(erase_flag)
    {

      laser_line.erase(laser_line.begin()+i);
      i --;
    }
    else
    {
      //如果当前线段末尾与下条线段开始距离小于最大线线距离 chq
      if( i+1<laser_line.size() && point_point_dist(laser_line[i].end,laser_line[i+1].start)<MAX_LINE2LINE_DIST )
      {
        double distmax1=0, distmax2=0, distmax=0;
        for(int j=laser_line[i].start; j<=laser_line[i].end; ++j)
        {
          double dis1 = point_line_dist(j,laser_line[i].start,laser_line[i].end);
          double dis  = point_line_dist(j,laser_line[i].start,laser_line[i+1].end);
          if(dis1 > distmax1)    distmax1 = dis1;//chq 第i条线段中的所有点到该线段垂距的最大值
          if(dis > distmax)      distmax = dis;//chq 第ｉ条线段中的所有点到i线段起始点和i+1线段终点构成的线段的最大值
        }
        for(int k=laser_line[i+1].start; k<=laser_line[i+1].end; ++k)
        {
          double dis2 = point_line_dist(k,laser_line[i+1].start,laser_line[i+1].end);
          double dis  = point_line_dist(k,laser_line[i].start,laser_line[i+1].end);//chq 第ｉ+1条线段中的点到i线段起始点和i+1线段终点构成的线段的dist
          if(dis2>distmax2)    distmax2 = dis2;//chq 第i+1条线段中的所有点到该线段垂距的最大值
          if(dis>distmax)      distmax = dis;
        }
       //distmax : 第ｉ条线段和第i+1条线段中的所有点到第i条线段起始点和第i+1条线段的终点构成线段的距离最大值
        double MNE1 = distmax1/point_point_dist(laser_line[i].start,laser_line[i].end);//第i条线段中的点到该线段最大垂距　与　该线段长度的比值 chq 简称为线段i垂距比
        double MNE2 = distmax2/point_point_dist(laser_line[i+1].start,laser_line[i+1].end);//第i+1条线段中的点到该线段最大垂距　与　该线段长度的比值 简称为线段i+1垂距比
        double MNE  = distmax/point_point_dist(laser_line[i].start,laser_line[i+1].end);//第i和ｉ＋１条线段中的点到第i线段起点与ｉ＋１线段终点构成线段的最大垂距　与　构成长度的比值 chq 简称为线段i i+1垂距比
        //if (MNE1 > MNE || MNE2 > MNE)
        //若线段i垂距比大于i i+1垂距比　|| 线段i+1垂距比大于i i+1垂距比 || i i+1 点线最大距离　小于距离阈值  合并两条线段 chq
        if (MNE1 > MNE || MNE2 > MNE || distmax < DIST_THRESHOLD)//
        {
          ROS_INFO_STREAM("Laser2Line::merge.i " << i
                          <<"ratio i:" << MNE1
                          << " ratio i+1: " << MNE2
                          << " i i+1 ratio: "<< MNE
                          << "distmax i i+1 "<<distmax
                          << " DIST_THRESHOLD: "<<DIST_THRESHOLD<< " merge this line to next" );
          laser_line[i].end = laser_line[i+1].end;//
          laser_line.erase(laser_line.begin()+i+1);
          if(i>=0)    i --;
        //  printf("_");
        }
        else{
          ROS_INFO_STREAM("Laser2Line::merge.i " << i << "do not merge.just fitting.");
          line_fitting(i,laser_line[i].start,laser_line[i].end);
        }
      }
      else{
        ROS_INFO_STREAM("Laser2Line::merge.i " << i
            << " the dist bet cur line end to next line start is " << point_point_dist(laser_line[i].end,laser_line[i+1].start)
            << " large than thread:" << MAX_LINE2LINE_DIST << " do not merge.just fitting.");
        line_fitting(i,laser_line[i].start,laser_line[i].end);
      }
    }
  }
}

void Laser2Line::split_and_merge()
{
  Laser2Line::split(0, Laser_Data.size() - 1);
  Laser2Line::merge();
}

void Laser2Line::hough(void)
{
  for(int i=0; i<LASER_NUM; i++)
    for(int j=0; j<180; j++)
    {
      double this_row = Laser_Data[i].y*cos(j*M_PI/180.0) + Laser_Data[i].x*sin(j*M_PI/180.0);
      laser_hough[j].row[int(this_row)] ++;
    }

}

std::vector<segments> Laser2Line::get_line()
{
  return laser_line;
//  printf("set line data over size:%d\n",laser_line.size());
}
