/** \file
  \brief 2D point and oriented point plus operation
*/

#ifndef POINT_H
#define POINT_H
#include <cmath>
#include <iostream>
#include <vector>

 
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

#define DEG2RAD(degree) (degree * 0.01745329251994329576)
#define RAD2DEG(rad)    (rad * 57.29577951308232087721)
 

/**
  * @brief 2D point struct.
  *
  * point with x,y.support operation: + - *(1.point*k 2.k*point 3.point*point),operate only in numerical.
  */
  template <class T>
  struct point{
    inline point(T _x=0, T _y=0):x(_x),y(_y){}
    double mod()const {return hypot(x,y);}//hypot计算直角三角形的斜边长
    double dir()const {return atan2(y,x);}
    bool operator==(const point& p) const {
      return x == p.x && y == p.y;
    }
    T x, y;
  }; 
/**
  * @brief 2D oriented point struct.
  *
  * point with x,y and theta.Unit are m,m,rad
  * Support operation: Numerical: + - *   Geometry:absolute sum, absolute substract, euclidian distance
  */
  template <class T, class A>
  struct orientedpoint: public point<T>{
    inline orientedpoint(const point<T>& p, double dir = 0) :point<T>(p), theta(dir){}
    inline orientedpoint(T _x=0, T _y=0, A _theta=0): point<T>(_x,_y), theta(_theta){}
   /** @brief Normalize the theta into  [-pi,pi)
     * only done in object.
     */
    inline void normalize(){
      theta = atan2(sin(theta),cos(theta));
    } 
    A theta;
  };
 
  template <class T>
  inline point<T> operator+(const point<T>& p1, const point<T>& p2){
    return point<T>(p1.x + p2.x, p1.y + p2.y);
  }
   //add by chq ---------------->
  template <class T, class A>
  inline orientedpoint<T,A> operator-(const orientedpoint<T,A>& p1,const orientedpoint<T,A>& p2){
    return orientedpoint<T,A>(p1.x - p2.x, p1.y - p2.y, p1.theta - p2.theta);
  }
  //add by chq ----------------<

/** @brief get the motion from pose2 to pose1,motion is in pose2's coordinate system
  * pose1 = absoluteSum(pose2,reslut)
  */
  template <class T, class A>  //means the difference in p2's coodinate system  p1-p2
  orientedpoint<T,A> absoluteDifference(const orientedpoint<T,A>& p1,const orientedpoint<T,A>& p2){
    orientedpoint<T,A> delta=p1-p2;
    delta.theta=atan2(sin(delta.theta), cos(delta.theta));
    double s=sin(p2.theta), c=cos(p2.theta);
    return orientedpoint<T,A>(c*delta.x+s*delta.y,-s*delta.x+c*delta.y, delta.theta);
  }

/** @brief thansfer a point coodinate p1 from global coordinate system to p2's coordinate system, given p2's pose
  * p1 = absoluteSum(p2,reslut)
  */
  template<class T, class A>
  point<T> absoluteDifference(const point<T>& p1, const orientedpoint<T,A>& p2){
    orientedpoint<T,A> delta = absoluteDifference(orientedpoint<T,A>(p1.x, p1.y, 0), p2);
    return point<T>(delta.x, delta.y);
  } 

/** @brief add a motion p2 to pose p1. motion p2 is in p1's coordinate system
  * pose1 = absoluteSum(pose2,reslut)
  */
  template <class T, class A>  //add a action p2 to p1，p2 is in p1's coodinate system //此处相当于把世界坐标系和p2同时进行相同的旋转平移，由于p2一直是相对于世界坐标系的坐标位置，当把世界坐标系移动到p1坐标系的位置，此时p2的坐标就是在p1坐标系下了
  orientedpoint<T,A> absoluteSum(const orientedpoint<T,A>& p1,const orientedpoint<T,A>& p2){
    double s=sin(p1.theta), c=cos(p1.theta);
    orientedpoint<T,A> ans(c*p2.x-s*p2.y,s*p2.x+c*p2.y, p2.theta);
    ans = ans + p1;
    ans.normalize();
    return ans;
  }
/** @brief compute p2's global coordinate in p1's local coordinate system
  */
  template <class T, class A>
  point<T> absoluteSum(const orientedpoint<T,A>& p1,const point<T>& p2){
    double s=sin(p1.theta), c=cos(p1.theta);
    return point<T>(c*p2.x-s*p2.y, s*p2.x+c*p2.y) + (point<T>) p1;
  }

  ////// type def
 // typedef point<int> IntPoint;//disble by chq to forbid conflict to others
  typedef point<double> Point;
  typedef orientedpoint<double, double> OrientedPoint;

  typedef std::vector<Point> PointList; 
  typedef std::vector<OrientedPoint> OrientedPointList;
  //typedef std::vector<IntPoint> IntPointList;//disble by chq to forbid conflict to others


  extern PointList transformFrame(const PointList &points, const OrientedPoint &pose);


#endif


