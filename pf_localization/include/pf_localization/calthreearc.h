#ifndef CALTHREEARC_H
#define CALTHREEARC_H
#include <iostream>
#include <math.h>
#include "Geometry.h"
#include <ros/ros.h>
class CalThreeArc
{
public:
  CalThreeArc() {}
  CalThreeArc(VecPosition a,VecPosition b,VecPosition c,double ra,double rb,double rc):_pa(a),_pb(b),_pc(c),_ra(ra),_rb(rb),_rc(rc){

  }


  VecPosition DoABXiangliXiangQieOrXiangjiao(double x_a,double y_a,
                                             double x_b,double y_b,
                                             double x_c,double y_c,
                                             double R_a,double R_b,double R_c)
  {

    //计算A、B的交点位置
    double K_ab=(y_b - y_a)/(x_b - x_a);
    double Ang_ab = atan(K_ab);
    double D_ab = sqrt(pow(x_b - x_a, 2) + pow(y_b - y_a, 2));
    double L_ab,X_ab,Y_ab,X_ab1,Y_ab1,X_ab2,Y_ab2;

      //相离及相切
      if (D_ab - (R_a + R_b )>0.0)
      {
          X_ab = x_a + (x_b - x_a)*0.5*(R_a + D_ab - R_b)/D_ab;
          Y_ab = y_a + (y_b - y_a)*0.5*(R_a + D_ab - R_b)/D_ab;
          return VecPosition(X_ab,Y_ab);
      }
      //相交
      else
      {
          X_ab = x_a + (x_b - x_a)*(pow(R_a , 2) - pow(R_b , 2) + pow(D_ab, 2))/(2*pow(D_ab, 2));
          Y_ab = y_a + (X_ab - x_a)*K_ab;
          L_ab = sqrt(pow(R_a, 2) - pow(X_ab - x_a, 2) - pow(Y_ab - y_a, 2));
          X_ab1 = X_ab - L_ab*sin(Ang_ab);
          Y_ab1 = Y_ab + L_ab*cos(Ang_ab);
          X_ab2 = X_ab + L_ab*sin(Ang_ab);
          Y_ab2 = Y_ab - L_ab*cos(Ang_ab);
           //比较两个交点距离C点的距离与R_c的远近，选取一个合适的交点
          if (fabs(sqrt(pow(X_ab1 - x_c, 2) + pow(Y_ab1 - y_c, 2)) - R_c) < fabs(sqrt(pow(X_ab2 - x_c, 2) + pow(Y_ab2 - y_c, 2)) - R_c))
          {
            return VecPosition(X_ab1,Y_ab1);
          }
          else
          {
            return VecPosition(X_ab2,Y_ab2);
          }
      }
  }
  //下面函数借鉴了Geometry 的Circle::getIntersectionPoints函数
  //获得两个圆的交点：
  //１：互相包含　返回错误
  //２： 相交或者相切，返回中心连线和交点连线的交点
   int getCrossPointByTwoCircles(VecPosition va,VecPosition vb,VecPosition vc,
                                 double r0,double r1, double r2,
                                 VecPosition &result){
    int value = -1;
    double x0, y0;
    double x1, y1;
    double x2, y2;

    x0 = va.getX();
    y0 = va.getY();
    x1 = vb.getX();
    y1 = vb.getY();
    x2 = vc.getX();
    y2 = vc.getY();

    VecPosition p1,p2;//cross point
    double d, dx, dy, h, a, x, y, p2_x, p2_y;
    // first calculate distance between two centers circles P0 and P1.
    dx = x1 - x0;
    dy = y1 - y0;
    d = sqrt(dx*dx + dy*dy);

    // normalize differences
    dx /= d; dy /= d;
    double delta = d - r0 - r1;
    //相离且不包含　返回中心点连线与两圆相交，处在两圆中心构成的线段上的两个交点的中心
    //----------a----------(c0--(c_x,c_y)--c1)------b-------
    if( delta > EPSILON ){
       double c_x = x0+(r0+delta/2)*dx;
       double c_y = y0+(r0+delta/2)*dy;
        result.setVecPosition(c_x,c_y);
        value = 1;
    }
    //circle有包含关系　返回错误
    else
      if( d < fabs(r0-r1) ){
        ROS_ERROR("cal three arc.getCrossPointByTwoCircles.dist bet va:(%.6f,%.6f) and vb:(%.6f,%.6f) is:%.6f.the radius ra:%.6f,rb:%.6f.one circle is included by anther(dist < fabs(ra-rb)) .return -1",
                 x0,y0,x1,y1,d,r0,r1 );
         value = -1;
      }
      //否则　返回圆的交点连线与两圆中心点连线的交点
      else {
      a = (r0*r0 + d*d - r1*r1) / (2.0 * d);
      // h is then a^2 + h^2 = r0^2 ==> h = sqrt( r0^2 - a^2 )
      double arg = r0*r0 - a*a;
      h = (arg > 0.0) ? sqrt(arg) : 0.0;

      // First calculate P2
      p2_x = x0 + a * dx;
      p2_y = y0 + a * dy;

      // and finally the two intersection points
      x =  p2_x - h * dy;
      y =  p2_y + h * dx;
      p1.setVecPosition( x, y );
      x =  p2_x + h * dy;
      y =  p2_y - h * dx;
      p2.setVecPosition( x, y );
      double dist_c_p1 = fabs(p1.getDistanceTo(vc)-r2);
      double dist_c_p2 = fabs(p2.getDistanceTo(vc)-r2);
      result = dist_c_p1<dist_c_p2?p1:p2;//select the cross point which near to vc
      //VecPosition test_reslut = DoABXiangliXiangQieOrXiangjiao(x0,y0,x1,y1,x2,y2,r0,r1,r2);
     value = 1;
    }
    return value;
  }
   //计算由三个交点构成的三角形的内心
   VecPosition getTriangleInnerPoint(VecPosition v_a,VecPosition v_b,VecPosition v_c){

     double ab_dist = v_a.getDistanceTo(v_b);
     double ac_dist = v_a.getDistanceTo(v_c);
     double bc_dist = v_b.getDistanceTo(v_c);
     double len = ab_dist+ac_dist+bc_dist;
     double x = v_a.getX()*bc_dist+v_b.getX()*ac_dist+v_c.getX()*ab_dist;
     double y = v_a.getY()*bc_dist+v_b.getY()*ac_dist+v_c.getY()*ab_dist;
     return VecPosition(x/len,y/len);
 }

   //get the optimized cross point bet three circles
   int getCrossPoint( VecPosition& result ){
      int v1,v2,v3;
      VecPosition v_c_1,v_c_2,v_c_3;
        v1 = getCrossPointByTwoCircles(_pa,_pb,_pc,_ra,_rb,_rc,v_c_1);
      if( v1 > 0 )
        v2 = getCrossPointByTwoCircles(_pa,_pc,_pb,_ra,_rc,_rb,v_c_2);
      else{
        ROS_ERROR("cal three arc.getCrossPoint happen err bet pa:(%.6f,%.6f) and pb:(%.6f,%.6f)",
                  _pa.getX(),_pa.getY(),_pb.getX(),_pb.getY());
        return -1;
      }
      if( v2 > 0 )
        v3 = getCrossPointByTwoCircles(_pb,_pc,_pa,_rb,_rc,_ra,v_c_3);
      else{
        ROS_ERROR("cal three arc.getCrossPoint happen err bet pa:(%.6f,%.6f) and pc:(%.6f,%.6f)",
                  _pa.getX(),_pa.getY(),_pc.getX(),_pc.getY());
       return -1;
      }
      if( v3 > 0 )
        result = getTriangleInnerPoint(v_c_1,v_c_2,v_c_3);
      else{
        ROS_ERROR("cal three arc.getCrossPoint happen err bet pb:(%.6f,%.6f) and pc:(%.6f,%.6f)",
                  _pb.getX(),_pb.getY(),_pc.getX(),_pc.getY());
        return -1;
      }
      return 1;

  }
private:
  VecPosition _pa;
  VecPosition _pb;
  VecPosition _pc;
  double _ra,_rb,_rc;
};
#endif // CALTHREEARC_H
