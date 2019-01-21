/** \file purepursuit_planner.cc
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2014
 *
 * \brief purepursuit_planner class motorsDev
 * Component to manage the Agvs controller
 * (C) 2014 Robotnik Automation, SLL
*/
#include <string.h>
#include <vector>
#include <queue>
#include <stdint.h>//<stdint.h>中定义了几种扩展的整数类型和宏
#include <ros/ros.h>
#include <math.h>
#include <cstdlib>
#include "robotnik_pp_planner/pid.h"
#include "robotnik_pp_planner/Geometry.h"
#include <robotnik_pp_planner/Component.h>
#include <ackermann_msgs/AckermannDriveStamped.h>//disable by chq
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
#include <actionlib/server/simple_action_server.h>
#include <robotnik_pp_msgs/GoToAction.h>
#include <robotnik_pp_msgs/goal.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>///added for display path
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <robotnik_pp_planner/robotnik_pp_plannerConfig.h>
//#include <s3000_laser/enable_disable.h>


#define ODOM_TIMEOUT_ERROR          0.2        // max num. of seconds without receiving odom values
#define MAP_TIMEOUT_ERROR           0.2        // max num. of seconds without receiving map transformations
#define AGVS_TURN_RADIUS            0.20      // distancia en la que empieza a girar el robot cuando llega a una esquina
#define MIN_ANGLE_BEZIER            0.261799388    // ángulo (radianes) mínimo entre segmentos de la recta para los que ajustaremos a una curva de BEZIER//我们将调整为BEZIER曲线的线段之间的最小角度（弧度）
#define MIN_ANGLE_BSPLINE           0.261799388    // ángulo (radianes) mínimo entre segmentos de la recta para los que ajustaremos a una curva de BEZIER//我们将调整为BEZIER曲线的线段之间的最小角度（弧度）

#define BEZIER_CONTROL_POINTS          5
#define BSPLINE_CONTROL_POINTS         5
#define D_LOOKVERTICAL_MIN             0.3    // heading to vertical point limit dist chq
#define D_LOOKAHEAD_INC                0.01
#define D_LOOKAHEAD_MIN                0.1    // Minima distancia del punto objetivo en m (PurePursuit con lookahead dinámico)
#define D_LOOKAHEAD_MAX                1.1    // Maxima distancia del punto objetivo
#define D_WHEEL_ROBOT_CENTER           0.478   // Distance from the motor wheel to the robot center

//#define MAX_SPEED_LVL1            0.5
//#define MAX_SPEED_LVL2            0.3
#define MAX_SPEED_LVL1              0.3
#define MAX_SPEED_LVL2              0.2
#define MAX_SPEED                   1.2
//chq mod 190121 for rfs reloc check
//#define WAYPOINT_POP_DISTANCE_M              0.10    //Distancia mínima para alcanzar punto objetivo m (PurePursuit)
#define WAYPOINT_POP_DISTANCE_M              0.03    //Distancia mínima para alcanzar punto objetivo m (PurePursuit)

#define AGVS_FIRST_DECELERATION_DISTANCE     0.5   // meters -> when the vehicle is arriving to the goal, it has to decelarate at this distance
#define AGVS_FIRST_DECELERATION_MAXSPEED     0.15  // m/s
#define AGVS_SECOND_DECELERATION_DISTANCE    0.25   // meters -> when the vehicle is arriving to the goal, it has to decelarate another time at this distance
#define AGVS_SECOND_DECELERATION_MAXSPEED    0.1   // m/s
#define AGVS_DEFAULT_KR                      0.20            //


#define COMMAND_ACKERMANN              100
#define COMMAND_TWIST                  200
#define COMMAND_ACKERMANN_STRING       "Ackermann"
#define COMMAND_TWIST_STRING           "Twist"
//chq 切换算法rviz显示
enum look_dir{
  look_back = 0,//瞄准起点后方
  lool_ahead =1 //瞄准起点前方
};

enum{
  ODOM_SOURCE = 1,
  MAP_SOURCE = 2
};

look_dir g_dir = look_back;

using namespace std;

//! Data structure for a Magnet
typedef struct MagnetStruct{
    //! Id of the magnet
    int iID;
    //! X position
    double dX;
    //! Y position
    double dY;
}MagnetStruct;

//! Data structure for a Waypoint
typedef struct Waypoint{
    //! Id of the magnet
    int iID;
    //! X position
    double dX;
    //! Y position
    double dY;
    //! Orientation
    double dA;
    //! Speed to arrive to the point
    double dSpeed;
}Waypoint;

//! class to manage the waypoints and magnets of the current path
class Path{
  public:
  //! Current waypoint
  int iCurrentWaypoint;
  //! Current magnet
  int iCurrentMagnet;

  private:
  //! Mutex to control the access
  pthread_mutex_t mutexPath;
  //! Vector to store all the Waypoints
  vector <Waypoint> vPoints;
  //! Vector to store all the magnets
  vector <MagnetStruct> vMagnets;
  //! Flag to control the optimization
  bool bOptimized;

  public:

  //! public constructor
  Path(){
    iCurrentWaypoint = iCurrentMagnet = -1;
    pthread_mutex_init(&mutexPath, NULL);//Initialization for WaypointRoutes' mutex
    bOptimized = false;
  }

  //! Destructor
  ~Path(){
    pthread_mutex_destroy(&mutexPath);
  }

  //! Adds a new waypoint
  ReturnValue AddWaypoint(Waypoint point){
    Waypoint aux;

    pthread_mutex_lock(&mutexPath);
      if(vPoints.size() > 0){
        aux = vPoints.back();
        // Only adds the waypoint if it's different from waypoint before
        if( (aux.dX != point.dX) || (aux.dY != point.dY) )
          vPoints.push_back(point);
      } else { // First point
        if(iCurrentWaypoint < 0){ //First point
          iCurrentWaypoint = 0;
        }

        vPoints.push_back(point);
      }

    pthread_mutex_unlock(&mutexPath);

    return OK;
  }

  //! Adds a vector of waypoints
  ReturnValue AddWaypoint(vector <Waypoint> po){
    pthread_mutex_lock(&mutexPath);
      if(iCurrentWaypoint < 0){ //First point
        iCurrentWaypoint = 0;
      }
      for(int i = 0; i < po.size(); i++){
        vPoints.push_back(po[i]);
      }
    pthread_mutex_unlock(&mutexPath);
  }

  //! Creates a new point from the desired position to the first waypoint
  ReturnValue CreateInterpolatedWaypoint(geometry_msgs::Pose2D pose){
    ReturnValue ret = OK;

    pthread_mutex_lock(&mutexPath);

    if(vPoints.size()> 0){
      vector <Waypoint> vAuxPoints;
      Waypoint new_point;
      new_point.dX = (vPoints[0].dX - pose.x) / 2.0;
      new_point.dY = (vPoints[0].dY - pose.y) / 2.0;
      new_point.dSpeed = vPoints[0].dSpeed;
      new_point.dA = vPoints[0].dA;
      vAuxPoints.push_back(new_point);
      for(int i = 0; i < vPoints.size(); i++)
        vAuxPoints.push_back(vPoints[i]);
      vPoints = vAuxPoints;
    }else{
      ret  = ERROR;
    }

    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

  //! Adds a new magnet
  ReturnValue AddMagnet(MagnetStruct magnet){
    pthread_mutex_lock(&mutexPath);
    if(iCurrentMagnet < 0){ //First point
      iCurrentMagnet = 0;
    }
    vMagnets.push_back(magnet);
    pthread_mutex_unlock(&mutexPath);

    return OK;
  }

  //! Adds a vector of magnets
  ReturnValue AddMagnet(vector <MagnetStruct> po){
    pthread_mutex_lock(&mutexPath);
      if(iCurrentMagnet < 0){ //First point
        iCurrentMagnet = 0;
      }
      for(int i = 0; i < po.size(); i++){
        vMagnets.push_back(po[i]);
      }
    pthread_mutex_unlock(&mutexPath);
  }

  //! Clears the waypoints and magnets
  void Clear(){
    pthread_mutex_lock(&mutexPath);
      iCurrentWaypoint = -1;
      iCurrentMagnet = -1;
      bOptimized = false;
      vPoints.clear();
      vMagnets.clear();
    pthread_mutex_unlock(&mutexPath);
  }

  //! Returns the size of the vector points
  unsigned int Size(){
    return vPoints.size();
  }

  //! Returns the next waypoint
  ReturnValue GetNextWaypoint(Waypoint *wp){
    ReturnValue ret = ERROR;

    pthread_mutex_lock(&mutexPath);

      if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < (vPoints.size() - 1)) ){
        *wp = vPoints[iCurrentWaypoint + 1];
        ret = OK;
      }

    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

  //! Returns the last waypoint
  ReturnValue BackWaypoint(Waypoint *wp){
    ReturnValue ret = ERROR;

    pthread_mutex_lock(&mutexPath);
      if( vPoints.size() > 0){
        *wp = vPoints.back();
        ret = OK;
      }
    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

  //! Gets the current waypoint
  ReturnValue GetCurrentWaypoint(Waypoint *wp){
    ReturnValue ret = ERROR;

    pthread_mutex_lock(&mutexPath);
      if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < vPoints.size()) ){
        *wp = vPoints[iCurrentWaypoint];
        ret = OK;
      }
    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

  //! Gets selected waypoint
  ReturnValue GetWaypoint(int index, Waypoint *wp){
    ReturnValue ret = ERROR;

    pthread_mutex_lock(&mutexPath);
      if( (index >= 0) && ( index< vPoints.size() ) ){
        *wp = vPoints[index];
        ret = OK;
      }
    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

  //! Gets all waypoints chq
  std::vector <Waypoint> GetWaypoints(){
    return vPoints;
  }

  //! Gets the current Waypoint in the path
  int GetCurrentWaypointIndex(){
    return iCurrentWaypoint;
  }

  //!  Sets the current Waypoint to index
  ReturnValue SetCurrentWaypoint(int index){
    ReturnValue ret = ERROR;

    if(index < (vPoints.size() - 1)){
      pthread_mutex_lock(&mutexPath);
        iCurrentWaypoint = index;
      pthread_mutex_unlock(&mutexPath);
      ret = OK;
    }

    return ret;
  }

   //! Increase waypoint's number
  void NextWaypoint(){
    pthread_mutex_lock(&mutexPath);
      iCurrentWaypoint++;
    pthread_mutex_unlock(&mutexPath);
  }

  //! Returns the number of waypoints
  int NumOfWaypoints(){
    return vPoints.size();
  }

  //! Returns the next magnet
  ReturnValue GetNextMagnet(MagnetStruct *mg){
    ReturnValue ret = ERROR;
    pthread_mutex_lock(&mutexPath);

      if( (iCurrentMagnet >= 0) && (iCurrentMagnet < (vMagnets.size() - 1)) ){
        *mg = vMagnets[iCurrentMagnet + 1];
        ret = OK;
      }

    pthread_mutex_unlock(&mutexPath);


    return ret;
  }

  //! Returns the back magnet
  ReturnValue BackMagnet(MagnetStruct * mg){
    ReturnValue ret = ERROR;

     pthread_mutex_lock(&mutexPath);
      if( vMagnets.size() > 0){
        *mg = vMagnets.back();
        ret = OK;
      }
    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

  //! Gets the current magnet
  ReturnValue GetCurrentMagnet( MagnetStruct * mg ){
    ReturnValue ret = ERROR;

    pthread_mutex_lock(&mutexPath);
      if( (iCurrentMagnet >= 0) && (iCurrentMagnet < vMagnets.size()) ){
        *mg = vMagnets[iCurrentMagnet];
        ret = OK;
      }
    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

  //! Gets the previous magnet
  ReturnValue GetPreviousMagnet( MagnetStruct * mg ){
    ReturnValue ret = ERROR;

    pthread_mutex_lock(&mutexPath);
      if( (iCurrentMagnet > 0) && (iCurrentMagnet <= vMagnets.size()) ){
        *mg = vMagnets[iCurrentMagnet-1];
        ret = OK;
      }
    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

  //! Gets the current MagnetStruct in the path
  int GetCurrentMagnetIndex(){
    return iCurrentMagnet;
  }

  //!  Sets the current magnet to index
  ReturnValue SetCurrentMagnet(int index){
    ReturnValue ret = ERROR;

    if(index < (vMagnets.size() - 1)){
      pthread_mutex_lock(&mutexPath);
        iCurrentMagnet = index;
      pthread_mutex_unlock(&mutexPath);
      ret = OK;
    }
    return ret;
  }

  //! Increase magnet's number
  void NextMagnet(){
    pthread_mutex_lock(&mutexPath);
      iCurrentMagnet++;
    pthread_mutex_unlock(&mutexPath);
  }

   //! Gets the last magnet
  ReturnValue GetLastMagnet( MagnetStruct * mg ){
    ReturnValue ret = ERROR;

    pthread_mutex_lock(&mutexPath);
      if( (iCurrentMagnet > 0) && (iCurrentMagnet <= vMagnets.size() ) ){
        *mg = vMagnets[iCurrentMagnet - 1];
        ret = OK;
      }
    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

    //! Returns the number of magnets
  int NumOfMagnets(){
    return vMagnets.size();
  }

  //! Overloaded operator +=
  Path &operator+=(const Path &a){
    AddWaypoint(a.vPoints);
    AddMagnet(a.vMagnets);
    return *this;
  }

  //! Cross product
  double dot2( Waypoint w, Waypoint v) {
    return (w.dX*v.dX + w.dY*v.dY);
  }

  //! Obtains the points for a quadratic Bezier's curve
  //! \param cp0 as player_pose2d_t, control point 0
  //!  \param cp1 as player_pose2d_t, control point 1
  //!  \param cp2 as player_pose2d_t, control point 2
  //!  \param t as float, [0 ... 1]
  //!  \return Point over the curve
  Waypoint PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t){
    Waypoint aux;
    ///chq 二次Bezier曲线
    //B(t)= (1-t)^2 * cp0 + 2*t*(1-t)*cp1 + t^2 * cp2;
    //Bx(t)
    aux.dX = (1.0-t)*(1.0-t)*cp0.dX + 2*t*(1.0-t)*cp1.dX + t*t*cp2.dX;
    //By(t)
    aux.dY = (1.0-t)*(1.0-t)*cp0.dY + 2*t*(1.0-t)*cp1.dY + t*t*cp2.dY;

    return aux;
  }

  //! Obtains the points for a B spline curve
  //!  \param cp0 as player_pose2d_t, control point 0
  //!  \param cp1 as player_pose2d_t, control point 1
  //!  \param cp2 as player_pose2d_t, control point 2
  //!  \param cp3 as player_pose2d_t, control point 3
  //!  \param t as float, [0 ... 1] to get the mid point from <cp1 to cp2>
  //!  \return Point over the curve
  Waypoint PosOnCubicBSpline(Waypoint cp0, Waypoint cp1, Waypoint cp2,Waypoint cp3, float t){
    Waypoint aux;
    ///3次BSPLINAE曲线
    //B(t)= (1-t)^2 * cp0 + 2*t*(1-t)*cp1 + t^2 * cp2;
    //Bx(t)
    double a = 1.0/6.0;
    aux.dX =
        a * pow(1-t,3) * cp0.dX +
        a * ( 3 * pow(t,3) - 6 * pow(t,2) + 4 ) * cp1.dX +
        a * ( -3 * pow(t,3) + 3 * pow(t,2) + 3*t+1 ) * cp2.dX +
        a * pow(t,3) * cp3.dX;

    //By(t)
    aux.dY =
        a * pow(1-t,3) * cp0.dY +
        a * ( 3 * pow(t,3) - 6 * pow(t,2) + 4 ) * cp1.dY +
        a * ( -3 * pow(t,3) + 3 * pow(t,2) + 3*t+1 ) * cp2.dY +
        a * pow(t,3) * cp3.dY;

    return aux;
  }
  //! Function that calculate the distance to deccelerate from target speed
  //! \param target_speed as double, speed on m/s
  //! \return distance on meters
  double DistForSpeed(double target_speed){
    if(target_speed > 1.0)
      return 2.0;
    else if(target_speed > 0.8)
      return 1.5;
    else return
      1.0;

  }

  //! Modifies and adds new waypoints to the route for improving the path
  //! \param distance as double, used for the calculation of the new points
  //! \return ERROR if Size is lower than 3, distance <= 0 or the waypoints has already been optimized
  //! \return OK
  ReturnValue Optimize(double distance){
    int i, j=0;
    int a, b, c;
    int x = 0, y = 1, speed = 2;
    double mod_ab, mod_bc;
    double dAngle;
    Waypoint ab, bc, ba;
    double K= 0.0;
    vector <Waypoint> new_points;
    Waypoint aux;
    double Ax = 0.0, Ay = 0.0, Bx = 0.0, By = 0.0, Cx = 0.0, Cy = 0.0;
    Waypoint A, B, C;
    double Kt = 1.0 / BEZIER_CONTROL_POINTS;
    double dAuxSpeed = 0.0;
    double dMinDist = 0.0;  // Minica distancia a la que hay q frenar en funcion de la velocidad

    if(bOptimized){  //Already optimized
      return OK;
    }

    if((vPoints.size() < 2) || (distance <= 0.0)){  //Minimo 3 puntos en la ruta
      //printf("WaypointRoute::Optimize: Error: not enought points (%d)\n",Size());
      return ERROR;
    }
    pthread_mutex_lock(&mutexPath);
      //
      // Si solo hay dos puntos, interpolamos y creamos un punto intermedio
    //if only have 2 points then we create a mid point
      if(vPoints.size() == 2){
        aux = vPoints[1];
        vPoints.push_back(aux); // Añadimos un punto al final y modificamos el del medio  //we add a point in the end and change the mid point

        if((vPoints[0].dX - aux.dX) == 0.0){// Punto en el mismo eje X
          vPoints[1].dX = vPoints[0].dX;    // Mantenemos la coordenada en X
          vPoints[1].dY = vPoints[0].dY + (aux.dY - vPoints[0].dY) / 2.0; // La coordenada en Y será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
        }else if((vPoints[0].dY - aux.dY) == 0.0){ // Punto en el mismo eje Y
          vPoints[1].dX = vPoints[0].dX + (aux.dX - vPoints[0].dX) / 2.0; // La coordenada en X será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
          vPoints[1].dY = vPoints[0].dY;    // Mantenemos la coordenada en Y
        }else{ // Punto en eje X, Y distinto
          vPoints[1].dX = vPoints[0].dX + (aux.dX - vPoints[0].dX) / 2.0; // La coordenada en X será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
          vPoints[1].dY = vPoints[0].dY + (aux.dY - vPoints[0].dY) / 2.0; // La coordenada en Y será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
        }
      }

      new_points.push_back(vPoints[0]);
      new_points.push_back(vPoints[1]);

      for(i=2; i < vPoints.size(); i++){  // Primera pasada, añadimos puntos para los giros en curvas
        //cout << " Partiendo de punto " << i << endl;
        a = i-2;
        b = i-1;
        c = i;

        Ax = vPoints[a].dX;
        Ay = vPoints[a].dY;
        Bx = vPoints[b].dX;
        By = vPoints[b].dY;
        Cx = vPoints[c].dX;
        Cy = vPoints[c].dY;

        ab.dX = Bx - Ax;
        ab.dY = By - Ay;
        mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);

        bc.dX = Cx - Bx;
        bc.dY = Cy - By;
        mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);

        dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));
        //cout <<  i << " Angle =  "<< dAngle << endl;
        if(fabs(dAngle) >= MIN_ANGLE_BEZIER){ // en caso del angulo que forman los segmentos sea menor, entonces generaremos puntos para aproximar mjor la curva
          // siendo ba vector director de b->a y bc el vector director b->c
          // Calculamos un punto a una distancia 'd' desde 'b' hacia 'a' y otro punto desde 'b' hacia 'c'
          // Estos puntos serán los que se añadirán a la lista de waypoints para poder trazar una curva de bezier
          //1和２之间要插入新点，所以第二个点要先pop
          new_points.pop_back();

          // Calcula velocidad maxima en funcion del giro de la curva
          if(fabs(dAngle) >= (Pi/4)){
            dAuxSpeed = MAX_SPEED_LVL2;
          }else
            dAuxSpeed = MAX_SPEED_LVL1;
          //cout << "Aux speed = " << dAuxSpeed << ", Next speed =  " << vPoints[b].dSpeed << endl;
          // Si la velocidad en ese waypoint supera el máximo establecido para un giro así
          if(fabs(vPoints[b].dSpeed) > dAuxSpeed){

            if(vPoints[b].dSpeed < 0.0)  // Cambiamos sentido de avance
              dAuxSpeed = -dAuxSpeed;

            dMinDist = DistForSpeed(fabs(vPoints[b].dSpeed));//根据速度确定最小限定距离
            //cout << "Min dist = " << dMinDist  << endl;
            // Si el punto antes del giro esta a una distancia menor a la mínima, añadimos nuevo punto a un metro
            if( mod_ab > dMinDist){//如果两点距离超过了速度决定的限定距离，可以再插入点
              //Lo creamos//we create
              /// chq if the dist bett a and b is much more bigger, then we  create a mid point after a
              ba.dX = -ab.dX;
              ba.dY = -ab.dY;
              K = dMinDist / mod_ab;
              //似乎是简单的线性插值
              aux.dX = Bx + K * ba.dX;  // x = x' + K*Vx
              aux.dY = By + K * ba.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
              aux.dSpeed = vPoints[b].dSpeed;
              //cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
              new_points.push_back(aux);
            }

            vPoints[b].dSpeed = dAuxSpeed;

          }
          // El primer waypoint no se modifica//第一个航点不会被修改
          //大于最小间隔距离，插入点
          if(mod_ab > distance){ // si la distancia entre ab es MAYOR a la distancia del punto que pretendemos crear, creamos un punto intermedio
            ///Lo creamos
            /// chq if the dist bet a and b is beyond the limit dist then we create a mid point
            ba.dX = -ab.dX;
            ba.dY = -ab.dY;
            K = distance / mod_ab;

            aux.dX = Bx + K * ba.dX;  // x = x' + K*Vx
            aux.dY = By + K * ba.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
            aux.dSpeed = vPoints[b].dSpeed;
            //cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
            new_points.push_back(aux);
            //j++;
          }

          new_points.push_back(vPoints[b]);

          if(mod_bc > distance){ // si la distancia entre ab es menor a la distancia del punto que pretendemos crear, lo dejamos como está
            ///Lo creamos// chq if the dist bet b and c is beyond the limit dist then we create a mid point
            K = distance / mod_bc;
            //K = distance / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
            aux.dX = Bx + K * bc.dX;  // x = x' + K*Vx
            aux.dY = By + K * bc.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
            aux.dSpeed = vPoints[b].dSpeed;
            //j++;
            //cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
            new_points.push_back(aux);
          }

          // Creamos punto para después del giro
          if(dMinDist > 0.0) {
            if(mod_bc > 1.0){  // si la distancia al punto C es mayor que 1 metro, después del giro
              /// chq if the dist bet b and c is beyond 1 m then we create a mid point
              // Creamos un nuevo punto
              //Lo creamos
              K = 1.0 / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
              aux.dX = Bx + K * bc.dX;  // x = x' + K*Vx
              aux.dY = By + K * bc.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
              aux.dSpeed = vPoints[b].dSpeed;
              new_points.push_back(aux);
            }else{
              // Si no, establecemos una velocidad máxima
              vPoints[c].dSpeed = vPoints[b].dSpeed;
            }
          }


          new_points.push_back(vPoints[c]);
          //j++;
        }else{  //Se queda como está

          new_points.push_back(vPoints[c]);

        }
      }

      // Borramos antiguos waypoints e insertamos los nuevos
      vPoints.clear();
      // BEZIER
      vPoints.push_back(new_points[0]);
      vPoints.push_back(new_points[1]);
      for(i=2; i < new_points.size(); i++){  // Segunda pasada, aproximamos los giros a curvas de Bezier
        a = i-2;
        b = i-1;
        c = i;

        Ax = new_points[a].dX;
        Ay = new_points[a].dY;
        Bx = new_points[b].dX;
        By = new_points[b].dY;
        Cx = new_points[c].dX;
        Cy = new_points[c].dY;

        ab.dX = Bx - Ax;
        ab.dY = By - Ay;
        mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);

        bc.dX = Cx - Bx;
        bc.dY = Cy - By;
        mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);

        dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));

        if(fabs(dAngle) >= MIN_ANGLE_BEZIER){ // en caso del angulo que forman los segmentos sea menor, entonces generaremos puntos, siguiendo una curva de Bezier, para aproximar mjor la curva
          // siendo ba vector director de b->a y bc el vector director b->c

          Waypoint aux_wp;
          double t, aux_speed;

          A = new_points[a];
          B = new_points[b];
          C = new_points[c];

          aux_speed = new_points[b].dSpeed; //takes speed of the waypoint in the middle
          vPoints.pop_back();      // Eliminamos el waypoint del medio. El primer waypoint no se modifica

          for(int j=1; j <= BEZIER_CONTROL_POINTS; j++) {
            t = (double) j * Kt;
            aux_wp = PosOnQuadraticBezier(A, B, C,  t);
            aux_wp.dSpeed = aux_speed;
            vPoints.push_back(aux_wp);
          //  std::cout << "\tWaypointRoute::Optimize: (Bezier) Waypoint,X= " << aux_wp.dX << " Y= " << aux_wp.dY
          //        << " size= " << (int)points.size() << endl;
          }
        }
        else{  //Se queda como está

          vPoints.push_back(new_points[c]);
        }
      }

      iCurrentWaypoint = 0;

    //  for(int i = 0; i < new_points.size(); i++){
    //  points.push_back(new_points[i]);
    //  }

    pthread_mutex_unlock(&mutexPath);

    bOptimized  = true;
    new_points.clear();

    return OK;
  }

  //! Modifies and adds new waypoints to the route for improving the path
  //! \param distance as double, used for the calculation of the new points
  //! \return ERROR if Size is lower than 4, distance <= 0 or the waypoints has already been optimized
  //! \return OK
  ReturnValue OptimizeByBSpline(double distance){
    int i, j=0;
    int befa,a, b, c,d;
    int x = 0, y = 1, speed = 2;
    double mod_ab, mod_bc,mod_cd;
    double dAngle,dAngle1;
    Waypoint ab, bc, ba,cd;
    double K= 0.0;
    vector <Waypoint> new_points;
    Waypoint aux;
    double BefAx,BefAy,Ax = 0.0, Ay = 0.0, Bx = 0.0, By = 0.0, Cx = 0.0, Cy = 0.0, Dx = 0.0, Dy = 0.0;
    Waypoint BefA,A, B, C, D;
    double Kt = 1.0 / BSPLINE_CONTROL_POINTS;
    double dAuxSpeed = 0.0;
    double dMinDist = 0.0;  // Minica distancia a la que hay q frenar en funcion de la velocidad
    int have_beyond_onemeter = false;
    if(bOptimized){  //Already optimized
      return OK;
    }

    if((vPoints.size() < 4) || (distance <= 0.0)){  //Minimo 3 puntos en la ruta
      ROS_ERROR("WaypointRoute::OptimizeByBSpline: Error: not enought points (%d)\n",Size());
      return ERROR;
    }

    pthread_mutex_lock(&mutexPath);
    ///added by chq -s

    if(vPoints[0].dSpeed>MAX_SPEED_LVL1)
      vPoints[0].dSpeed = MAX_SPEED_LVL1;
    if(vPoints[0].dSpeed<-MAX_SPEED_LVL1)
      vPoints[0].dSpeed = -MAX_SPEED_LVL1;

    if(vPoints[1].dSpeed>MAX_SPEED_LVL1)
      vPoints[1].dSpeed = MAX_SPEED_LVL1;
    if(vPoints[1].dSpeed<-MAX_SPEED_LVL1)
      vPoints[1].dSpeed =- MAX_SPEED_LVL1;

    ///added by chq -e
    new_points.push_back(vPoints[0]);
    new_points.push_back(vPoints[1]);
    for(i=2; i < vPoints.size(); i++){  // Primera pasada, añadimos puntos para los giros en curvas
      //cout << " Partiendo de punto " << i << endl;
      a = i-2;
      b = i-1;
      c = i;

      Ax = vPoints[a].dX;
      Ay = vPoints[a].dY;
      Bx = vPoints[b].dX;
      By = vPoints[b].dY;
      Cx = vPoints[c].dX;
      Cy = vPoints[c].dY;

      ab.dX = Bx - Ax;
      ab.dY = By - Ay;
      mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);

      bc.dX = Cx - Bx;
      bc.dY = Cy - By;
      mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);

      dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));
      ///add by chq ---s-----
      /// //force limit start speed in two meter
      double dist;
     if( !have_beyond_onemeter ){
       dist = pow(vPoints[c].dX-vPoints[0].dX,2)+pow(vPoints[c].dY-vPoints[0].dY,2);
       if(dist>2*2)
         have_beyond_onemeter = true;
     }
     if(!have_beyond_onemeter){
       double sp = new_points[c].dSpeed;
       if(sp>MAX_SPEED_LVL1)vPoints[c].dSpeed=MAX_SPEED_LVL1;
       if(sp<-MAX_SPEED_LVL1)vPoints[c].dSpeed=-MAX_SPEED_LVL1;
     }
/// //force limit end speed at the last two meter
     double dist_end = pow(vPoints[c].dX-vPoints[vPoints.size()-1].dX,2)+pow(vPoints[c].dY-vPoints[vPoints.size()-1].dY,2);
     if(dist_end < 2*2 ){
       double sp = new_points[c].dSpeed;
       if(sp>MAX_SPEED_LVL1)vPoints[c].dSpeed=MAX_SPEED_LVL1;
       if(sp<-MAX_SPEED_LVL1)vPoints[c].dSpeed=-MAX_SPEED_LVL1;
     }
     ///add by chq ---end------
      //cout <<  i << " Angle =  "<< dAngle << endl;
      if(fabs(dAngle) >= MIN_ANGLE_BSPLINE){ // en caso del angulo que forman los segmentos sea menor, entonces generaremos puntos para aproximar mjor la curva
        // siendo ba vector director de b->a y bc el vector director b->c
        // Calculamos un punto a una distancia 'd' desde 'b' hacia 'a' y otro punto desde 'b' hacia 'c'
        // Estos puntos serán los que se añadirán a la lista de waypoints para poder trazar una curva de bezier
        //1和２之间要插入新点，所以第二个点要先pop
        new_points.pop_back();

        // Calcula velocidad maxima en funcion del giro de la curva
        if(fabs(dAngle) >= (Pi/4)){
          dAuxSpeed = MAX_SPEED_LVL1;
        }else
          dAuxSpeed = MAX_SPEED_LVL2;
        //cout << "Aux speed = " << dAuxSpeed << ", Next speed =  " << vPoints[b].dSpeed << endl;
        // Si la velocidad en ese waypoint supera el máximo establecido para un giro así
        /// if vPoints[b].dSpeed > dAuxSpeed
        /// set从A到auxPoint(缓冲点)速度保持为vPoints[b].dSpeed原始速度，
        /// 从auxPoint到vPoints[b]速度降低为限制速度dAuxSpeed
        if(fabs(vPoints[b].dSpeed) > dAuxSpeed){

          if(vPoints[b].dSpeed < 0.0)  // Cambiamos sentido de avance
            dAuxSpeed = -fabs(dAuxSpeed);

          dMinDist = DistForSpeed(fabs(vPoints[b].dSpeed));//根据速度确定最小限定距离
          //cout << "Min dist = " << dMinDist  << endl;
          // Si el punto antes del giro esta a una distancia menor a la mínima, añadimos nuevo punto a un metro
          ///减去１的目的是上一循环可能在１m处插入了缓冲点 chq
          /// //b-->minturnp---->1m-->mindist------------>c
          if( mod_ab - 1 > dMinDist ){//如果两点距离超过了速度决定的限定距离，可以再插入点
            //Lo creamos//we create
            /// chq if the dist bett a and b is much more bigger, then we  create a mid point after a
            ba.dX = -ab.dX;
            ba.dY = -ab.dY;
            K = dMinDist / mod_ab;
            //似乎是简单的线性插值
            aux.dX = Bx + K * ba.dX;  // x = x' + K*Vx
            aux.dY = By + K * ba.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director

            aux.dSpeed = vPoints[b].dSpeed;
            new_points.push_back(aux);
          }

          vPoints[b].dSpeed = dAuxSpeed;

        }
        // El primer waypoint no se modifica//第一个航点不会被修改
        //在即将到转折点的地方，按照最小转弯半径distance插入点
        if(mod_ab > distance){ // si la distancia entre ab es MAYOR a la distancia del punto que pretendemos crear, creamos un punto intermedio
          ///Lo creamos
          /// chq if the dist bet a and b is beyond the limit dist then we create a mid point
          ba.dX = -ab.dX;
          ba.dY = -ab.dY;
          K = distance / mod_ab;

          aux.dX = Bx + K * ba.dX;  // x = x' + K*Vx
          aux.dY = By + K * ba.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
          ///added by chq 转弯速度限制为dAuxSpeed -start
          if(vPoints[b].dSpeed < 0.0)  // Cambiamos sentido de avance
            dAuxSpeed = -fabs(dAuxSpeed);
          aux.dSpeed = dAuxSpeed;
          ///added by chq -end

          ///aux.dSpeed = vPoints[b].dSpeed;
          new_points.push_back(aux);
          //j++;
        }
        ///增加完缓冲点+最小提前转弯转折点后，加入原有的转弯点
        new_points.push_back(vPoints[b]);
        ///在原有转弯点的前方加入最小转弯点
        if(mod_bc > distance){ // si la distancia entre ab es menor a la distancia del punto que pretendemos crear, lo dejamos como está
          ///Lo creamos// chq if the dist bet b and c is beyond the limit dist then we create a mid point
          K = distance / mod_bc;
          //K = distance / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
          aux.dX = Bx + K * bc.dX;  // x = x' + K*Vx
          aux.dY = By + K * bc.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
          ///added by chq start
          if(vPoints[b].dSpeed < 0.0)  // Cambiamos sentido de avance
            dAuxSpeed = -fabs(dAuxSpeed);
          aux.dSpeed = dAuxSpeed;
          ///added by chq end
          ///aux.dSpeed = vPoints[b].dSpeed;
          //j++;
          //cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
          new_points.push_back(aux);
        }
        ///加入缓冲点
        // Creamos punto para después del giro
        if( dMinDist > 0.0) {
          ///减去distance的目的是　为了防止下一个点在距离其前方distance处插入最小转弯点（１m缓冲点必须在下一个最小转弯点前方） chq
          //a-------->1mp-->minturnp---->b
          if( mod_bc-distance > 1.0){  // si la distancia al punto C es mayor que 1 metro, después del giro
            /// chq if the dist bet b and c is beyond 1 m then we create a mid point
            // Creamos un nuevo punto
            // Lo creamos
            K = 1.0 / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
            aux.dX = Bx + K * bc.dX;  // x = x' + K*Vx
            aux.dY = By + K * bc.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
            aux.dSpeed = vPoints[b].dSpeed;
            new_points.push_back(aux);
          }
          else
          {
            // Si no, establecemos una velocidad máxima
            //vPoints[c].dSpeed = vPoints[b].dSpeed;//disabled by chq
            if(vPoints[b].dSpeed < 0.0)
              dAuxSpeed = -fabs(dAuxSpeed);
            vPoints[c].dSpeed = dAuxSpeed;
          }
        }

       ///加入原有转弯后的点
        new_points.push_back(vPoints[c]);
        //j++;
      }
      else{  //Se queda como está
        //如果没有转折，直接加入下一个点
        new_points.push_back(vPoints[c]);
      }
    }

    // Borramos antiguos waypoints e insertamos los nuevos
    vPoints.clear();
    // BSPLINE
    vPoints.push_back(new_points[0]);
    vPoints.push_back(new_points[1]);
    for(i=3; i < new_points.size(); i++){  // Segunda pasada, aproximamos los giros a curvas de BSPLINE
      a = i-3;
      b = i-2;
      c = i-1;
      d = i;

      Ax = new_points[a].dX;
      Ay = new_points[a].dY;
      Bx = new_points[b].dX;
      By = new_points[b].dY;
      Cx = new_points[c].dX;
      Cy = new_points[c].dY;
      Dx = new_points[d].dX;
      Dy = new_points[d].dY;

      ab.dX = Bx - Ax;
      ab.dY = By - Ay;
      mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);

      bc.dX = Cx - Bx;
      bc.dY = Cy - By;
      mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);

      cd.dX = Dx - Cx;
      cd.dY = Dy - Cy;
      mod_cd = sqrt(cd.dX * cd.dX + cd.dY * cd.dY);

      dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));
      dAngle1= acos(dot2(bc,cd)/(mod_bc*mod_cd));

      //夹角超过阈值的才插入BSPLINE点
     #if 0
      if(fabs(dAngle) >= MIN_ANGLE_BSPLINE && mod_ab>0.3 && mod_bc>0.3){ // en caso del angulo que forman los segmentos sea menor, entonces generaremos puntos, siguiendo una curva de Bezier, para aproximar mjor la curva
        // siendo ba vector director de b->a y bc el vector director b->c
        Waypoint aux_wp;
        double t, aux_speed;

        A = new_points[a];
        B = new_points[b];
        C = new_points[c];
        D = new_points[d];
        aux_speed = new_points[b].dSpeed; //takes speed of the waypoint in the middle
        vPoints.pop_back();      // Eliminamos el waypoint del medio. El primer waypoint no se modifica
        //基于三次B样条特性，为了平滑转角两端，我们必须分别用3nd B样条曲线分别在A-B,A-B之间进行平滑
        if( i-3 > 0){
          //insert bet bef A and B
           befa = i-4;
           BefA = new_points[befa];
           for(int j=1; j <= BSPLINE_CONTROL_POINTS; j++) {
             t = (double) j * Kt;
             //t 0->1 point A->B
             aux_wp = PosOnCubicBSpline(BefA, A, B, C, t);
             aux_wp.dSpeed = aux_speed;
             vPoints.push_back(aux_wp);
           //  std::cout << "\tWaypointRoute::Optimize: (Bezier) Waypoint,X= " << aux_wp.dX << " Y= " << aux_wp.dY
           //        << " size= " << (int)points.size() << endl;
           }

        }
        for(int j=1; j <= BSPLINE_CONTROL_POINTS; j++) {
          //insert bet bef B and C
          t = (double) j * Kt;
          //t 0->1 point B->C
          aux_wp = PosOnCubicBSpline(A, B, C, D, t);
          aux_wp.dSpeed = aux_speed;
          vPoints.push_back(aux_wp);
        //  std::cout << "\tWaypointRoute::Optimize: (Bezier) Waypoint,X= " << aux_wp.dX << " Y= " << aux_wp.dY
        //        << " size= " << (int)points.size() << endl;
        }
      }
      else
      #endif
      {  //Se queda como está

        vPoints.push_back(new_points[c]);
      }
    }
    ///push back the last point
    vPoints.push_back(new_points[new_points.size()-1]);
    iCurrentWaypoint = 0;

  //  for(int i = 0; i < new_points.size(); i++){
  //  points.push_back(new_points[i]);
  //  }

  pthread_mutex_unlock(&mutexPath);

  bOptimized  = true;
  new_points.clear();

  return OK;
  }

  //! Prints all the waypoints
  void Print(){
    cout << "Path::Print: Printing all the waypoints..." << endl;
    if(vPoints.size() > 0){
      for(int i = 0; i < vPoints.size(); i++){
        cout << "(" << i << ")\t " << vPoints[i].dX << ", " << vPoints[i].dY << endl;
      }
    }else
      cout << "Path::Print: No waypoints..." << endl;
  }

};


class purepursuit_planner_node: public Component
{

private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  double desired_freq_;
  //! constant for Purepursuit
  double Kr;

  //! Variable lookahead
    double dLookAhead;
     //! Object with the current path that the robot is following
    Path pathCurrent;
    //! Object with the path that is being filled
    Path pathFilling;
    //! Vector with next paths to follow
    queue <Path> qPath;
    //! current robot's position 
    geometry_msgs::Pose2D pose2d_robot;
    //! current robot's odometry
    nav_msgs::Odometry odometry_robot;
    //! current robot's linear speed
    double dLinearSpeed;
    //! Lookahead bounds
    double look_ahead_inc,d_lookahear_min_, d_lookahear_max_;
    double d_lookvertical_min_;//大于这个距离，瞄向垂点
    //! Distance from the robot center to the wheel's center
    double d_dist_wheel_to_center_;
    //! Max allowed speed
    double max_speed_;
    //! Flag to enable/disable the motion
    bool bEnabled;
    //! Flag to cancel the following path
    bool bCancel;
    //! Mode for reading the position of the robot ("ODOM", "MAP")
    std::string position_source_;
    //! Target frame for the transform from /map to it
    std::string target_frame_;
    //! Mode in numeric format
    unsigned int ui_position_source;
    //!  Sets the type of command to send to the robot (Twist or ackermann)
    int command_type;
    //! Direction of movement (-1 or +1)
    int direction;

    bool need_repub_path;
    bool comp_firstpoint;
   ///PID
    double dt , maxT , minT, Kp , Ki, Kd;
    double dtS, maxS , minS, KpS, KiS, KdS;
    PID* pidTheta ;
    PID* pidVelocity;

    ///reconfigure
    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<robotnik_pp_planner::robotnik_pp_plannerConfig> *dsrv_;
   // void reconfigureCB(robotnik_pp_planner::robotnik_pp_plannerConfig &config, uint32_t level);
    robotnik_pp_planner::robotnik_pp_plannerConfig default_config_;


  //////// ROS
  //! pub the path points chq
  ros::Publisher path_pub_;
  ros::Publisher path_marker_pub;
  ros::Publisher tar_marker_pub;

  //! Publishes the status of the robot
  ros::Publisher status_pub_;
  //! Publish to cmd vel (Ackermann)
  //! It will publish into command velocity (for the robot)
  ros::Publisher vel_pub_;
  //! publish the transformation between map->base_link
  ros::Publisher tranform_map_pub_;
  //! it subscribes to /odom
  ros::Subscriber odom_sub_;
  //! Topic name to read the odometry from
  std::string odom_topic_;
  //! Topic name to publish the vel & pos commands
  std::string cmd_topic_vel_;
  // DIAGNOSTICS
  //! Diagnostic to control the frequency of the published odom
  diagnostic_updater::TopicDiagnostic *updater_diagnostic_odom;
  //! General status diagnostic updater
  diagnostic_updater::Updater updater_diagnostic;
  //! Diagnostics min & max odometry freq
  double min_odom_freq, max_odom_freq;
  //! Saves the time whenever receives a command msg and a transform between map and base link (if configured)
  ros::Time last_command_time, last_map_time;
  // ACTIONLIB
  actionlib::SimpleActionServer<robotnik_pp_msgs::GoToAction> action_server_goto;
  robotnik_pp_msgs::GoToFeedback goto_feedback;
  robotnik_pp_msgs::GoToResult goto_result;
  robotnik_pp_msgs::GoToGoal goto_goal;
  // TFs
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // SERVICES
  //! service name to enable disable lasers
  std::string name_sc_enable_front_laser_, name_sc_enable_back_laser_;
  //! Service to enable/disable front laser
  ros::ServiceClient sc_enable_front_laser_;
  //! Service to enable/disable back laser
  ros::ServiceClient sc_enable_back_laser_;

public:

  /*!  \fn summit_controller::purepursuit_planner()
   *   \brief Public constructor
  */
  purepursuit_planner_node(ros::NodeHandle h) :
    node_handle_(h),
    private_node_handle_("~"),
    desired_freq_(50.0),
    Component(desired_freq_),
    action_server_goto(node_handle_, ros::this_node::getName(), false)
  // boost::bind(&purepursuit_planner_node::executeCB, this, _1), false)
  {
    bRunning = false;

    ROSSetup();

    dLookAhead = d_lookahear_min_;
    dLinearSpeed = 0;
    pose2d_robot.x = pose2d_robot.y = pose2d_robot.theta = 0.0;
    bEnabled = true;
    bCancel = false;

    comp_firstpoint = false;
    need_repub_path = true;
    sComponentName.assign("purepursuit_planner_node");
    iState = INIT_STATE;
    direction = 0;

    ///
    /// \brief reconfig
    ///
    dsrv_ = new dynamic_reconfigure::Server<robotnik_pp_planner::robotnik_pp_plannerConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<robotnik_pp_planner::robotnik_pp_plannerConfig>::CallbackType cb = boost::bind(&purepursuit_planner_node::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    ROS_INFO_STREAM("purepursuit_planner_node init done!ros::this_node::getName():"<<ros::this_node::getName());
  }

  /*!  \fn purepursuit_planner::~purepursuit_planner()
   *   \brief Public destructor
  */
  ~purepursuit_planner_node(){

  }

  /*!  \fn oid ROSSetup()
   *   \brief Setups ROS' stuff
  */
  void ROSSetup(){
    string s_command_type;

    private_node_handle_.param<std::string>("odom_topic", odom_topic_, "/wheel_diff_controller/odom");
    private_node_handle_.param("cmd_topic_vel", cmd_topic_vel_, std::string("/wheel_diff_controller/cmd_vel"));
    private_node_handle_.param("d_lookvertical_min", d_lookvertical_min_, D_LOOKVERTICAL_MIN);
    private_node_handle_.param("d_lookahear_min", d_lookahear_min_, D_LOOKAHEAD_MIN);
    private_node_handle_.param("look_ahead_inc", look_ahead_inc, D_LOOKAHEAD_INC);

    private_node_handle_.param("d_lookahear_max", d_lookahear_max_, D_LOOKAHEAD_MAX);
    private_node_handle_.param("d_dist_wheel_to_center", d_dist_wheel_to_center_, D_WHEEL_ROBOT_CENTER);
    private_node_handle_.param("max_speed", max_speed_, MAX_SPEED);
    private_node_handle_.param("kr", Kr, AGVS_DEFAULT_KR);
    private_node_handle_.param<std::string>("position_source", position_source_, "ODOM");
    private_node_handle_.param("desired_freq", desired_freq_, desired_freq_);
    private_node_handle_.param<std::string>("command_type", s_command_type, COMMAND_TWIST_STRING);
    private_node_handle_.param<std::string>("target_frame", target_frame_, "/base_link");
   //PID
    private_node_handle_.param("max_w", maxT, M_PI/2);
    private_node_handle_.param("min_w", minT, -M_PI/2);
    private_node_handle_.param("kp_w", Kp, 1.0);
    private_node_handle_.param("ki_w", Ki, 0.00);
    private_node_handle_.param("kd_w", Kd, 0.000);
    if(desired_freq_)
      dt = 1.0/desired_freq_;
    else
      dt = 0.1;
    private_node_handle_.param("max_v", maxS, 1.5);
    private_node_handle_.param("min_v", minS, 0.1);
    private_node_handle_.param("kp_v", KpS, 0.8);
    private_node_handle_.param("ki_v", KiS, 0.05);
    private_node_handle_.param("kd_v", KdS, 0.005);
    if(desired_freq_)
      dtS = 1.0/desired_freq_;
    else
      dtS = 0.1;
     pidTheta = new PID(dt, maxT, minT, Kp, Kd, Ki);
     pidVelocity = new PID(dtS, maxS, minS, KpS, KdS, KiS);
    //private_node_handle_.param<std::string>("name_sc_enable_frot_laser_", name_sc_enable_front_laser_, "/s3000_laser_front/enable_disable");
    //private_node_handl_.param<std::string>("name_sc_enable_back_laser", name_sc_enable_back_laser_, "/s3000_laser_back/enable_disable"  );

    if(s_command_type.compare(COMMAND_ACKERMANN_STRING) == 0){
      command_type = COMMAND_ACKERMANN;
    }else if(s_command_type.compare(COMMAND_TWIST_STRING) == 0){
      command_type = COMMAND_TWIST;
    }else{
      // default value
      command_type = COMMAND_TWIST;
      d_dist_wheel_to_center_ = 1.0;
    }

    // From Component class
    threadData.dDesiredHz = desired_freq_;

    if(position_source_ == "MAP")
      ui_position_source = MAP_SOURCE;
    else
      ui_position_source = ODOM_SOURCE;
    ROS_INFO_STREAM("robotnik_pureursuit.ui_position_source:"<<ui_position_source);//chq
    if(command_type == COMMAND_ACKERMANN){
      //
      // Publish through the node handle Ackerman type messages to the command vel topic
      vel_pub_ = private_node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>(cmd_topic_vel_, 1);
    }else{
      //
      // Publish through the node handle Twist type messages to the command vel topic
      vel_pub_ = private_node_handle_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
    }
    path_marker_pub = private_node_handle_.advertise<visualization_msgs::MarkerArray>("purepersuit_marker_path",1);
    tar_marker_pub = private_node_handle_.advertise<visualization_msgs::MarkerArray>("purepersuit_marker_target",1);

    path_pub_ = private_node_handle_.advertise<nav_msgs::Path>("purepersuit_path", 1);
    //
    if(ui_position_source == MAP_SOURCE)
      tranform_map_pub_ = private_node_handle_.advertise<geometry_msgs::TransformStamped>("map_location", 100);
    //status_pub_ = private_node_handle_.advertise<purepursuit_planner::ControllerStatus>("status", 1);
    odom_sub_ = private_node_handle_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &purepursuit_planner_node::OdomCallback, this );
    //cmd_vel_sub_ = private_node_handle_.subscribe<purepursuit_planner::AckermannDriveStamped>("command", 1, &purepursuit_planner_node::CmdVelCallback, this );

    // Diagnostics
    updater_diagnostic.setHardwareID("PurePursuit-Planner");
    // Topics freq control
    min_odom_freq = 5.0;
    max_odom_freq = 100.0;
    updater_diagnostic_odom = new diagnostic_updater::TopicDiagnostic(odom_topic_, updater_diagnostic,
              diagnostic_updater::FrequencyStatusParam(&min_odom_freq, &max_odom_freq, 0.1, 10),
              diagnostic_updater::TimeStampStatusParam(0.001, 0.1));


    // Action server
    action_server_goto.registerGoalCallback(boost::bind(&purepursuit_planner_node::GoalCB, this));
        action_server_goto.registerPreemptCallback(boost::bind(&purepursuit_planner_node::PreemptCB, this));
        
        // Services
        //sc_enable_front_laser_ = private_node_handle_.serviceClient<s3000_laser::enable_disable>(name_sc_enable_front_laser_);
        //sc_enable_back_laser_ = private_node_handle_.serviceClient<s3000_laser::enable_disable>(name_sc_enable_back_laser_);
        
    ROS_INFO("%s::ROSSetup(): odom_topic = %s, command_topic_vel = %s, position source = %s, desired_hz=%.1lf, min_lookahead = %.1lf, max_lookahead = %.1lf, kr = %.2lf, command_type = %s", sComponentName.c_str(), odom_topic_.c_str(),
     cmd_topic_vel_.c_str(), position_source_.c_str(), desired_freq_, d_lookahear_min_, d_lookahear_max_, Kr, s_command_type.c_str());

    //ROS_INFO("%s::ROSSetup(): laser_topics: front -> %s, back -> %s", sComponentName.c_str(), name_sc_enable_front_laser_.c_str(), name_sc_enable_back_laser_.c_str());

    //last_command_time = ros::Time::now();
  }
  /*!  \fn reconfigureCB(AMCLConfig &config, uint32_t level)
   *   \brief
  */
  void reconfigureCB(robotnik_pp_planner::robotnik_pp_plannerConfig &config, uint32_t level)
  {
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
    maxT = config.max_w;
    minT = config.min_w;
    Kp = config.kp_w;
    Ki = config.ki_w;
    Kd = config.kd_w;

    look_ahead_inc = config.look_ahead_inc;
    d_lookahear_min_ = config.d_lookahear_min;
    d_lookahear_max_ = config.d_lookahear_max;
    max_speed_ = config.max_speed;
    desired_freq_ = config.desired_freq;

    ROS_INFO("purepersuit.reconfigureCB.now reconfig rotate speed pid para:pid_kp:%.3f,pid_ki:%.3f,pid_kd:%.3f ",Kp,Ki,Kd);
    ROS_INFO("purepersuit.reconfigureCB.now reconfig trajectory track para:look_ahead_inc:%.3f,d_lookahear_min_:%.3f,d_lookahear_max_:%.3f,max_speed_:%.3f, desired_freq_",look_ahead_inc,d_lookahear_min_,d_lookahear_max_,max_speed_,desired_freq_);

  }
  /*!  \fn ReturnValue Setup()
   *   \brief
  */
  ReturnValue Setup(){
    // Checks if has been initialized
    if(bInitialized){
      ROS_INFO("purepursuit_planner::Setup: Already initialized");
      return INITIALIZED;
    }

    // Starts action server

    action_server_goto.start();

    bInitialized = true;
    ROS_INFO("purepursuit_planner::Setup.start action_server_goto done!");//chq
    return OK;
  }


  /*! \fn int ReadAndPublish()
   * Reads data a publish several info into different topics
  */
  int ReadAndPublish()
  {
    //updater_diagnostic_odom->tick(ros::Time::now());
    updater_diagnostic.update();
    return(0);
  }
  /*! \fn void pubMarkerRfs()
   * Pub the path to rviz
   * @author chq
  */
  void pubMarkerRfs(std::vector<Waypoint> new_points){
  visualization_msgs::Marker points_marker;
  visualization_msgs::Marker text_marker;
  nav_msgs::Path path_points;
  visualization_msgs::MarkerArray marker_array;
  points_marker.header.stamp = ros::Time::now();
  path_points.header.stamp = points_marker.header.stamp;
  std::string frame;
  if(position_source_ == "MAP")
    frame = "/map";
  else
    frame = "/odom";
  points_marker.header.frame_id = frame;
  path_points.header.frame_id = points_marker.header.frame_id;

  points_marker.type = visualization_msgs::Marker::POINTS;
  points_marker.ns = "map_nmspace";
  points_marker.action = visualization_msgs::Marker::ADD;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  points_marker.scale.x = 0.1;
  points_marker.scale.y = 0.1;
  points_marker.scale.z = 0.1;
  // Set the color -- be sure to set alpha to something non-zero!
  //DarkOrchid	153 50 204
  points_marker.color.r = 153;
  points_marker.color.g = 50;
  points_marker.color.b = 204;
  points_marker.color.a = 0.3;
  points_marker.lifetime = ros::Duration(100000);
  geometry_msgs::Point p;
  geometry_msgs::Pose pose;
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = path_points.header.frame_id;
  if(new_points.size() ){
    for(int i=0;i < new_points.size();i++){
      points_marker.id = i;
      //在相对于激光头的坐标空间
      p.x = new_points[i].dX;
      p.y = new_points[i].dY;
      p.z = 0;
      points_marker.points.push_back(p);

      pose.position.x = p.x;
      pose.position.y = p.y;
      pose.position.z = 0;
      pose_stamped.pose.position.x = p.x ;
      pose_stamped.pose.position.y = p.y ;
      pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(new_points[i].dA);
      path_points.poses.push_back(pose_stamped);

      ////fort text type,Only scale.z is used. scale.z specifies the height of an uppercase "A".
      text_marker = points_marker;
      text_marker.ns = "purepersuit_text_space";
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.color.r = 255;
      text_marker.color.g = 0;
      text_marker.color.b = 0;
      text_marker.color.a = 1;
      text_marker.scale.z = 0.1;
      text_marker.id = i;
      stringstream ss ;
      ss << i;
      string str;
      ss >> str;
      text_marker.text = str ;
      text_marker.pose = pose;
      marker_array.markers.push_back(text_marker);

    }
    marker_array.markers.push_back(points_marker);
    marker_array.markers.push_back(text_marker);
    path_marker_pub.publish(marker_array);
    path_pub_.publish(path_points);
  }


  }

  /*!  \fn ReturnValue Start()
   *   \brief Start Controller
  */
  ReturnValue Start(){

    if(bRunning){
      ROS_INFO("agvs_controller::Start: the component's thread is already running");
      return THREAD_RUNNING;
    }


    bRunning = true;
    return OK;
  }

  /*!  \fn ReturnValue Stop()
   *   \brief Stop Controller
  */
  ReturnValue Stop(){

    if(!bRunning){
      ROS_INFO("agvs_controller::Stop: Thread not running");

      return THREAD_NOT_RUNNING;
    }

    bRunning = false;

    return OK;
  }


  /*! \fn void ControlThread()
  */
  void ControlThread()
  {
    //ROS_INFO("purepursuit_planner.ControlThread() start...");
    ros::Rate r(desired_freq_);  // 50.0


    // while(node_handle_.ok()) {
    while(ros::ok()) {
      //ROS_INFO("purepursuit.ControlThread.start...");
      switch(iState){

        case INIT_STATE:
          InitState();
        break;

        case STANDBY_STATE:
          StandbyState();
        break;

        case READY_STATE:
          ReadyState();
        break;

        case SHUTDOWN_STATE:
          ShutDownState();
        break;

        case EMERGENCY_STATE:
          EmergencyState();
        break;

        case FAILURE_STATE:
          FailureState();
        break;

      }

      AllState();
      //ROS_INFO("purepursuit.ControlThread.end...");
      ros::spinOnce();
      r.sleep();
    }
    ShutDownState();

    //ROS_INFO("purepursuit_planner::ControlThread(): End");

  }


  /*!  \fn void InitState()
  */
  void InitState(){
    ROS_INFO("purepursuit_planner::InitSate:");
    if(bInitialized && bRunning){
      if(CheckOdomReceive() == 0){
        ROS_INFO("purepursuit_planner::InitState.suc to rec odom,do SwitchToState(STANDBY_STATE).");//chq
        SwitchToState(STANDBY_STATE);
      }
    }else{
      if(!bInitialized)
        Setup();
      if(!bRunning)
        Start();
    }

  }

  /*!  \fn void StandbyState()
  */
  void StandbyState(){
    if(CheckOdomReceive() == -1){
      ROS_WARN("StandbyState.CheckOdomReceive() == -1 doSwitchToState(EMERGENCY_STATE) ");//chq
      SwitchToState(EMERGENCY_STATE);
    }
    else{
      if(bEnabled && !bCancel ){

        if(pathCurrent.Size() > 0 || MergePath() == OK){
          ROS_INFO("%s::StandbyState: route available", sComponentName.c_str());
          SwitchToState(READY_STATE);
        }
      }

    }
  }

  /*!  \fn void ReadyState()
  */
  void ReadyState(){
    //ROS_INFO("purepersuit.ReadyState.start...");
    if(CheckOdomReceive() == -1){
      ROS_INFO("purepursuit_planner.ReadyState.CheckOdomReceive() == -1.do set speed = 0 SwitchToState(EMERGENCY_STATE),return!");
      SetRobotSpeed(0.0, 0.0);
      SwitchToState(EMERGENCY_STATE);
      return;
    }
    if(!bEnabled){
      ROS_INFO("%s::ReadyState: Motion is disabled", sComponentName.c_str());
      SetRobotSpeed(0.0, 0.0);
      SwitchToState(STANDBY_STATE);
      return;
    }
    if(bCancel){
      ROS_INFO("%s::ReadyState: Cancel requested", sComponentName.c_str());
      SetRobotSpeed(0.0, 0.0);
      SwitchToState(STANDBY_STATE);
      return;
    }
    //ROS_INFO("purepursuit_planner.ReadyState.do PurePursuit...");

    int ret = PurePursuit();
    static int cnt = 0;
    if(cnt++ > 100){
      ROS_INFO("purepursuit_planner.ReadyState.do PurePursuit done.ret(0 controlling,-1 error,1 finished):%d",ret);
      cnt = 0;
    }

    if(ret == -1){
      ROS_ERROR("%s::ReadyState: Error on PurePursuit", sComponentName.c_str());
      bCancel = true;  //Activates the flag to cancel the mision
      SetRobotSpeed(0.0, 0.0);
      goto_result.route_result = -1;
      goto_feedback.percent_complete = 100.0;  // Set the percent to 100% to complete the action

      SwitchToState(STANDBY_STATE);

    }else if(ret == 1){
      ROS_INFO("%s::ReadyState: Route finished", sComponentName.c_str());
      SetRobotSpeed(0.0, 0.0);
      goto_result.route_result = 0;
      goto_feedback.percent_complete = 100.0;  // Set the percent to 100% to complete the action
      SwitchToState(STANDBY_STATE);
    }
    // We have to update the percent while the mision is ongoing
   //ROS_INFO("purepersuit.ReadyState.end...");
  }


  /*! \fn void UpdateLookAhead()
  *   \brief Updates (little by little) the variable lookahead depending of the current velocity
  */
  void UpdateLookAhead(){
    double aux_lookahead = fabs(dLinearSpeed);
    double desired_lookahead = 0.0;
    //double inc = 0.01;  // incremento del lookahead
    ///期望的前瞻距离与速度有关，一般是速度大小值，超过边界等于边界
    if(aux_lookahead < d_lookahear_min_)
      desired_lookahead = d_lookahear_min_;
    else if(aux_lookahead > d_lookahear_max_)
      desired_lookahead = d_lookahear_max_;
    else{
      desired_lookahead = aux_lookahead;
    }
    ///如果期望与现状差值超过0.001 则将现状加inc
    ///速度突变，dLookAhead非突变，而是逐渐改变，这个对于跟踪平滑较为重要，当然跟踪越平滑，可能跟踪误差就越大
    if((desired_lookahead - 0.001) > dLookAhead){
      dLookAhead += look_ahead_inc;
    }else if((desired_lookahead + 0.001) < dLookAhead)
      dLookAhead -= look_ahead_inc;
  }


  /*! \fn double Dot2( double x1, double y1, double x2, double y2)
  *   \brief Obtains vector cross product w x v
  *   \return w.x * v.x + w.y * w.y
  */
  double Dot2( double x1, double y1, double x2, double y2) {
    //在点积运算中，第一个向量投影到第二个向量上
    //点乘的结果就是两个向量的模相乘,然后再与这两个向量的夹角的余弦值相乘.或者说是两个向量的各个分量分别相乘的结果的和
    //如果点乘的结果为0,那么这两个向量互相垂直；如果结果大于0,那么这两个向量的夹角小于90度；如果结果小于0,那么这两个向量的夹角大于90度
    return (x1*x2 + y1*y2); // cross product
}

  /*! \fn double Dist(double x1, double y1, double x2, double y2)
  *   \brief obtains distance between points p1 and p2
  */
  double Dist(double x1, double y1, double x2, double y2) {
    double diff_x = (x2 - x1);
    double diff_y = (y2 - y1);
    return sqrt( diff_x*diff_x + diff_y*diff_y );
  }

  /*! \fn double DistP2S( Odometry current_position, Waypoint s0, Waypoint s1, Waypoint *Pb)
   *  \brief obtains distance between the current position and segment s0->s1, and returns the point
   *  Return: the shortest distance from p to s (utm points) and the point
   *  of the segment that gives the shortest distance
  */
  double DistP2S( geometry_msgs::Pose2D current_position, Waypoint s0, Waypoint s1, Waypoint *Pb){
    double vx,vy, wx, wy;

    double c1, c2, di, b;

    vx = s1.dX - s0.dX;
    vy = s1.dY - s0.dY;

    wx = current_position.x - s0.dX;
    wy = current_position.y - s0.dY;

    c1 = Dot2( wx, wy, vx, vy );
    //如果投影点在线段后方
    //
    //   \.cur_pos
    //    \
    //     \di
    //      \
    //       \
    //       s0---s1
    if ( c1 <= 0.0 ) {
      di = Dist(current_position.x, current_position.y, s0.dX, s0.dY);
      Pb->dX = s0.dX;
      Pb->dY = s0.dY;
      return di;
    }

    c2 = Dot2(vx,vy, vx, vy);
    //如果投影点在线段前方
    //     /.cur_pos
    //    / |
    //   /di|
    //  /   |
    // /    |
    //s0-s1-|
    if ( c2 <= c1 ) {
      //printf("kanban::DistP2S: c2 <= c1\n");
      di = Dist(current_position.x, current_position.y, s1.dX, s1.dY);
      Pb->dX = s1.dX;
      Pb->dY = s1.dY;
      return di;
    }
    //如果投影点在线段上
    //    .cur_pos
    //   /|
    //  / |di
    // /  |
    //s0------s1
    b = c1 / c2;
    Pb->dX = s0.dX + b * vx;
    Pb->dY = s0.dY + b * vy;

    di = Dist(current_position.x, current_position.y, Pb->dX, Pb->dY);

    return di;
  }
  ///从第一个点逐一执行路径而不跳跃执行
ReturnValue PointOneByOne(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp) {
  int i=0,j=0,k=0;
  static int last_i=0;
  double dmin, d, d0, d1, d2,d3, *d_seg;
  double t;
  geometry_msgs::Pose2D target_position;
  Waypoint s0, s1, Pb, Pb1;

  int size = pathCurrent.NumOfWaypoints();
  d_seg = new double[size]; //array con la distancia entre puntos consecutivos en la ruta
  i = pathCurrent.GetCurrentWaypointIndex();
  ROS_INFO("PointOneByOne.cur way index :%d,comp_firstpoint(1==true):%d",i,comp_firstpoint);
  /// 3-Find cur segment
  if( pathCurrent.GetCurrentWaypoint(&s0) != OK ){
    ROS_ERROR("%s::PointOneByOne: Error getting cur waypoint %d", sComponentName.c_str(), j);
    return ERROR;
  }
  if( pathCurrent.GetNextWaypoint(&s1) != OK ){
    ROS_ERROR("%s::PointOneByOne: Error getting next waypoint %d", sComponentName.c_str(), j);
    return ERROR;
  }
  d0 = Dist(s0.dX, s0.dY, s1.dX, s1.dY);        //dist bet s0 and s1
  d  = DistP2S(current_position, s0, s1, &Pb1);  //Pb1 closest point on segment
  d1 = Dist(Pb1.dX, Pb1.dY, s1.dX, s1.dY);      //dist bet pb1 and s1
  d2 = Dist(s0.dX, s0.dY, Pb1.dX, Pb1.dY);        //dist bet s0 and P
  d3 = Dist(current_position.x, current_position.y, s1.dX, s1.dY);        //dist bet curpos and s1
  /*double vx,vy, wx, wy;
  wx = s0.dX - current_position.x ;
  wy = s0.dY - current_position.y ;
  double rela_a = atan2(wy,wx)-current_position.theta;
  radnorm(&rela_a);
  */
  ///ROS_INFO("pointdlh.cur dmin:%.3f, comp_firstpoint(0==false,1=yes):%d,g_dir(0==back):%d,need_repub_path:%d",dmin,comp_firstpoint,g_dir,need_repub_path);
  if(  d > d_lookvertical_min_ && !comp_firstpoint)///must ahead to first tar + min dist + not comp
  {
    //double d0 = Dist(Pb.dX, Pb.dY, s0.dX, s0.dY);
    double gradient = atan2(s0.dY-s1.dY,s0.dX-s1.dX);//reverse dir
    target_position.x = s0.dX + dLookAhead*exp(-1.0/d)*cos(gradient);//无穷远处瞄准起点后方lookahead处，靠近瞄准起始点(即始终瞄准目标点后方)
    target_position.y = s0.dY + dLookAhead*exp(-1.0/d)*sin(gradient);
    double angle_segment = atan2(target_position.y - current_position.y, target_position.x - current_position.x);
    radnorm(&angle_segment);
    target_position.theta = angle_segment;
    *wp = target_position;
    delete d_seg;
    g_dir = look_back;
    return OK;
  }
  else{
    g_dir = lool_ahead;
    if( dmin <= d_lookvertical_min_ )
      comp_firstpoint = true;
  }

  double angle_segment = atan2(s1.dY - s0.dY, s1.dX - s0.dX);
  radnorm(&angle_segment);
//  d0 = Dist(s0.dX, s0.dY, s1.dX, s1.dY);        //dist bet s0 and s1
//  d  = DistP2S(current_position, s0, s1, &Pb1);  //Pb1 closest point on segment
//  d1 = Dist(Pb1.dX, Pb1.dY, s1.dX, s1.dY);      //dist bet pb1 and s1
//  d2 = Dist(s0.dX, s0.dY, Pb1.dX, Pb1.dY);        //dist bet s0 and P

  ///method lookahead_tar = dist_curpos2tar;
  /// 这种方式跟踪更平滑但是误差相对下一种较差，但是不容易震荡
  //  .----
  //   \  lookahead
  //    \___
  //s0-----s1
  //
  if( d3 <= dLookAhead ){
    //change tar
   k = pathCurrent.GetCurrentWaypointIndex()+1;
   if( k < size-1 ){
    j= k;
    ///update target
    ROS_INFO("PointOneByOne.change to next target------------------index:%d",j);
    // Sets the waypoint where the robot is at the moment
    if(pathCurrent.SetCurrentWaypoint(j) == ERROR){
      ROS_ERROR("%s::PointOneByOne: Error setting current waypoint to %d", sComponentName.c_str(), j);
      return ERROR;
    }

   }
   ///不管是到达终点还是到达线段端点，also指定为end点
   //else{
     target_position.x = s1.dX;
     target_position.y = s1.dY;
     target_position.theta = angle_segment;
     *wp = target_position;
     delete d_seg;
     return OK;
  // }
  }
  else if( d >= dLookAhead )//toward to pb1
  {
    target_position.x = Pb1.dX;
    target_position.y = Pb1.dY;
    target_position.theta = angle_segment;
    *wp = target_position;
    delete d_seg;
    ROS_INFO("PointOneByOne.d(%.6f) >= dLookAhead(%.6f).track p",d,dLookAhead);
    return OK;
  }
  else{
      if( d2 <= 0 ){///d2 s0TOpb
        VecPosition p1(s0.dX,s0.dY);
        VecPosition p2(s1.dX,s1.dY);
        VecPosition cur_p(current_position.x,current_position.y);
        Line l = Line::makeLineFromTwoPoints( p1, p2 );
        double d_close = l.getDistanceWithPoint(cur_p);
        double head_dist1 = sqrt(pow(dLookAhead,2)-pow(d_close,2));
        double head_dist2 = sqrt(pow(d,2)-pow(d_close,2));
        double dist = head_dist1 - head_dist2;
        if( d0 > 1e-5 )
          t = dist/d0; //chq
        else
          t = 0;
        ROS_INFO("PointOneByOne.at back of s0.dist:%.6f",dist);
      }
      else
      {
        double head_dist1 = sqrt(pow(dLookAhead,2)-pow(d,2));
        double dist = d2+head_dist1 ;
        if( d0 > 1e-5 )
          t = dist/d0; //chq
        else
          t = 0;
        ROS_INFO("PointOneByOne.at up or down of s0.dist:%.6f",dist);
      }

      target_position.x = s0.dX + ( s1.dX - s0.dX )*t;
      target_position.y = s0.dY + ( s1.dY - s0.dY )*t;
      target_position.theta = angle_segment;
      *wp = target_position;
      delete d_seg;
      return OK;
    }





///method lookahead = dist_curpos2p+dist_p2tar
///这种方式跟踪精度较好，但是速度高的时候容易震荡
#if 0
  if( d >= dLookAhead )//toward to pb1
  {
    target_position.x = Pb1.dX;
    target_position.y = Pb1.dY;
    target_position.theta = angle_segment;
    *wp = target_position;
    delete d_seg;
    ROS_INFO("PointOneByOne.d(%.6f) >= dLookAhead(%.6f).track p",d,dLookAhead);
    return OK;
  }
  else{
    ///1-if track end then end this seg
    if( d+d1 < dLookAhead ){
      //change tar
     k = pathCurrent.GetCurrentWaypointIndex()+1;
     if( k < size-1 ){
      j= k;
      ///update target
      ROS_INFO("PointOneByOne.change to next target------------------index:%d",j);
      // Sets the waypoint where the robot is at the moment
      if(pathCurrent.SetCurrentWaypoint(j) == ERROR){
        ROS_ERROR("%s::PointOneByOne: Error setting current waypoint to %d", sComponentName.c_str(), j);
        return ERROR;
      }
     }
   //  else{
       target_position.x = s1.dX;
       target_position.y = s1.dY;
       target_position.theta = angle_segment;
       *wp = target_position;
       delete d_seg;
       return OK;
  //   }
    }
    //otheewise
    double dist=0.;
    dist = d2+(dLookAhead-d);
    if( d0 > 1e-5 )
      t = dist/d0; //chq
    else
      t = 0;
    //if(d >= dLookAhead ){//toward to s0-s1
      /*if( d0 > 1e-5 )
        t = 1 - (d-dLookAhead)/d0; //chq
      else
        t = 0;
        */

      //if at back
      //.(cur)    |
      //  \       |
      //   \ |dist|
      //    (P)s0---------s1
      // lh = curpos2s0 + dist
      // => dist  = lh-curpos2s0
      if( d2<=0 ){
        ROS_INFO("PointOneByOne.track s0-s1.at back to s0.dLookAhead:%.3f,dist(s0totar):%.3f,d0(s02s1):%.3f,t:%.6f",dLookAhead,dist,d0,t);
      }
      //if the p is bet s0 and s1
      //       .
      //       |
      //   |   |dist |
      //  s0---P---(tar)--s1
      //  lh = curpos2p+dist
      //  dist_s02tar = s02p+dist
      //=>dist_s02tar = s02p+lh-curpos2p
      else{
        ROS_INFO("PointOneByOne.track s0-s1 up or down to seg.dLookAhead:%.3f,dist(s02tar):%.3f,d0(s02s1):%.3f,d2(s02p):%6.f,t:%.6f",dLookAhead,dist,d0,t);
      }
      target_position.x = s0.dX + ( s1.dX - s0.dX )*t;
      target_position.y = s0.dY + ( s1.dY - s0.dY )*t;
      target_position.theta = angle_segment;
      *wp = target_position;
      delete d_seg;
      return OK;
    //}
  }
//}

#endif
#if 0
       ROS_INFO("PointOneByOne.d0(s02s1):%.6f,d1(p2s2):%.6f,d2(s0_2p):%.6f,d(cur2p):%.6f",d0,d1,d2,d);
      if( d1 > dLookAhead ){
        double dist = d2+dLookAhead;
        if( d0 > 1e-5 )
          t = dist/d0; //chq
        else
          t = 0;
        ROS_INFO("PointOneByOne.track s0-s1.dLookAhead:%.3f,dist(s02tar):%.3f,d0(s02s1):%.3f,d2(s02p):%6.f,t:%.6f",dLookAhead,dist,d0,t);

        target_position.x = s0.dX + ( s1.dX - s0.dX )*t;
        target_position.y = s0.dY + ( s1.dY - s0.dY )*t;
        target_position.theta = angle_segment;
        *wp = target_position;
        delete d_seg;
        return OK;
       }
      else{
        //change tar
       k = pathCurrent.GetCurrentWaypointIndex()+1;
       if( k < size-1 ){
        j= k;
        ///update target
        ROS_INFO("PointOneByOne.change to next target------------------index:%d",j);
        // Sets the waypoint where the robot is at the moment
        if(pathCurrent.SetCurrentWaypoint(j) == ERROR){
          ROS_ERROR("%s::PointOneByOne: Error setting current waypoint to %d", sComponentName.c_str(), j);
          return ERROR;
        }
       }
      // else{
         target_position.x = s1.dX;
         target_position.y = s1.dY;
         target_position.theta = angle_segment;
         *wp = target_position;
         delete d_seg;
         return OK;
      // }
      }
#endif



}

  /*! \fn ReturnValue PointDlh(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp  )
   *  \brief Returns a point in a distance dlookahead on the path
   *  \brief 0,跟踪的不是给定的目标点　而是由给定的路径点构成的线段上的动态形成点，由lookahead推动点移动，这样跟踪更平滑!!!
   *  \brief 1,找到距离当前位置最近的线段索引k，并设为最新的跟踪索引
   *  \brief 2,沿着索引k线段向前累计线段距离，直到加到索引k'线段(长度l)，sum距离为s,超过dLookAhead
   *  \brief 3,last跟踪ｋ’上的哪一点，取决于k'长度l在s中，超过dLookAhead部分所占的比重，比重逐渐增大，跟踪点逐渐由k'起点趋向于k'末尾点
   *  \return OK
   *  \return ERROR
  */
  ReturnValue PointDlh(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp) {
    int i,j=0,k;
    double dmin, d, d0, d1, d2, *d_seg;
    double t;
    geometry_msgs::Pose2D target_position;
    Waypoint s0, s1, Pb, Pb1;
    static bool comp_firstpoint = false;
    static bool need_repub_path = true;
    int size = pathCurrent.NumOfWaypoints();

    d_seg = new double[size]; //array con la distancia entre puntos consecutivos en la ruta

    /// 1- Find closest segment
    /// 1 寻找最近线段
    dmin = 100000000;
    //递推找到所有路径点，两两构成的线段中，距离当前车子位置最近距离的线段和垂点
    i = pathCurrent.GetCurrentWaypointIndex();
    ROS_INFO("pointdlh.cur way index :%d,need_repub_path(1==true):%d",i,need_repub_path);
    if( i == 0 ){
      if(need_repub_path){
        comp_firstpoint = false;///reset when get new targets
        need_repub_path = false;
      }
    }
    else{
      need_repub_path = true;
    }

    for(; i < (size - 1); i++) {
      if( (pathCurrent.GetWaypoint(i, &s0) == OK) &&  (pathCurrent.GetWaypoint(i+1, &s1) == OK) ){
        d_seg[i] = Dist(s0.dX, s0.dY, s1.dX, s1.dY);
        ///Pb1 当前位置到线段上的最近点（s0-s1之间，不是垂足!!!）
        d = DistP2S(current_position, s0, s1, &Pb1);    // Pb1 closest point on segment

        if (d < dmin) {
          Pb.dX = Pb1.dX;  // not the same as Pb=Pb1 !
          Pb.dY = Pb1.dY;
          dmin = d;
          j = i;      // j : index to closest segment
        }
        //ROS_INFO("PointDlh. Distance to segment %d(%.2lf, %2.lf)->%d(%.2lf, %2.lf) = %.3lf,point (%.3lf, %.3lf) (DMIN = %.3lf)", i,
        //s0.dX, s0.dY, i+1, s1.dX, s1.dY, d, Pb1.dX, Pb1.dY, dmin);
      }else{
        ROS_ERROR("%s::PointDlh: Error Getting waypoints", sComponentName.c_str());
        return ERROR;
      }
    }

    //ROS_INFO("PointDlh:: Current waypoint index %d, next %d",pathCurrent.GetCurrentWaypointIndex(), j);

    // Si cambiamos de waypoint
    // 如果当前距离最近的线段不再是历史track point，更新
    if(pathCurrent.GetCurrentWaypointIndex() != j){
      //add by chq for reset process
      comp_firstpoint = false;///reset when change target
      // Sets the waypoint where the robot is at the moment
      if(pathCurrent.SetCurrentWaypoint(j) == ERROR){
        ROS_ERROR("%s::PointDlh: Error setting current waypoint to %d", sComponentName.c_str(), j);
        return ERROR;
      }else{ // OK
        //ROS_INFO("PointDlh:: Changing waypoint to %d", j);
        if(j == (size - 2)){  // Penultimo waypoint
          Waypoint w_last, w_before_last;
          pathCurrent.GetCurrentWaypoint(&w_before_last);  // Penultimo waypoint
          pathCurrent.BackWaypoint(&w_last);        // Ultimo waypoint
          // Distancia maxima = distancia entre el punto actual y el penultimo punto + más la distancia entre los dos ultimos puntos + un valor constante
          //dMaxDistance = Dist(w_before_last.dX, w_before_last.dY, odomWhenLastWaypoint.px, odomWhenLastWaypoint.py) + Dist(w_last.dX, w_last.dY, w_before_last.dX, w_before_last.dY) + 0.1;
          //ROS_INFO("%s::PointDlh: Penultimo punto. Robot en (%.3lf, %.3lf, %.3lf). Distancia máxima a recorrer = %.3lf m ", sComponentName.c_str(), odomWhenLastWaypoint.px, odomWhenLastWaypoint.py, odomWhenLastWaypoint.pa, dMaxDistance);
        }

      }
    }

    /// 2-Find cur segment add by chq to cal s0
    if( pathCurrent.GetCurrentWaypoint(&s0) != OK ){
      ROS_ERROR("%s::PointDlh: Error getting cur waypoint %d", sComponentName.c_str(), j);
      return ERROR;
    }

    ///2 获取最新的所跟踪的线段
    /// 2-Find segment ahead in dlookahead
    if( pathCurrent.GetNextWaypoint(&s1) != OK ){
      ROS_ERROR("%s::PointDlh: Error getting next waypoint %d", sComponentName.c_str(), j);
      return ERROR;
    }
    //d0 = Dist(Pb.dX, Pb.dY, s0.dX, s0.dY);
    d1 = Dist(Pb.dX, Pb.dY, s1.dX, s1.dY);

    k = j;              // k : index of D_LOOKAHEAD point segment
    ///!!!added by chq for move fast to segment line ---start
    // if dist to line is large then we treat the Pb as the target
    // 这种方式可以更好的进入目标点
    ROS_INFO("pointdlh.cur dmin:%.3f, comp_firstpoint(0==false,1=yes):%d,g_dir(0==back):%d,need_repub_path:%d",dmin,comp_firstpoint,g_dir,need_repub_path);
    if( dmin > d_lookvertical_min_ && !comp_firstpoint)
    {
      //double d0 = Dist(Pb.dX, Pb.dY, s0.dX, s0.dY);
      double gradient = atan2(s0.dY-s1.dY,s0.dX-s1.dX);//reverse dir
      target_position.x = s0.dX + dLookAhead*exp(-1.0/dmin)*cos(gradient);//无穷远处瞄准起点后方lookahead处，靠近瞄准起始点(即始终瞄准目标点后方)
      target_position.y = s0.dY + dLookAhead*exp(-1.0/dmin)*sin(gradient);
      double angle_segment = atan2(target_position.y - current_position.y, target_position.x - current_position.x);
      radnorm(&angle_segment);
      target_position.theta = angle_segment;
      *wp = target_position;
      delete d_seg;
      g_dir = look_back;
      return OK;
    }
    else{
      g_dir = lool_ahead;
      if( dmin <= d_lookvertical_min_ )
        comp_firstpoint = true;
    }
    ///!!!!added by chq for move fast to segment line ---end
    //如果从垂点到垂点所在线段的末尾距离不超过dLookAhead，则持续沿着前向线段的距离累加，直到超过之。
    while ( (d1 < dLookAhead) && ( (k+1) < (size - 1) ) ) {
      // searched point on this segment
      k = k + 1;
      d1 = d1 + d_seg[k];
    }
    /// 累加的最后一条线段k，其在超过dLookAhead所占的分值比比重，决定了偏向于跟踪k线段的起点还是末尾.比重越大，越倾向于跟踪末尾
    /// 3- Obtain t parameter in the segment
    d2 = ( d1 - dLookAhead );       // t parameter of segment k
    if(d_seg[k]>1e-5)
      t = (d_seg[k] - d2) / d_seg[k]; // Pendiente avoid div/0. En teoria no puede producirse pq dos waypoints consecutivos serán diferentes
    ///add by chq start---
    else
      t = 0;
    if(t>1)t=1;
    if(t<0)t=0;
    ///add by chq end---
    ///t = 1 - (d1-lookahead)/seg[k] //chq
    ROS_INFO("pointdlh.para d1:%.6f, dLookAhead:%.6f, t :%.6f",d1,dLookAhead,t);
    // 4- Obtain point with t parameter
    if( (pathCurrent.GetWaypoint(k, &s0) == OK) && (pathCurrent.GetWaypoint(k + 1, &s1) == OK) ) {
      ///chq seg[k]在累计和超过dLookAhead距离中占的比重越大，越倾向于参考ｋ末尾点。否则越偏向于参考看ｋ起始点
      target_position.x = s0.dX + ( s1.dX - s0.dX )*t;
      target_position.y = s0.dY + ( s1.dY - s0.dY )*t;
      double angle_segment = atan2(s1.dY - s0.dY, s1.dX - s0.dX);

      radnorm(&angle_segment);

      target_position.theta = angle_segment;
      *wp = target_position;

      delete d_seg;

      return OK;
    }else{
      ROS_ERROR("%s::PointDlh: Error getting next waypoint %d", sComponentName.c_str(), j);
      return ERROR;
    }

  }
  /*!  \fn void pubTarget()
   * \brief visualize the target and the rotation speed for debugging
   * \param current_position cur robot pos
   * \param next_position target pos
   * \param w rotate angle(speed)
   */
void pubTarget(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D next_position,double w){
 visualization_msgs::Marker points_marker;
 visualization_msgs::Marker dir_marker;
 visualization_msgs::MarkerArray marker_array;
 points_marker.header.stamp = ros::Time::now();
 dir_marker.header.stamp = points_marker.header.stamp;
 std::string frame;
 if(position_source_ == "MAP")
   frame = "/map";
 else
   frame = "/odom";
 points_marker.header.frame_id = frame;
 dir_marker.header.frame_id = points_marker.header.frame_id;

 points_marker.type = visualization_msgs::Marker::POINTS;
 points_marker.ns = "purepersuit_space";
 points_marker.action = visualization_msgs::Marker::ADD;
 // Set the scale of the marker -- 1x1x1 here means 1m on a side
 points_marker.scale.x = 0.2;
 points_marker.scale.y = 0.2;
 points_marker.scale.z = 0.2;
 // Set the color -- be sure to set alpha to something non-zero!
 //DarkOrchid	153 50 204

 if( g_dir == look_back){
   points_marker.color.r = 0.8;
   points_marker.color.g = 0;
   points_marker.color.b = 0;
   points_marker.color.a = 0.5;
 }
 else{
   points_marker.color.r = 0.0;
   points_marker.color.g = 0.8;
   points_marker.color.b = 0.0;
   points_marker.color.a = 0.5;
 }

 points_marker.lifetime = ros::Duration(100000);
 points_marker.id = 0;
 geometry_msgs::Point p;
 p.x = next_position.x;
 p.y = next_position.y;
 p.z = 1;
 points_marker.points.push_back(p);


 dir_marker.type = visualization_msgs::Marker::ARROW;
 dir_marker.ns = "purepersuit_space";
 dir_marker.action = visualization_msgs::Marker::ADD;
 // Set the scale of the marker -- 1x1x1 here means 1m on a side
 //if is arrow scale.x is the shaft diameter, and scale.y is the head diameter.
 //If scale.z is not zero, it specifies the head length.
 dir_marker.scale.x = 0.1;
 dir_marker.scale.y = 0.1;
 dir_marker.scale.z = 0.05;
 // Set the color -- be sure to set alpha to something non-zero!
 //DarkOrchid	153 50 204
 dir_marker.color.r = 153;
 dir_marker.color.g = 50;
 dir_marker.color.b = 204;
 dir_marker.color.a = 0.3;
 dir_marker.lifetime = ros::Duration(100000);
 ///do not euqal to last points_marker id!!!
 dir_marker.id = 1;

 p.x = current_position.x;
 p.y = current_position.y;
 p.z = 0;
 //The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end.
 dir_marker.points.push_back(p);

 float len = 0.5;//arrow len
 float abs_angle = current_position.theta+w;
 p.x = current_position.x+cos(abs_angle)*len;
 p.y = current_position.y+sin(abs_angle)*len;
 p.z = 0;
 dir_marker.points.push_back(p);

 marker_array.markers.push_back(points_marker);
 marker_array.markers.push_back(dir_marker);
 tar_marker_pub.publish(marker_array);

}
  /*!  \fn int PurePursuit()
   * \brief High level control loop in cartesian coordinates
   * obtains desiredSpeedMps and desiredPhiEffort according to
   * the robot location and the path defined by the waypoints
   *  \return 0 if the iteration is OK
   *  \return -1 if there's a problem
   *  \return 1 if the route finishes
   */
  int PurePursuit(){
    //ROS_INFO("purepursuit.PurePursuit start...");
    double dx, dy, x1, y1;
    double curv, yaw;
    double wref;//, epw, uw;
    //double d = D_WHEEL_ROBOT_CENTER;   // Length in m (equiv to curv radius)
    double Kd = 1.1; // don't increase! 250
    Waypoint last_waypoint, next_waypoint;

    double dAuxSpeed = 0.0;
    double dth;
    double aux = 0.0, dDistCovered = 0.0;
    int ret = 0;

    geometry_msgs::Pose2D current_position = this->pose2d_robot;
    geometry_msgs::Pose2D next_position;
    ///disabled by chq
    //if(pathCurrent.NumOfWaypoints() < 2)  {
    if(pathCurrent.NumOfWaypoints() < 4)  {
      ROS_ERROR("%s::PurePursuit:points num:%d,below to 4, not enought waypoints",pathCurrent.NumOfWaypoints(), sComponentName.c_str());
      return -1 ;
    }


    yaw = current_position.theta;

    ///根据速度大小更新前瞻距离actively!!!
    ///
    //Updates the lookahead depending of the current velocity
    UpdateLookAhead();

    //
    // Get next point in cartesian coordinates
    if(PointOneByOne(current_position, &next_position) != OK)
    //if(PointDlh(current_position, &next_position) != OK)
    {
      ROS_ERROR("%s::PurePursuit: Error getting next point in the route", sComponentName.c_str());
      return -1;
    }
    ///chq 绝对坐标系旋转到车子参考系 to cal relative pos
    // Curvature 曲率
    dx = current_position.x - next_position.x;
    dy = current_position.y - next_position.y;
    //rotate to robot self cord
    ///chq newx = R * rot(-cur_theta)_dx = Rcos(theta - cur_pos_theta) = R*(cos t * cos c + sin t * sin c) = dx * cos c + dy * sin c
    x1 = cos(yaw)*dx + sin(yaw)*dy; //Original
    y1 = -sin(yaw)*dx + cos(yaw)*dy;
    //这个公式不难推导，就是以当前点和目标点构成的圆弧，求解旋转半径或者转向速度
    if ((x1*x1 + y1*y1) == 0)
      curv = 0;
    else
      curv = (2.0 / (x1*x1 + y1*y1)) * -y1;      //Original

    // Obtenemos alfa_ref en bucle abierto segun curvatura
    ///对于舵轮车要考虑转向轮到车子中心的前后轴距，计算的偏转角是转向轮的旋转角
    ///而对于差速，直接计算的到的偏转角就为角速度
    /// wref = atan(d_dist_wheel_to_center_/(1.0/curv));
    ///wref = atan2(-y1,-x1); //因为是差速，这里没有前后轮轴距 chq
    ///wref = 2*atan2(-y1,-x1); //两倍代表偏差是到目标点的旋转角度变化，而不是一半（到目标的连线与当前朝向的夹角） chq

    if(pathCurrent.BackWaypoint(&last_waypoint) == ERROR){
      ROS_ERROR("%s::PurePursuit: Error getting the last point in the path", sComponentName.c_str());
      return -1;
    }

    double dAuxDist = Dist(current_position.x, current_position.y, last_waypoint.dX, last_waypoint.dY);  //dist(waypoints.back().pos, current_position);

    if(pathCurrent.GetNextWaypoint(&next_waypoint) == ERROR){
      ROS_ERROR("%s::PurePursuit: Error getting next waypoint in the path", sComponentName.c_str());
      return -1;
    }

    dAuxSpeed = next_waypoint.dSpeed;
    ///wref = dAuxSpeed * curv;
    if (dAuxSpeed >= 0)
       dth = next_position.theta - current_position.theta;
    else
       dth = -(next_position.theta + Pi - current_position.theta);

    // normalize
    radnorm(&dth);
    double aux_wref = wref;
    ///wref += Kr * dth;
     double wref0 = wref;
    //ROS_INFO("PID control.pre_error bef theta control:%.6f ",pidTheta->preError());
    //wref = pidTheta->calculate(0, -wref);
    ///ROS_INFO("PID control.pre_error after theta control:%.6f ",pidTheta->preError());
    ///pubTarget(current_position,next_position,wref);//added by chq
    //ROS_INFO("cur orientation %f, Angle Error: %f , pid w: %f",current_position.theta,wref0, wref);

    ///ROS_INFO("Purepursuit: current pos (%.2lf, %.2lf), next pos (%.2lf, %.2lf), lookahead %.2lf, yaw = %.3lf, curv = %.3lf, dth = %.3lf, wref = %.3lf(%.3lf), speed=%.3lf", current_position.x, current_position.y, next_position.x, next_position.y, dLookAhead, yaw, curv, dth, wref, aux_wref, dAuxSpeed);
    //ROS_INFO("Purepursuit: yaw = %.3lf, curv = %.3lf, dth = %.3lf, wref = %.3lf", yaw, curv, dth, wref);


    ////////////////// Sets the speed depending of distance or speed restrictions /////////
    ///////////////////////////////////////////////////////////////////////////////////////
    // Controls the max allowed using first restriction
    ///限定最大速度
    if(fabs(dAuxSpeed) > max_speed_){
      if(dAuxSpeed > 0)
        dAuxSpeed = max_speed_;
      else
        dAuxSpeed = -max_speed_;
    }

    ///接近最后一个点的距离分两级，modify速度也分两级
    if(dAuxDist <= AGVS_SECOND_DECELERATION_DISTANCE)  {
      if( (dAuxSpeed < 0.0) && (dAuxSpeed < -AGVS_SECOND_DECELERATION_MAXSPEED) )
        dAuxSpeed = -AGVS_SECOND_DECELERATION_MAXSPEED;
      else if( (dAuxSpeed > 0.0) && (dAuxSpeed > AGVS_SECOND_DECELERATION_MAXSPEED) )
        dAuxSpeed = AGVS_SECOND_DECELERATION_MAXSPEED;

    }else if(dAuxDist <= AGVS_FIRST_DECELERATION_DISTANCE) {
      if( (dAuxSpeed < 0.0) && (dAuxSpeed < AGVS_FIRST_DECELERATION_MAXSPEED))
        dAuxSpeed = -AGVS_FIRST_DECELERATION_MAXSPEED;
      else if( (dAuxSpeed > 0.0) && (dAuxSpeed > AGVS_FIRST_DECELERATION_MAXSPEED) )
        dAuxSpeed = AGVS_FIRST_DECELERATION_MAXSPEED;
    }
    //如果在后方，按照方式１舵轮转向方式，转向，否则按照固定旋转半径转弯
    //if(x1 > 0 ){
    //if not arrive first point the use method 1
    ///第一个点要求快速到达，方法一更合适（如果初始位置离第一个点很远，使用方法２得到的转弯半径可能很远，需要很久才能到达）
    if(pathCurrent.GetCurrentWaypointIndex()==0 ){
      wref = 2 * atan2(-y1,-x1);
      ROS_INFO("-------------purepersuit.using turn directly------------------------");
    }
    else
      //wref = dAuxSpeed * curv;
      wref = 2*atan2(-y1,-x1);
      //wref = atan(d_dist_wheel_to_center_/(1.0/curv));


    wref += Kr * dth;
    wref = pidTheta->calculate(0, -wref);
    pubTarget(current_position,next_position,wref);//added by chq
    ROS_INFO("Purepursuit: current pos (%.2lf, %.2lf), next pos (%.2lf, %.2lf), lookahead %.2lf, yaw = %.3lf, curv = %.3lf, dth = %.3lf, wref(+dth) = %.6lf(%.6lf(raw)), speed=%.3lf", current_position.x, current_position.y, next_position.x, next_position.y, dLookAhead, yaw, curv, dth, wref, aux_wref, dAuxSpeed);

    //ROS_INFO("purepersuit.after mod, the speed aux:%.3f",dAuxSpeed);
    if(command_type == COMMAND_ACKERMANN){
      SetRobotSpeed(dAuxSpeed, wref);
    }else{
      SetRobotSpeed(dAuxSpeed, wref*direction);
    }

    //
    // When the robot is on the last waypoint, checks the distance to the end
    if( pathCurrent.GetCurrentWaypointIndex() >= (pathCurrent.NumOfWaypoints() - 2) ){
      ret = -10;
      double ddist2 = Dist( current_position.x, current_position.y, last_waypoint.dX, last_waypoint.dY);
      // Distancia recorrida
      //dDistCovered = Dist( current_position.px, current_position.py, odomWhenLastWaypoint.px, odomWhenLastWaypoint.py);
      ///到终点精度
      if (ddist2 < WAYPOINT_POP_DISTANCE_M) {
        SetRobotSpeed(0.0, 0.0);

        ROS_INFO("%s::PurePursuit: target position reached (%lf, %lf, %lf). Ending current path", sComponentName.c_str(), current_position.x, current_position.x, current_position.theta*180.0/Pi);

        pathCurrent.Clear();
        return 1;
      }
    }

    return 0;
  }

  /*!  \fn void CancelPath()
   * Removes all the waypoints introduced in the system
  */
  void CancelPath(){

    pathCurrent.Clear();  // Clears current path
    pathFilling.Clear();  // Clears the auxiliary path
    while(!qPath.empty())  // Clears the queue of paths
      qPath.pop();

    bCancel = false;
    // Cancels current action
    ROS_INFO("%s::CancelPath: action server preempted", sComponentName.c_str());
    action_server_goto.setPreempted();
  }

  /*!  \fn void SetRobotSpeed()
  */
  void SetRobotSpeed(double speed, double angle){
    if(command_type == COMMAND_ACKERMANN){
      ackermann_msgs::AckermannDriveStamped ref_msg;

      ref_msg.header.stamp = ros::Time::now();
      ref_msg.drive.jerk = 0.0;
      ref_msg.drive.acceleration = 0.0;
      ref_msg.drive.steering_angle_velocity = 0.0;
      ref_msg.drive.steering_angle = angle;
      ref_msg.drive.speed = speed;

      vel_pub_.publish(ref_msg);
    }else{
      geometry_msgs::Twist ref_msg;
      ref_msg.angular.x = 0.0;  ref_msg.angular.y = 0.0; ref_msg.angular.z = angle;
      ref_msg.linear.x = speed;   ref_msg.linear.y = 0.0; ref_msg.linear.z = 0.0;
      vel_pub_.publish(ref_msg);
    }
  }


  /*!  \fn void ShutDownState()
  */
  void ShutDownState(){
    if(bRunning)
      Stop();
    else if(bInitialized)
      ShutDown();

  }

  /*!  \fn void EmergencyState()
  */
  void EmergencyState(){
    if(CheckOdomReceive() == 0){
      SwitchToState(STANDBY_STATE);
      return;
    }

  }

  /*!  \fn void FailureState()
  */
  void FailureState(){

  }

  /*!  \fn void AllState()
  */
  void AllState(){
    //ROS_INFO("purepursuit.AllState start...");
    // Only if we use the map as source for positioning
    if(ui_position_source == MAP_SOURCE){
      try{
        listener.lookupTransform("/map", target_frame_, ros::Time(0), transform);
        geometry_msgs::TransformStamped msg;
        tf::transformStampedTFToMsg(transform, msg);
        pose2d_robot.x = msg.transform.translation.x;
        pose2d_robot.y = msg.transform.translation.y;
        pose2d_robot.theta = tf::getYaw(msg.transform.rotation);
        // Safety check
        last_map_time = ros::Time::now();
        msg.header.stamp = last_map_time;
        tranform_map_pub_.publish(msg);
      }catch (tf::TransformException ex){
        ROS_ERROR("%s::AllState lookupTransform from %s to %s failed!info: %s", sComponentName.c_str(),"map",target_frame_.c_str(), ex.what());
      }
    }

    AnalyseCB();  // Checks action server state

    ReadAndPublish();  // Reads and publish into configured topics

    if(bCancel)    // Performs the cancel in case of required
      CancelPath();

    //ROS_INFO("purepursuit.AllState end...");
  }

  /*! \fn void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
    * Receives odom values
  */
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
  {
    // Safety check
    last_command_time = ros::Time::now();

    // If we want to use the odom source, subscribes to odom topic
    if(ui_position_source == ODOM_SOURCE){
      // converts the odom to pose 2d
      pose2d_robot.x = odom_value->pose.pose.position.x;
      pose2d_robot.y = odom_value->pose.pose.position.y;
      pose2d_robot.theta = tf::getYaw(odom_value->pose.pose.orientation);
    }

    // Copies the current odom
    odometry_robot = *odom_value;
    // Gets the linear speed
    dLinearSpeed = odometry_robot.twist.twist.linear.x;
  }

  /*! \fn int CheckOdomReceive()
    * Checks whether or not it's receiving odom values and/or map transformations
    * \return 0 if OK
    * \return -1 if ERROR
  */
  int CheckOdomReceive()
  {
    // Safety check
    if((ros::Time::now() - last_command_time).toSec() > ODOM_TIMEOUT_ERROR)
      return -1;
    else{
      if( ui_position_source == MAP_SOURCE and ((ros::Time::now() - last_map_time).toSec() > MAP_TIMEOUT_ERROR))
        return -1;
      else return 0;
    }

  }


  void executeCB(const robotnik_pp_msgs::GoToGoalConstPtr &goal)
  {

  }

  /*! \fn int CalculateDirectionSpeed(Waypoint target_position)
  *  \brief Calcula el sentido de movimiento de una ruta, dependiendo de la posición inicial y el ángulo del robot
  *  \return 1 si el sentido es positivo
  *  \return -1 si el sentido es negativo
  */
  int CalculateDirectionSpeed(Waypoint target_position){
    int ret = 1;
    double alpha = pose2d_robot.theta;
    double x =  pose2d_robot.x, y = pose2d_robot.y;
    double ux, uy, vx, vy;
    double beta = 0.0;
    static int last_direction = 0;
    static double pi_medios = M_PI / 2.0, max_diff = 5.0 * M_PI / 180.0;
    int iCase = 0;

    //
    // si la posicion objetivo es la misma del robot, devolvemos 0
    if( (target_position.dX == x) && (target_position.dY == y) ){
      return 0;
    }
    // Cálculo del vector director del robot respecto al sistema de coordenadas del robot
    ux = cos(alpha);
    uy = sin(alpha);
    // Cálculo del vector entre el punto objetivo y el robot
    vx = target_position.dX - x;
    vy = target_position.dY - y;

    // Cálculo del ángulo entre el vector director y el vector al punto objetivo
    beta = acos( (ux * vx + uy * vy) / ( sqrt(ux*ux + uy*uy) * sqrt(vx*vx + vy*vy) ) );

    // Devolvemos valor dependiendo del ángulo entre la orientación del robot y la posición objetivo (radianes)
    // Tendremos en cuenta el valor del sentido de avance de la última ruta.
    if(fabs(beta) <= pi_medios){
      // Calculo inicial de direccion
      if(last_direction == 0)
        ret = 1;
      else {
        ret = 1;

        if( fabs(beta - pi_medios) <= max_diff){
          if(last_direction != ret){
            iCase = 1;
            ret = -1;

          }else {
            iCase = 2;
          }
        }

      }
    }else{
      // Calculo inicial de direccion
      if(last_direction == 0)
        ret = -1;
      else {
        ret = -1;
        if(fabs(beta - pi_medios) <= max_diff){
          if(last_direction != ret){
            ret = 1;
            iCase = 3;
          }else{
            iCase = 4;
          }
        }

      }
    }

    ROS_INFO("%s:CalculateDirectionSpeed:  case %d. Beta = %.2lf. Diff = %.2lf. Last direction = %d, new direction = %d", sComponentName.c_str(),
         iCase, beta*180.0/M_PI, (beta - pi_medios)*180.0/M_PI, last_direction, ret);

    last_direction = ret;
    return ret;
  }

  /*! \fn int CalculateDirectionSpeed(geometry_msgs::Pose2D target_position)
  *  \brief Calcula el sentido de movimiento de una ruta, dependiendo de la posición inicial y el ángulo del robot
  * 根据机器人的初始位置和角度计算路线的移动方向
  *  \return 1 si el sentido es positivo
  *  \return -1 si el sentido es negativo
  */
  int CalculateDirectionSpeed(geometry_msgs::Pose2D target_position){
    int ret = 1;
    double alpha = pose2d_robot.theta;
    double x =  pose2d_robot.x, y = pose2d_robot.y;
    double ux, uy, vx, vy;
    double beta = 0.0;
    static int last_direction = 0;
    static double pi_medios = M_PI / 2.0, max_diff = 5.0 * M_PI / 180.0;
    int iCase = 0;

    //
    // si la posicion objetivo es la misma del robot, devolvemos 0
    if( (target_position.x == x) && (target_position.y == y) ){
      return 0;
    }
    // Cálculo del vector director del robot respecto al sistema de coordenadas del robot
    ux = cos(alpha);
    uy = sin(alpha);
    // Cálculo del vector entre el punto objetivo y el robot
    vx = target_position.x - x;
    vy = target_position.y - y;
    //计算方向向量和向量到目标点之间的角度
    // Cálculo del ángulo entre el vector director y el vector al punto objetivo
    beta = acos( (ux * vx + uy * vy) / ( sqrt(ux*ux + uy*uy) * sqrt(vx*vx + vy*vy) ) );
    //我们根据机器人的方向和目标位置（弧度）之间的角度返回值
    // 我们将考虑最后一条路线前进方向的价值。
    // Devolvemos valor dependiendo del ángulo entre la orientación del robot y la posición objetivo (radianes)
    // Tendremos en cuenta el valor del sentido de avance de la última ruta.
    if(fabs(beta) <= pi_medios){
      // Calculo inicial de direccion
      // 初步计算方向
      if(last_direction == 0)
        ret = 1;
      else {
        ret = 1;

        if( fabs(beta - pi_medios) <= max_diff){
          if(last_direction != ret){
            iCase = 1;
            ret = -1;

          }else {
            iCase = 2;
          }
        }

      }
    }else{
      // Calculo inicial de direccion
      if(last_direction == 0)
        ret = -1;
      else {
        ret = -1;
        if(fabs(beta - pi_medios) <= max_diff){
          if(last_direction != ret){
            ret = 1;
            iCase = 3;
          }else{
            iCase = 4;
          }
        }

      }
    }

    ROS_INFO("%s:CalculateDirectionSpeed:  case %d. Beta = %.2lf. Diff = %.2lf. Last direction = %d, new direction = %d", sComponentName.c_str(),
         iCase, beta*180.0/M_PI, (beta - pi_medios)*180.0/M_PI, last_direction, ret);

    last_direction = ret;
    return ret;
  }

  /*!  \fn ReturnValue Agvs::MergePath()
   *   \brief Merges the current path with the next path
  */
  ReturnValue MergePath(){
    Waypoint new_waypoint, wFirst, wLast;
    Path aux;

    if(action_server_goto.isNewGoalAvailable()){
      goto_goal.target = action_server_goto.acceptNewGoal()->target; // Reads points from action server
      if(goto_goal.target.size() > 0){
        if(goto_goal.target.size() > 1){  // Tries to use the second point of the route
          direction = CalculateDirectionSpeed(goto_goal.target[1].pose);
        }else{
          direction = CalculateDirectionSpeed(goto_goal.target[0].pose);
        }
        ///force to use front laser  chq!!!!!!!!---->
        direction = 1;
         ///force to use front laser  chq!!!!!!!<----
        if(direction == 1){  // Uses only front laser
          SetLaserFront();
        }else{  // Uses only back laser
          SetLaserBack();
        }

        for(int i = 0; i < goto_goal.target.size(); i++){
          ROS_INFO("%s::MergePath: Point %d (%.3lf, %.3lf, %.3lf) speed = %.3lf", sComponentName.c_str(), i,  goto_goal.target[i].pose.x,
          goto_goal.target[i].pose.y, goto_goal.target[i].pose.theta, goto_goal.target[i].speed  );

          new_waypoint.dX = goto_goal.target[i].pose.x;
          new_waypoint.dY = goto_goal.target[i].pose.y;
          new_waypoint.dA = goto_goal.target[i].pose.theta;
          // Depending on the calculated motion direction, applies positive or negative speed
          if(direction == 1){
            new_waypoint.dSpeed = fabs(goto_goal.target[i].speed);
          }else{
            new_waypoint.dSpeed = -fabs(goto_goal.target[i].speed);
          }

          pathFilling.AddWaypoint(new_waypoint);
        }
        //if(pathFilling.Optimize(AGVS_TURN_RADIUS) != OK)
        if(pathFilling.OptimizeByBSpline(AGVS_TURN_RADIUS) != OK)
          ROS_ERROR("%s::GoalCB: Error optimizing the path", sComponentName.c_str());

        //pathFilling.Print();
        // Adds the new path to the queue

        qPath.push(pathFilling);
        // Clears temporary path object
        pathFilling.Clear();

        goto_feedback.percent_complete = 0.0;  // Inits the feedback percentage

        // Only if exists any path into the queue
        if(qPath.size() > 0){
          aux = qPath.front();
          aux.GetWaypoint(0, &wFirst);
          aux.BackWaypoint(&wLast);
          ROS_INFO("%s::MergePath: Adding new %d points from (%.2lf, %.2lf) to (%.2lf, %.2lf) and %d magnets", sComponentName.c_str(), aux.NumOfWaypoints() ,wFirst.dX, wFirst.dY, wLast.dX, wLast.dY, aux.NumOfMagnets());
          ROS_INFO("%s::MergePath: Current number of points = %d and magnets = %d", sComponentName.c_str(), pathCurrent.NumOfWaypoints() , pathCurrent.NumOfMagnets());

          // Adds the first path in the queue to the current path
          pathCurrent+=qPath.front();

          // Needs at least two points
          if(pathCurrent.NumOfWaypoints() < 2){
            if(pathCurrent.CreateInterpolatedWaypoint(this->pose2d_robot) == ERROR){
              ROS_ERROR("%s::MergePath: Error adding an extra point", sComponentName.c_str());
            }

          }

          ROS_INFO("%s::MergePath: New number of points = %d and magnets = %d", sComponentName.c_str(), pathCurrent.NumOfWaypoints() , pathCurrent.NumOfMagnets());
          // Pops the extracted path
          qPath.pop();

          goto_goal.target.clear();  // removes current goals
          //reset flag
          comp_firstpoint = false;
          need_repub_path = true;
         // if(need_repub_path){
            pubMarkerRfs(pathCurrent.GetWaypoints());//rviz打印跟踪点和跟踪方向 add for debugging
         //   need_repub_path = false;
         // }
          return OK;
        }
      }

    }

    return ERROR;
  }

  /*! \fn void GoalCB()
    * Called when receiving a new target. (ActionServer)
  */
  void GoalCB()
  {



  }

  /*! \fn void PreemptCB()
    * Called to cancel or replace current mision. (ActionServer)
  */
  void PreemptCB()
  {
    bCancel = true;
  }

  /*! \fn void AnalyseCB()
    * Checks the status. (ActionServer)
  */
  void AnalyseCB(){
    static int cnt =0 ;
    if (!action_server_goto.isActive()){
      if( cnt++ > 100){
        ROS_INFO("%s::AnalyseCB: Not active.return!", sComponentName.c_str());
        cnt = 0;
      }
      return;
    }
    //goto_feedback.percent_complete+=1.0;

    action_server_goto.publishFeedback(goto_feedback);

    if(goto_feedback.percent_complete == 100.0){
      //action_server_goto.setSucceeded(goto_result);
      action_server_goto.setAborted(goto_result);
      ROS_INFO("%s::AnalyseCB: Action finished", sComponentName.c_str());
    }
  }

  /*! \fn void SetLaserFront()
    * Disables laser back, enables laser front
  */
  bool SetLaserFront(){
    /*s3000_laser::enable_disable srv;

    srv.request.value = false;
    sc_enable_back_laser_.call(srv);
    ROS_INFO("%s::SetLaserFront: Setting laser back to false, ret = %d", sComponentName.c_str(), srv.response.ret);

    srv.request.value = true;
    sc_enable_front_laser_.call(srv);
    ROS_INFO("%s::SetLaserFront: Setting laser front to true, ret = %d", sComponentName.c_str(), srv.response.ret);*/
  }

  /*! \fn void SetLaserBack()
    * Disables laser front, enables laser back
  */
  bool SetLaserBack(){
    /*s3000_laser::enable_disable srv;

    srv.request.value = false;
    sc_enable_front_laser_.call(srv);
    ROS_INFO("%s::SetLaserBack: Setting laser front to false, ret = %d", sComponentName.c_str(), srv.response.ret);

    srv.request.value = true;
    sc_enable_back_laser_.call(srv);
    ROS_INFO("%s::SetLaserBack: Setting laser back to true, ret = %d", sComponentName.c_str(), srv.response.ret);*/
  }

}; // class purepursuit_planner_node

// MAIN
int main(int argc, char** argv)
{
  ///程序会以这个名字发布server,所以不要轻易改这个个名字
  ros::init(argc, argv, "robotnik_pp_planner");

  ros::NodeHandle n;

  purepursuit_planner_node planner(n);

  planner.ControlThread();

  return (0);
}
// EOF
