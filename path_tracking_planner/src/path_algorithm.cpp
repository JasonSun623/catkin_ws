#include "path_algorithm.h"


namespace path_tracking_planner {



//! public constructor
Path::Path()
{
    iCurrentWaypoint = iCurrentMagnet = -1;
    pthread_mutex_init(&mutexPath, NULL);//Initialization for WaypointRoutes' mutex
    bOptimized = false;
}

//! Destructor
Path::~Path()
{
    pthread_mutex_destroy(&mutexPath);
}

//! Adds a new waypoint
ReturnValue Path::AddWaypoint(Waypoint point)
{
  Waypoint aux;

  pthread_mutex_lock(&mutexPath);
  if(vPoints.size() > 0)
  {
    aux = vPoints.back();
    // Only adds the waypoint if it's different from waypoint before
    if( (aux.dX != point.dX) || (aux.dY != point.dY) )
      vPoints.push_back(point);
  } else
  { // First point
    if(iCurrentWaypoint < 0){ //First point
      iCurrentWaypoint = 0;
    }

    vPoints.push_back(point);
  }

  pthread_mutex_unlock(&mutexPath);

return OK;
}

//! Adds a vector of waypoints
ReturnValue Path::AddWaypoint(vector <Waypoint> po)
{
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
ReturnValue Path::CreateInterpolatedWaypoint(geometry_msgs::Pose2D pose)
{
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
ReturnValue Path::AddMagnet(MagnetStruct magnet)
{
    pthread_mutex_lock(&mutexPath);
    if(iCurrentMagnet < 0){ //First point
      iCurrentMagnet = 0;
    }

    vMagnets.push_back(magnet);
    pthread_mutex_unlock(&mutexPath);

    return OK;
}

  //! Adds a vector of magnets
ReturnValue Path::AddMagnet(vector <MagnetStruct> po)
{
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
void Path::Clear()
{
    pthread_mutex_lock(&mutexPath);
      iCurrentWaypoint = -1;
      iCurrentMagnet = -1;
      bOptimized = false;
      vPoints.clear();
      vMagnets.clear();
    pthread_mutex_unlock(&mutexPath);
}

  //! Returns the size of the vector points
unsigned int Path::Size(){
    return vPoints.size();
}

  //! Returns the next waypoint
ReturnValue Path::GetNextWaypoint(Waypoint *wp)
{
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
ReturnValue Path::BackWaypoint(Waypoint *wp)
{
    ReturnValue ret = ERROR;

    pthread_mutex_lock(&mutexPath);
      if( vPoints.size() > 0){
        *wp = vPoints.back();
        ret = OK;
      }
    pthread_mutex_unlock(&mutexPath);

    return ret;
}

int Path::getIndex(size_t id) const
{
    if(vPoints.size() > id) {
        return vPoints[id].index;
    }
    else
    {
        return 0;
    }

}

  //! Gets the current waypoint
ReturnValue Path::GetCurrentWaypoint(Waypoint *wp)
{
    ReturnValue ret = ERROR;

    pthread_mutex_lock(&mutexPath);
    if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < vPoints.size()) )
    {
        *wp = vPoints[iCurrentWaypoint];
        ret = OK;
    }
    pthread_mutex_unlock(&mutexPath);

    return ret;
  }

  //! Gets selected waypoint
ReturnValue Path::GetWaypoint(int index, Waypoint *wp)
{
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
std::vector <Waypoint> Path::GetWaypoints(){
    return vPoints;
}

  //! Gets the current Waypoint in the path
int Path::GetCurrentWaypointIndex(){
    return iCurrentWaypoint;
}

  //!  Sets the current Waypoint to index
ReturnValue Path::SetCurrentWaypoint(int index)
{
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
void Path::NextWaypoint(){
    pthread_mutex_lock(&mutexPath);
      iCurrentWaypoint++;
    pthread_mutex_unlock(&mutexPath);
}

  //! Returns the number of waypoints
int Path::NumOfWaypoints(){
    return vPoints.size();
}

  //! Returns the next magnet
ReturnValue Path::GetNextMagnet(MagnetStruct *mg)
{
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
ReturnValue Path::BackMagnet(MagnetStruct * mg)
{
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
ReturnValue Path::GetCurrentMagnet( MagnetStruct * mg )
{
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
ReturnValue Path::GetPreviousMagnet( MagnetStruct * mg )
{
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
int Path::GetCurrentMagnetIndex(){
    return iCurrentMagnet;
}

  //!  Sets the current magnet to index
ReturnValue Path::SetCurrentMagnet(int index)
{
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
void Path::NextMagnet(){
    pthread_mutex_lock(&mutexPath);
      iCurrentMagnet++;
    pthread_mutex_unlock(&mutexPath);
}

   //! Gets the last magnet
ReturnValue Path::GetLastMagnet( MagnetStruct * mg )
{
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
int Path::NumOfMagnets(){
    return vMagnets.size();
}

  //! Overloaded operator +=
Path& Path::operator+=(const Path &a){
    AddWaypoint(a.vPoints);
    AddMagnet(a.vMagnets);
    return *this;
}

  //! Cross product
double Path::dot2( Waypoint w, Waypoint v) {
    return (w.dX*v.dX + w.dY*v.dY);
}

  //! Obtains the points for a quadratic Bezier's curve
  //! \param cp0 as player_pose2d_t, control point 0
  //!  \param cp1 as player_pose2d_t, control point 1
  //!  \param cp2 as player_pose2d_t, control point 2
  //!  \param t as float, [0 ... 1]
  //!  \return Point over the curve
Waypoint Path::PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t){
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
Waypoint Path::PosOnCubicBSpline(Waypoint cp0, Waypoint cp1, Waypoint cp2,Waypoint cp3, float t){
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
  double Path::DistForSpeed(double target_speed){
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
ReturnValue Path::Optimize(double distance)
{
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
            }
            else{
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
ReturnValue Path::OptimizeByBSpline(double distance)
{
    int i, j = 0;
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

    if((vPoints.size() < 4) || ( distance <= 0.0)){  //Minimo 3 puntos en la ruta
      ROS_ERROR("WaypointRoute::OptimizeByBSpline: Error: not enought points (%d)\n",Size());
      return ERROR;
    }

    //--------------------------------------------------------------------------------
    for(size_t x=0;x<vPoints.size();x++){
        if(vPoints[x].dSpeed < 0.0){
            std::cout << "[ Path::OptimizeByBSpline ] start vPoint.index : " << vPoints[x].index << "vPoint.speed : " << vPoints[x].dSpeed << "\n";
        }
        assert(vPoints[x].dSpeed > 0.0);
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
    for(i=2; i < vPoints.size(); i++)
    {  // Primera pasada, añadimos puntos para los giros en curvas
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
      //--------------------------------------------------------------------------------------
      if(vPoints[c].dSpeed < 0 ) std::cout << "loop begin.i: " << i << ":vPoints[c].dSpeed:"<< vPoints[c].dSpeed<<" below zero\n";
      assert(vPoints[c].dSpeed > 0);

      ab.dX = Bx - Ax;
      ab.dY = By - Ay;
      mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);

      bc.dX = Cx - Bx;
      bc.dY = Cy - By;
      mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);

      dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));
      ///add by chq ---s-----
      /// force limit start speed in two meter
      double dist;
     if( !have_beyond_onemeter ){
       dist = pow(vPoints[c].dX-vPoints[0].dX,2)+pow(vPoints[c].dY-vPoints[0].dY,2);
       if(dist>2*2)
         have_beyond_onemeter = true;
     }
     if(!have_beyond_onemeter){
       double sp = vPoints[c].dSpeed;
       if(sp>MAX_SPEED_LVL1)vPoints[c].dSpeed=MAX_SPEED_LVL1;
       if(sp<-MAX_SPEED_LVL1)vPoints[c].dSpeed=-MAX_SPEED_LVL1;

       //-----------------------------------------------------------------------------------
       if(vPoints[c].dSpeed < 0 ) std::cout << "!have_beyond_onemeter.i: " << i <<  " sp: " << sp <<" below zero\n";

       assert(vPoints[c].dSpeed > 0.0);
     }
     /// force limit end speed at the last two meter
     double dist_end = pow(vPoints[c].dX-vPoints[vPoints.size()-1].dX,2)+pow(vPoints[c].dY-vPoints[vPoints.size()-1].dY,2);
     if(dist_end < 2*2 ){
       double sp = vPoints[c].dSpeed;
       if(sp>MAX_SPEED_LVL1)vPoints[c].dSpeed=MAX_SPEED_LVL1;
       if(sp<-MAX_SPEED_LVL1)vPoints[c].dSpeed=-MAX_SPEED_LVL1;

       //---------------------------------------------------------------------------------------
       if(vPoints[c].dSpeed < 0 ) std::cout << "dist_end < 2*2.below zero.i:" << i <<  " sp: " << sp << "\n";
        assert(vPoints[c].dSpeed > 0.0);
     }
     ///add by chq ---end------
      //cout <<  i << " Angle =  "<< dAngle << endl;
     ///if need to add buffer point
      if(fabs(dAngle) >= MIN_ANGLE_BSPLINE ){
        //1和２之间要插入新点，所以第二个点要先pop
        new_points.pop_back();

       //the lower the turn angle the lower the limit speed
        if( fabs( dAngle ) >=  Pi/4 ){
          dAuxSpeed = MAX_SPEED_LVL1;//max limit turn speed
        }
        else
          dAuxSpeed = MAX_SPEED_LVL2;

        //cout << "Aux speed = " << dAuxSpeed << ", Next speed =  " << vPoints[b].dSpeed << endl;
        // if vPoints[b].dSpeed > dAuxSpeed
        // set从A到auxPoint(缓冲点)速度保持为vPoints[b].dSpeed原始速度，
        // but from auxPoint to vPoints[b] 速度降低为限制速度dAuxSpeed
        if( fabs(vPoints[b].dSpeed) > dAuxSpeed ){

          if( vPoints[b].dSpeed < 0.0 )  // Cambiamos sentido de avance
            dAuxSpeed = -fabs(dAuxSpeed);
          //-----------------------------------------------------------------------------------
          if( vPoints[b].dSpeed < 0 )
            std::cout << "if(fabs(vPoints[b].dSpeed) > dAuxSpeed).vPoints[b].dSpeed: " << vPoints[b].dSpeed <<", below zero\n";
           assert(vPoints[b].dSpeed > 0.0);

          dMinDist = DistForSpeed(fabs(vPoints[b].dSpeed));//根据速度确定最小限定距离
          //cout << "Min dist = " << dMinDist  << endl;
          //减去１的目的是上一循环可能在１m处插入了缓冲点 chq
          //b-->min_turn_p---->1m-->min_dist------------>c
          if( mod_ab - 1 > dMinDist ){//如果两点距离超过了速度决定的限定距离，可以再插入点
            //we create
            /// chq if the dist bet a and b is much more bigger, then we  create a mid point after a
            ba.dX = -ab.dX;
            ba.dY = -ab.dY;
            K = dMinDist / mod_ab;
            //似乎是简单的线性插值
            aux.dX = Bx + K * ba.dX;  // x = x' + K*Vx
            aux.dY = By + K * ba.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director

            aux.dSpeed = vPoints[b].dSpeed;
            //--------------------------------------------------------------------------
            std::cout << "mod_ab - 1 > dMinDist.vPoints[b].dSpeed < 0.0.add point.vPoints[b].dSpeed:" << vPoints[b].dSpeed << ", i:" << i <<" \n";
            new_points.push_back(aux);
          }

          vPoints[b].dSpeed = dAuxSpeed;

        }
        //第一个航点不会被修改
        //在即将到转折点的地方，按照最小转弯半径distance插入点
        if( mod_ab > distance ){ // si la distancia entre ab es MAYOR a la distancia del punto que pretendemos crear, creamos un punto intermedio
          ///Lo creamos
          // chq if the dist bet a and b is beyond the limit dist then we create a mid point
          ba.dX = -ab.dX;
          ba.dY = -ab.dY;
          K = distance / mod_ab;

          aux.dX = Bx + K * ba.dX;  // x = x' + K*Vx
          aux.dY = By + K * ba.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
          //added by chq 转弯速度限制为dAuxSpeed -start
          if( vPoints[b].dSpeed < 0.0 )
            dAuxSpeed = -fabs(dAuxSpeed);
          //added by chq: if not beyond limit speed then keep the pre speed
          if( (vPoints[b].dSpeed > 0 && dAuxSpeed > vPoints[b].dSpeed)
              || (vPoints[b].dSpeed < 0 && dAuxSpeed < vPoints[b].dSpeed) )
            dAuxSpeed = vPoints[b].dSpeed;

          aux.dSpeed = dAuxSpeed;
          //added by chq -end

          //--------------------------------------------------------------------------
          if(vPoints[b].dSpeed < 0 )
            std::cout << "mod_ab > distance.vPoints[b].dSpeed: " << vPoints[b].dSpeed <<", below zero\n";
           assert(vPoints[b].dSpeed > 0.0);

          std::cout << "mod_ab > distance.add point." << "i:"<< i <<" \n";
          new_points.push_back(aux);
          //j++;
        }
        ///增加完缓冲点+最小提前转弯转折点后，加入原有的转弯点
        new_points.push_back(vPoints[b]);
        ///在原有转弯点的前方加入最小转弯点
        if( mod_bc > distance ){
          ///Lo creamos// chq if the dist bet b and c is beyond the limit dist then we create a mid point
          K = distance / mod_bc;
          //K = distance / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
          aux.dX = Bx + K * bc.dX;  // x = x' + K*Vx
          aux.dY = By + K * bc.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
          ///added by chq start
          if(vPoints[b].dSpeed < 0.0)  // Cambiamos sentido de avance
            dAuxSpeed = -fabs(dAuxSpeed);

          //added by chq: if not beyond limit speed then keep the pre speed
          if( (vPoints[b].dSpeed > 0 && dAuxSpeed > vPoints[b].dSpeed)
              || (vPoints[b].dSpeed < 0 && dAuxSpeed < vPoints[b].dSpeed) )
            dAuxSpeed = vPoints[b].dSpeed;

          aux.dSpeed = dAuxSpeed;
          //--------------------------------------------------------------------------
          if(vPoints[b].dSpeed < 0 ) std::cout << "mod_bc > distance.vPoints[b].dSpeed: " << vPoints[b].dSpeed <<", below zero\n";
           assert(vPoints[b].dSpeed > 0.0);
          ///added by chq end
          //j++;
          //cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;

          new_points.push_back(aux);
        }
        ///加入缓冲点
        if( dMinDist > 0.0) {
          ///减去distance的目的是　为了防止下一个点在距离其前方distance处插入最小转弯点（１m缓冲点必须在下一个最小转弯点前方） chq
          //a-------->1mp-->minturnp---->b
          if( mod_bc - distance > 1.0 ){
            /// chq if the dist bet b and c is beyond 1 m then we create a mid point
            K = 1.0 / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
            aux.dX = Bx + K * bc.dX;  // x = x' + K*Vx
            aux.dY = By + K * bc.dY;  // y = y' + K*Vy  //(Vx, Vy) vector director
            aux.dSpeed = vPoints[b].dSpeed;
            //--------------------------------------------------------------------------
            std::cout << " dMinDist > 0.0&& mod_bc - distance >1 .add point.vPoints[b].dSpeed:" <<vPoints[b].dSpeed << ", i:"<< i <<" \n";
            new_points.push_back(aux);
          }
          else
          {
            // Si no, establecemos una velocidad máxima
            //vPoints[c].dSpeed = vPoints[b].dSpeed;//disabled by chq
            if(vPoints[b].dSpeed < 0.0)
              dAuxSpeed = -fabs(dAuxSpeed);

            //added by chq: if not beyond limit speed then keep the pre speed
            if( (vPoints[b].dSpeed > 0 && dAuxSpeed > vPoints[b].dSpeed)
                || (vPoints[b].dSpeed < 0 && dAuxSpeed < vPoints[b].dSpeed) )
              dAuxSpeed = vPoints[b].dSpeed;

            vPoints[c].dSpeed = dAuxSpeed;

            //--------------------------------------------------------------------------
            if(vPoints[b].dSpeed < 0 ) std::cout << "!dMinDist > 0.0.vPoints[b].dSpeed: " << vPoints[b].dSpeed <<", below zero\n";
             assert(vPoints[b].dSpeed > 0.0);

          }
        }

       ///加入原有转弯后的点
        new_points.push_back(vPoints[c]);

        //--------------------------------------------------------------------------
        std::cout << "add turn point.vPoints[c].dSpeed:" <<vPoints[c].dSpeed << ", i:"<< i <<" \n";

        //j++;
      }
      else{
        //如果没有转折，直接加入下一个点
        new_points.push_back(vPoints[c]);
        //--------------------------------------------------------------------------
        std::cout << "fabs(dAngle) <  MIN_ANGLE_BSPLINE.vPoints[c].dSpeed:" <<vPoints[c].dSpeed << ", i:"<< i <<" \n";
      }
    }
//--------------------------------------------------------------------
    for(size_t x=0;x<vPoints.size();x++){
      if(vPoints[x].dSpeed < 0.0){
          std::cout << "[ Path::OptimizeByBSpline ] before vPoint.index : " << vPoints[x].index<< ", vPoint.speed : " << vPoints[x].dSpeed << "\n";
      }
      assert(vPoints[x].dSpeed > 0.0);
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

    //--------------------------------------------------------------------
    for(size_t x=0;x<vPoints.size();x++){
      if(vPoints[x].dSpeed < 0.0){
          std::cout << "[ Path::OptimizeByBSpline ] affter vPoint.index : " << vPoints[x].index;
      }
      assert(vPoints[x].dSpeed > 0.0);
    }

  //  for(int i = 0; i < new_points.size(); i++){
  //  points.push_back(new_points[i]);
  //  }

  pthread_mutex_unlock(&mutexPath);

  bOptimized  = true;

//  new_points.clear();

  return OK;
}

  //! Prints all the waypoints
void Path::Print(){
    cout << "Path::Print: Printing all the waypoints..." << endl;
    if(vPoints.size() > 0){
      for(int i = 0; i < vPoints.size(); i++){
        cout << "(" << i << ")\t " << vPoints[i].dX << ", " << vPoints[i].dY << endl;
      }
    }else
      cout << "Path::Print: No waypoints..." << endl;
}


}



