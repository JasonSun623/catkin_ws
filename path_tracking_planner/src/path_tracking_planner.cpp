
#include "path_tracking_planner.h"


namespace path_tracking_planner {



ReturnValue PathTrackingPlanner::Start(){

    if(bRunning){
//      ROS_INFO("agvs_controller::Start: the component's thread is already running");
      return THREAD_RUNNING;
    }


    bRunning = true;
    return OK;
}

ReturnValue PathTrackingPlanner::Setup(){
    // Checks if has been initialized
    if(bInitialized){
//      ROS_INFO("purepursuit_planner::Setup: Already initialized");
      return INITIALIZED;
    }

    // Starts action server

    action_server_goto.start();

    bInitialized = true;
//    ROS_INFO("purepursuit_planner::Setup.start action_server_goto done!");//chq
    return OK;
}

ReturnValue PathTrackingPlanner::Stop(){

    if(!bRunning){
//      ROS_INFO("agvs_controller::Stop: Thread not running");

      return THREAD_NOT_RUNNING;
    }

    bRunning = false;

    return OK;
}

void PathTrackingPlanner::ControlThread()
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


void PathTrackingPlanner::InitState(){
//    ROS_INFO("purepursuit_planner::InitSate:");
    if(bInitialized && bRunning){
      if(CheckOdomReceive() == 0){
//        ROS_INFO("purepursuit_planner::InitState.suc to rec odom,do SwitchToState(STANDBY_STATE).");//chq
        SwitchToState(STANDBY_STATE);
      }
    }else{
      if(!bInitialized)
        Setup();
      if(!bRunning)
        Start();
    }

}


void PathTrackingPlanner::StandbyState(){
    if(CheckOdomReceive() == -1){
//      ROS_WARN("StandbyState.CheckOdomReceive() == -1 doSwitchToState(EMERGENCY_STATE) ");//chq
      SwitchToState(EMERGENCY_STATE);
    }
    else{
      if(bEnabled && !bCancel ){

        if(pathCurrent.Size() > 0 || MergePath() == OK){
//          ROS_INFO("%s::StandbyState: route available", sComponentName.c_str());
          SwitchToState(READY_STATE);
        }
      }

    }
}

void PathTrackingPlanner::ReadyState(){
    //ROS_INFO("purepersuit.ReadyState.start...");
    if(CheckOdomReceive() == -1){
//      ROS_INFO("purepursuit_planner.ReadyState.CheckOdomReceive() == -1.do set speed = 0 SwitchToState(EMERGENCY_STATE),return!");
      SetRobotSpeed(0.0, 0.0);
      SwitchToState(EMERGENCY_STATE);
      return;
    }
    if(!bEnabled){
//      ROS_INFO("%s::ReadyState: Motion is disabled", sComponentName.c_str());
      SetRobotSpeed(0.0, 0.0);
      SwitchToState(STANDBY_STATE);
      return;
    }
    if(bCancel){
//      ROS_INFO("%s::ReadyState: Cancel requested", sComponentName.c_str());
      SetRobotSpeed(0.0, 0.0);
      SwitchToState(STANDBY_STATE);
      return;
    }

    if(obstacle_avoidance_ && bObstacle){
     // ROS_WARN("%s::ReadyState: Obstacle detected", sComponentName.c_str());
      SetRobotSpeed(0.0, 0.0);
      //SwitchToState(STANDBY_STATE);
      return;
    }
    //ROS_INFO("purepursuit_planner.ReadyState.do PurePursuit...");

    int ret = PurePursuit();
//    static int cnt = 0;
    if(ready_cnt_++ > 100){
      ROS_DEBUG("purepursuit_planner.ReadyState.do PurePursuit done.ret(0 controlling,-1 error,1 finished):%d",ret);
      ready_cnt_ = 0;
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

void PathTrackingPlanner::UpdateLookAhead(){
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


double PathTrackingPlanner::Dot2( double x1, double y1, double x2, double y2) {
    //在点积运算中，第一个向量投影到第二个向量上
    //点乘的结果就是两个向量的模相乘,然后再与这两个向量的夹角的余弦值相乘.或者说是两个向量的各个分量分别相乘的结果的和
    //如果点乘的结果为0,那么这两个向量互相垂直；如果结果大于0,那么这两个向量的夹角小于90度；如果结果小于0,那么这两个向量的夹角大于90度
    return (x1*x2 + y1*y2); // cross product
}

double PathTrackingPlanner::Dist(double x1, double y1, double x2, double y2) {
    double diff_x = (x2 - x1);
    double diff_y = (y2 - y1);
    return sqrt( diff_x*diff_x + diff_y*diff_y );
}

double PathTrackingPlanner::DistP2S( geometry_msgs::Pose2D current_position, Waypoint s0, Waypoint s1, Waypoint *Pb)
{
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
ReturnValue PathTrackingPlanner::PointOneByOne(geometry_msgs::Pose2D& current_position, geometry_msgs::Pose2D *wp)
{
  int i=0,j=0,k=0;
  static int last_i=0;
  double dmin, d, d0, d1, d2,d3, *d_seg;
  double t;
  geometry_msgs::Pose2D target_position;
  Waypoint s0, s1, Pb, Pb1;

  int size = pathCurrent.NumOfWaypoints();
  d_seg = new double[size]; //array con la distancia entre puntos consecutivos en la ruta
  i = pathCurrent.GetCurrentWaypointIndex();
  ROS_DEBUG("PointOneByOne.cur way index :%d,comp_firstpoint(1==true):%d",i,comp_firstpoint);
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
  if( (d > d_lookvertical_min_) && (!comp_firstpoint) )///must ahead to first tar + min dist + not comp
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
//    ROS_INFO("PointOneByOne.change to next target------------------index:%d",j);
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
//    ROS_INFO("PointOneByOne.d(%.6f) >= dLookAhead(%.6f).track p",d,dLookAhead);
    return OK;
  }
  else
  {
      if( d2 <= 0 )
      {///d2 s0TOpb
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
//        ROS_DEBUG("PointOneByOne.at back of s0.dist:%.6f",dist);
      }
      else
      {
        double head_dist1 = sqrt(pow(dLookAhead,2)-pow(d,2));
        double dist = d2+head_dist1 ;
        if( d0 > 1e-5 )
          t = dist/d0; //chq
        else
          t = 0;
//        ROS_DEBUG("PointOneByOne.at up or down of s0.dist:%.6f",dist);
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


ReturnValue PathTrackingPlanner::PointDlh(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp)
{
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


int PathTrackingPlanner::PurePursuit()
{
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
      ROS_ERROR("%s::PurePursuit:points num:%d below to 4, not enought waypoints", sComponentName.c_str(), pathCurrent.NumOfWaypoints());
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
#if 0
    // Curvature
    dx = current_position.x - next_position.x;
    dy = current_position.y - next_position.y;
    x1 = cos(yaw)*dx + sin(yaw)*dy; //Original
    y1 = -sin(yaw)*dx + cos(yaw)*dy;

    if ((x1*x1 + y1*y1) == 0)
        curv = 0;
    else
        curv = (2.0 / (x1*x1 + y1*y1)) * -y1;  		//Original

    // Obtenemos alfa_ref en bucle abierto segun curvatura
    wref = atan(d_dist_wheel_to_center_/(1.0/curv));


    if(pathCurrent.BackWaypoint(&last_waypoint) == ERROR){
        ROS_ERROR("%s::PurePursuit: Error getting the last point in the path", sComponentName.c_str());
        return -1;
    }

    double dAuxDist = Dist(current_position.x, current_position.y, last_waypoint.dX, last_waypoint.dY);	//dist(waypoints.back().pos, current_position);

    if(pathCurrent.GetNextWaypoint(&next_waypoint) == ERROR){
        ROS_ERROR("%s::PurePursuit: Error getting next waypoint in the path", sComponentName.c_str());
        return -1;
    }

    dAuxSpeed = next_waypoint.dSpeed;

    if (dAuxSpeed >= 0)
       dth = next_position.theta - current_position.theta;
    else
      dth = -(next_position.theta + Pi - current_position.theta);


    // normalize
    radnorm(&dth);
    double aux_wref = wref;
    wref += Kr * dth;
#endif
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

     //-------------------------------------------------------------------------------
     if(dAuxSpeed < 0.0){
         std::cout << "[ PurePursuit ] before dAuxSpeed : " << dAuxSpeed;
     }

//     assert(dAuxSpeed > 0.0);

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
      wref = atan2(-y1,-x1);
      ROS_DEBUG("-------------purepersuit.using turn directly------------------------");
    }
    else
    {
      //wref = dAuxSpeed * curv;
      wref = atan2(-y1,-x1);
//    wref = atan(d_dist_wheel_to_center_/(1.0/curv));
    }



//    wref += Kr * dth;



    boost::mutex::scoped_lock dAngulerLock(dAnguler_mutex_);
    double dAngulerSpeed_temp = dAngulerSpeed;
    dAngulerLock.unlock();

    ROS_INFO("Purepursuit: before pid cal wref = %f, dAngulerSpeed = %f", wref, dAngulerSpeed_temp);

    wref = pidTheta->calculate(wref, dAngulerSpeed_temp);

//    boost::mutex::scoped_lock dAngulerLock(dAnguler_mutex_);
//    dAngulerSpeed = odometry_robot.twist.twist.angular.z;
//    dAngulerLock.unlock();

    pubTarget(current_position,next_position,wref);//added by chq

    ROS_INFO("Purepursuit: affter pid cal wref = %f, dAngulerSpeed = %f, speed = %f", wref, dAngulerSpeed_temp, dAuxSpeed);

    radnorm(&wref);
    //-------------------------------------------------------------------------------
//    if(dAuxSpeed < 0.0){
//        std::cout << "[ PurePursuit ] affter dAuxSpeed : " << dAuxSpeed;
//    }

//    assert(dAuxSpeed > 0.0);

    //ROS_INFO("purepersuit.after mod, the speed aux:%.3f",dAuxSpeed);
    SetRobotSpeed(dAuxSpeed, wref);
//    if(command_type == COMMAND_ACKERMANN){
//      SetRobotSpeed(dAuxSpeed, wref);
//    }else{
//      SetRobotSpeed(dAuxSpeed, wref*direction);
//    }

    //
    // When the robot is on the last waypoint, checks the distance to the end
    if( pathCurrent.GetCurrentWaypointIndex() >= (pathCurrent.NumOfWaypoints() - 2) ){
      ret = -10;
      double ddist2 = Dist( current_position.x, current_position.y, last_waypoint.dX, last_waypoint.dY);
      // Distancia recorrida
      //dDistCovered = Dist( current_position.px, current_position.py, odomWhenLastWaypoint.px, odomWhenLastWaypoint.py);
      ///到终点精度
      if (ddist2 < waypoint_pop_distance_) {
        SetRobotSpeed(0.0, 0.0);

        ROS_INFO("%s::PurePursuit: target position reached (%lf, %lf, %lf). Ending current path", sComponentName.c_str(), current_position.x, current_position.x, current_position.theta*180.0/Pi);

        pathCurrent.Clear();
        return 1;
      }
    }

   // ROS_WARN("%s::PurePursuit : Current way index : %d !", sComponentName.c_str(), pathCurrent.GetCurrentWaypointIndex() );
    goto_feedback.seq_num = pathCurrent.GetCurrentWaypointIndex();
    goto_feedback.index = pathCurrent.getIndex(goto_feedback.seq_num);
    goto_feedback.pass_distance = dAuxDist;
    goto_feedback.seq_tatol = pathCurrent.NumOfWaypoints();

    return 0;
}


void PathTrackingPlanner::CancelPath()
{

    pathCurrent.Clear();  // Clears current path
    pathFilling.Clear();  // Clears the auxiliary path
    while(!qPath.empty())  // Clears the queue of paths
      qPath.pop();

    bCancel = false;
    // Cancels current action
    ROS_INFO("%s::CancelPath: action server preempted", sComponentName.c_str());
    action_server_goto.setPreempted(wash_floor_msgs::GoToResult(), "Canel all path.");
    //action_server_goto.setAborted(wash_floor_msgs::GoToResult(), "Canel Path. This is a bug, please report it.");
}

void PathTrackingPlanner::SetRobotSpeed(double speed, double angle)
{
    if(command_type == COMMAND_ACKERMANN)
    {
      ackermann_msgs::AckermannDriveStamped ref_msg;

      ref_msg.header.stamp = ros::Time::now();
      ref_msg.drive.jerk = 0.0;
      ref_msg.drive.acceleration = 0.0;
      ref_msg.drive.steering_angle_velocity = 0.0;
      ref_msg.drive.steering_angle = angle;
      ref_msg.drive.speed = speed;

      vel_pub_.publish(ref_msg);
    }else
    {
      geometry_msgs::Twist ref_msg;
      ref_msg.angular.x = 0.0;  ref_msg.angular.y = 0.0; ref_msg.angular.z = angle;
      ref_msg.linear.x = speed;   ref_msg.linear.y = 0.0; ref_msg.linear.z = 0.0;
      vel_pub_.publish(ref_msg);
    }
}


void PathTrackingPlanner::ShutDownState()
{
    if(bRunning)
      Stop();
    else if(bInitialized)
      ShutDown();

}


void PathTrackingPlanner::EmergencyState()
{
    if(CheckOdomReceive() == 0){
      SwitchToState(STANDBY_STATE);
      return;
    }

}


void PathTrackingPlanner::FailureState(){

}

void PathTrackingPlanner::AllState()
{
    //ROS_INFO("purepursuit.AllState start...");
    // Only if we use the map as source for positioning
    if(ui_position_source == MAP_SOURCE)
    {
      try
      {
        listener.lookupTransform(global_frame_id_, target_frame_, ros::Time(0), transform);
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
        ROS_ERROR("%s::AllState lookupTransform from %s to %s failed ! info: %s", sComponentName.c_str(),"map",target_frame_.c_str(), ex.what());
      }
    }

    AnalyseCB();  // Checks action server state

    ReadAndPublish();  // Reads and publish into configured topics

    if(bCancel)    // Performs the cancel in case of required
      CancelPath();

    //ROS_INFO("purepursuit.AllState end...");
}


void PathTrackingPlanner::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
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

    boost::mutex::scoped_lock dAngulerLock(dAnguler_mutex_);
    // Copies the current odom
    odometry_robot = *odom_value;
    // Gets the linear speed
    dLinearSpeed = odometry_robot.twist.twist.linear.x;
    dAngulerSpeed = odometry_robot.twist.twist.angular.z;

}

int PathTrackingPlanner::CheckOdomReceive()
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


void PathTrackingPlanner::executeCB(const wash_floor_msgs::GoToGoalConstPtr &goal)
{

}

int PathTrackingPlanner::CalculateDirectionSpeed(Waypoint target_position)
{
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

int PathTrackingPlanner::CalculateDirectionSpeed(geometry_msgs::Pose2D target_position){
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

ReturnValue PathTrackingPlanner::MergePath()
{
  Waypoint new_waypoint, wFirst, wLast;
  Path aux;

  //if(!action_server_goto.isPreemptRequested()){
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

        // clean last action feedback buffer
        goto_feedback.seq_num = 0;
        goto_feedback.index = 0;
        goto_feedback.pass_distance = 0;
        goto_feedback.seq_tatol = 0;

        for(int i = 0; i < goto_goal.target.size(); i++)
        {
          ROS_DEBUG("%s::MergePath: Point %d (%.3lf, %.3lf, %.3lf) speed = %.3lf", sComponentName.c_str(), i,  goto_goal.target[i].pose.x,
          goto_goal.target[i].pose.y, goto_goal.target[i].pose.theta, goto_goal.target[i].speed  );

          new_waypoint.dX = goto_goal.target[i].pose.x;
          new_waypoint.dY = goto_goal.target[i].pose.y;
          new_waypoint.dA = goto_goal.target[i].pose.theta;
          new_waypoint.index = goto_goal.target[i].index;

          // Depending on the calculated motion direction, applies positive or negative speed
          if(direction == 1){
            new_waypoint.dSpeed = fabs(goto_goal.target[i].speed);
          }else{
            new_waypoint.dSpeed = -fabs(goto_goal.target[i].speed);
          }

          pathFilling.AddWaypoint(new_waypoint);
        }
        //if(pathFilling.Optimize(AGVS_TURN_RADIUS) != OK)
        if(pathFilling.OptimizeByBSpline(AGVS_TURN_RADIUS) != OK){
          ROS_ERROR("%s::GoalCB: Error optimizing the path", sComponentName.c_str());
        }
        //pathFilling.Print();
        // Adds the new path to the queue

        qPath.push(pathFilling);
        // Clears temporary path object
        pathFilling.Clear();

        goto_feedback.percent_complete = 0.0;  // Inits the feedback percentage

        // Only if exists any path into the queue
        if(qPath.size() > 0)
        {
          aux = qPath.front();
          aux.GetWaypoint(0, &wFirst);
          aux.BackWaypoint(&wLast);
          ROS_DEBUG("%s::MergePath: Adding new %d points from (%.2lf, %.2lf) to (%.2lf, %.2lf) and %d magnets", sComponentName.c_str(), aux.NumOfWaypoints() ,wFirst.dX, wFirst.dY, wLast.dX, wLast.dY, aux.NumOfMagnets());
          ROS_DEBUG("%s::MergePath: Current number of points = %d and magnets = %d", sComponentName.c_str(), pathCurrent.NumOfWaypoints() , pathCurrent.NumOfMagnets());

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
  //}


  return ERROR;
}


void PathTrackingPlanner::GoalCB()
{



}


void PathTrackingPlanner::PreemptCB()
{
    bCancel = true;
}

void PathTrackingPlanner::AnalyseCB(){

    cnt_++;

    if (!action_server_goto.isActive())
    {
      if( cnt_ > 500){
        ROS_DEBUG("%s::AnalyseCB: Not active.return!", sComponentName.c_str());
        cnt_ = 0;
      }
      return;
    }


    //goto_feedback.percent_complete+=1.0;

    if( (cnt_ % 10) == 0)
    {
//      ROS_WARN("%s::AnalyseCB: Publish Feedback !", sComponentName.c_str());
      action_server_goto.publishFeedback(goto_feedback);
    }


    if(goto_feedback.percent_complete == 100.0)
    {
      //action_server_goto.setSucceeded(goto_result);
      action_server_goto.setSucceeded(goto_result);
      ROS_INFO("%s::AnalyseCB: Action finished", sComponentName.c_str());
    }
}


}


