#ifndef PATH_ALGORITHM_H_
#define PATH_ALGORITHM_H_

#include <geometry_msgs/Pose2D.h>

#include "Geometry.h"
#include "Component.h"
#include "path_tracking_comm.h"

namespace path_tracking_planner {




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
  Path();

  //! Destructor
  ~Path();

  //! Adds a new waypoint
  ReturnValue AddWaypoint(Waypoint point);

  int getIndex(size_t id) const;

  //! Adds a vector of waypoints
  ReturnValue AddWaypoint(vector <Waypoint> po);

  //! Creates a new point from the desired position to the first waypoint
  ReturnValue CreateInterpolatedWaypoint(geometry_msgs::Pose2D pose);

  //! Adds a new magnet
  ReturnValue AddMagnet(MagnetStruct magnet);

  //! Adds a vector of magnets
  ReturnValue AddMagnet(vector <MagnetStruct> po);

  //! Clears the waypoints and magnets
  void Clear();

  //! Returns the size of the vector points
  unsigned int Size();

  //! Returns the next waypoint
  ReturnValue GetNextWaypoint(Waypoint *wp);

  //! Returns the last waypoint
  ReturnValue BackWaypoint(Waypoint *wp);

  //! Gets the current waypoint
  ReturnValue GetCurrentWaypoint(Waypoint *wp);

  //! Gets selected waypoint
  ReturnValue GetWaypoint(int index, Waypoint *wp);

  //! Gets all waypoints chq
  std::vector <Waypoint> GetWaypoints();

  //! Gets the current Waypoint in the path
  int GetCurrentWaypointIndex();

  //!  Sets the current Waypoint to index
  ReturnValue SetCurrentWaypoint(int index);

   //! Increase waypoint's number
  void NextWaypoint();

  //! Returns the number of waypoints
  int NumOfWaypoints();

  //! Returns the next magnet
  ReturnValue GetNextMagnet(MagnetStruct *mg);

  //! Returns the back magnet
  ReturnValue BackMagnet(MagnetStruct * mg);

  //! Gets the current magnet
  ReturnValue GetCurrentMagnet( MagnetStruct * mg );

  //! Gets the previous magnet
  ReturnValue GetPreviousMagnet( MagnetStruct * mg );

  //! Gets the current MagnetStruct in the path
  int GetCurrentMagnetIndex();

  //!  Sets the current magnet to index
  ReturnValue SetCurrentMagnet(int index);

  //! Increase magnet's number
  void NextMagnet();

   //! Gets the last magnet
  ReturnValue GetLastMagnet( MagnetStruct * mg );

    //! Returns the number of magnets
  int NumOfMagnets();

  //! Overloaded operator +=
  Path &operator+=(const Path &a);

  //! Cross product
  double dot2( Waypoint w, Waypoint v);

  //! Obtains the points for a quadratic Bezier's curve
  //! \param cp0 as player_pose2d_t, control point 0
  //!  \param cp1 as player_pose2d_t, control point 1
  //!  \param cp2 as player_pose2d_t, control point 2
  //!  \param t as float, [0 ... 1]
  //!  \return Point over the curve
  Waypoint PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t);

  //! Obtains the points for a B spline curve
  //!  \param cp0 as player_pose2d_t, control point 0
  //!  \param cp1 as player_pose2d_t, control point 1
  //!  \param cp2 as player_pose2d_t, control point 2
  //!  \param cp3 as player_pose2d_t, control point 3
  //!  \param t as float, [0 ... 1] to get the mid point from <cp1 to cp2>
  //!  \return Point over the curve
  Waypoint PosOnCubicBSpline(Waypoint cp0, Waypoint cp1, Waypoint cp2,Waypoint cp3, float t);
  //! Function that calculate the distance to deccelerate from target speed
  //! \param target_speed as double, speed on m/s
  //! \return distance on meters
  double DistForSpeed(double target_speed);

  //! Modifies and adds new waypoints to the route for improving the path
  //! \param distance as double, used for the calculation of the new points
  //! \return ERROR if Size is lower than 3, distance <= 0 or the waypoints has already been optimized
  //! \return OK
  ReturnValue Optimize(double distance);

  //! Modifies and adds new waypoints to the route for improving the path
  //! \param distance as double, used for the calculation of the new points
  //! \return ERROR if Size is lower than 4, distance <= 0 or the waypoints has already been optimized
  //! \return OK
  ReturnValue OptimizeByBSpline(double distance);

  //! Prints all the waypoints
  void Print();

};

}

#endif  /*  PATH_ALGORITHM_H_  */
