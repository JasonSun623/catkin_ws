#ifndef PATH_TRACKING_COMM_H_
#define PATH_TRACKING_COMM_H_


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

#define WAYPOINT_POP_DISTANCE_M              0.10    //Distancia mínima para alcanzar punto objetivo m (PurePursuit)

#define AGVS_FIRST_DECELERATION_DISTANCE     0.5   // meters -> when the vehicle is arriving to the goal, it has to decelarate at this distance
#define AGVS_FIRST_DECELERATION_MAXSPEED     0.15  // m/s
#define AGVS_SECOND_DECELERATION_DISTANCE    0.25   // meters -> when the vehicle is arriving to the goal, it has to decelarate another time at this distance
#define AGVS_SECOND_DECELERATION_MAXSPEED    0.1   // m/s
#define AGVS_DEFAULT_KR                      0.20            //


#define COMMAND_ACKERMANN              100
#define COMMAND_TWIST                  200
#define COMMAND_ACKERMANN_STRING       "Ackermann"
#define COMMAND_TWIST_STRING           "Twist"

#define DEFAULT_OBSTACLE_RANGE 				1.0
#define DEFAULT_FOOTPRINT_WIDTH 			0.6
#define DEFAULT_FOOTPRINT_LENGTH 			1.0
#define DEFAULT_LATERAL_CLEARANCE 			0.5


#define GOAL_ERROR_TOLERANCE				0.05
#define ERROR_GOAL_DISTANCE					0.10


//chq 切换算法rviz显示
enum look_dir{
  look_back = 0,//瞄准起点后方
  lool_ahead =1 //瞄准起点前方
};

enum{
  ODOM_SOURCE = 1,
  MAP_SOURCE = 2
};



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
    //! Index
    int index;
}Waypoint;

#endif  /*  PATH_TRACKING_COMM_H_  */
