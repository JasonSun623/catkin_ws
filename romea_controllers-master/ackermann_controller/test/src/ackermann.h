// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// NaN
#include <limits>

// ostringstream
#include <sstream>

class Ackermann : public hardware_interface::RobotHW
{
public:
  Ackermann()
  : running_(true)
  , start_srv_(nh_.advertiseService("start", &Ackermann::start_callback, this))
  , stop_srv_(nh_.advertiseService("stop", &Ackermann::stop_callback, this))
  {
    std::vector<std::string> velocity_joints_name = {"front_left_wheel",
                                                     "rear_left_wheel", "rear_right_wheel"};
    // Connect and register the joint state and velocity interface
    for (unsigned int i = 0; i < 1; ++i)
    {

      hardware_interface::JointStateHandle state_handle(velocity_joints_name[i], &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(state_handle, &joints_[i].velocity_command);
      jnt_vel_interface_.registerHandle(vel_handle);
    }
    // for rear wheel state
     for (unsigned int i = 1; i < velocity_joints_name.size(); ++i)
    {
      hardware_interface::JointStateHandle state_handle(velocity_joints_name[i], &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
      jnt_state_interface_.registerHandle(state_handle);
    }

    std::vector<std::string> position_joints_name = {"front_left_steering_joint"};
    // Connect and register the joint state and position interface
    for (unsigned int i = 0; i < 1; ++i)
    {
      hardware_interface::JointStateHandle state_handle(position_joints_name[i], &steering_joints_[i].position, &steering_joints_[i].velocity, &steering_joints_[i].effort);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(state_handle, &steering_joints_[i].position_command);
      jnt_pos_interface_.registerHandle(pos_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);
    registerInterface(&jnt_pos_interface_);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}
 //实时读入控制指令
  void read()
  {
    std::ostringstream os;
    for (unsigned int i = 0; i < 1; ++i)
    {
      os << joints_[i].velocity_command << ", ";
    }

    ROS_DEBUG_STREAM("Commands for joints: " << os.str());

    os.str("");
    os << steering_joints_[0].position_command << ", ";

    ROS_DEBUG_STREAM("Commands for steering joints: " << os.str());
  }
  //根据读入指令，推算position和更新velocity,steer_angle
  void write()
  {
    if (running_)
    {
      for (unsigned int i = 0; i < 1; ++i)
      {
        // Note that joints_[i].position will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.
        joints_[i].position += joints_[i].velocity*getPeriod().toSec(); // update position
        ROS_INFO_STREAM("ackermann.write.joint[" << i<<"] position: " <<  joints_[i].position);
        joints_[i].velocity = joints_[i].velocity_command; // might add smoothing here later
      }
      for (unsigned int i = 0; i < 1; ++i)
      {
        steering_joints_[i].position = steering_joints_[i].position_command; // might add smoothing here later
      }
    }
    else
    {
      for (unsigned int i = 0; i < 1; ++i)
      {
        joints_[i].position = std::numeric_limits<double>::quiet_NaN();
        joints_[i].velocity = std::numeric_limits<double>::quiet_NaN();
      }
      for (unsigned int i = 0; i < 1; ++i)
      {
        steering_joints_[i].position = std::numeric_limits<double>::quiet_NaN();
        steering_joints_[i].velocity = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  struct Joint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0) { }
  } joints_[3];

  struct SteeringJoint
  {
    double position;
    double velocity;
    double effort;
    double position_command;

    SteeringJoint() : position(0), velocity(0), effort(0), position_command(0) { }
  } steering_joints_[1];
  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
