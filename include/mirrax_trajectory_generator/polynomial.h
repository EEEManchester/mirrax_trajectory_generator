#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <iostream>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <mirrax_trajectory_generator/kinematics.h>

struct TrajectorySampler
{
  TrajectorySampler()
  {
    dt_ = 0.01;
    current_sample_time_ = 0.0;
    pos_dorder = mav_trajectory_generation::derivative_order::POSITION;
    vel_dorder = mav_trajectory_generation::derivative_order::VELOCITY;
    acc_dorder = mav_trajectory_generation::derivative_order::ACCELERATION;
  }
  double dt_;
  double current_sample_time_;
  int pos_dorder;
  int vel_dorder;
  int acc_dorder;
  bool final_trigger_;
};

class Polynomial {
 public:
  Polynomial(ros::NodeHandle& nh);
  ~Polynomial();

  // Sets trajectory parameters
  void setMaxSpeed(const double max_v, const double max_w);
  void setStart(const Eigen::VectorXd& pos,
                const Eigen::VectorXd& vel);
  void setGoal(const Eigen::VectorXd& pos,
               const Eigen::VectorXd& vel);

  // Subscriber to waypoints. Generates trajectory on receiving waypoint
  // if zero_waypoint_velocity is false.
  // Generates base and leg trajectories seperately, then merge together.
  // On trajectory generation success, starts timer for sampling trajectory
  void waypointCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

  // Generate trajectory where each waypoint has zero velocity. Otherwise,
  // similar to waypointCallback function. 
  void zeroWaypointVelocity(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

  // Check trajectory feasibiliy. Iterates through trajectory at 0.1s interval.
  // Trajectory is scaled according to ratio of wheel velocity exceeded. Feasibility
  // check is repeated for a fixed number of times (not within this function) 
  bool checkFeasibility(const mav_trajectory_generation::Trajectory& trajectory);

  // Selects trajectory dimension based on waypoints received
  bool planTrajectory(const std::vector<Eigen::VectorXd>& waypoints,
                      mav_trajectory_generation::Trajectory* trajectory);

  // Plans a trajectory from start to goal position through the waypoints                 
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      const std::vector<Eigen::VectorXd>& waypoints,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);

  // Publishes the trajectory as segment type                    
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);
  
  // Sample trajectory at fixed time interval and publish for robot to track 
  void commandTimerCallback(const ros::TimerEvent&);
  void trajectoryToMsg(double sample_time, trajectory_msgs::MultiDOFJointTrajectory& msg);

  // TEMP: Checks to ensure velocity is satisfied for a range for fixed configuration
  // For experiment at the moment. 
  void checkRange(const mav_trajectory_generation::Trajectory& trajectory);

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_waypoints_;

  ros::NodeHandle& nh_;
  Eigen::Affine3d current_pose_;
  
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_; // rad/s
  double max_ang_a_;
  double max_leg_v_;
  double max_wheel_v_;
  double z_offset_;

  Eigen::VectorXd start_position_;
  Eigen::VectorXd start_velocity_;
  Eigen::VectorXd goal_position_;
  Eigen::VectorXd goal_velocity_;

  // Optimze up to 4th order derivative (POSITION, VELOCITY, ACCELERATION, SNAP)
  const int derivative_to_optimize;

  mirrax::Kinematics *kin_;

  TrajectorySampler traj_sampler_;
  ros::Timer publish_timer_;
  ros::Time start_time_;
  ros::Publisher pub_command_;
  mav_trajectory_generation::Trajectory trajectory_;
  
  bool zero_waypoint_velocity_;
  std::string frame_id_;
};

#endif // POLYNOMIAL_H