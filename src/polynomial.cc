#include <mirrax_trajectory_generator/polynomial.h>

Polynomial::Polynomial(ros::NodeHandle& nh)
    : nh_(nh),
      kin_(),
      z_offset_(0.4),
      max_v_(0.1),
      max_a_(0.1),
      max_ang_v_(0.1),
      max_ang_a_(0.1),
      max_wheel_velocity_(3.4),
      derivative_to_optimize(mav_trajectory_generation::derivative_order::ACCELERATION){
        
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[polynomial_node] param max_v not found");
  }
  if (!nh_.getParam("/zero_waypoint_velocity", zero_waypoint_velocity_)){
    ROS_WARN("[polynomial_node] param zero_waypoint_velocity not found, setting to FALSE");
    zero_waypoint_velocity_ = false;
  }

  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory>("trajectory", 0);

  pub_command_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "/command/trajectory", 1);

  // Subscriber to waypoints
  sub_waypoints_ = nh_.subscribe("/waypoints", 1, &Polynomial::waypointCallback, this);

  // n_.param("/robot_name", robot_name_, robot_name_);
  kin_.setKinematicParameters("mini");
  nh_.param("/max_wheel_velocity", max_wheel_velocity_, max_wheel_velocity_);

  const bool oneshot = false;
  const bool autostart = false;
  publish_timer_ = nh_.createTimer(ros::Duration(0.01),
                                   &Polynomial::commandTimerCallback,
                                   this, oneshot, autostart);
}

Polynomial::~Polynomial(){ publish_timer_.stop(); }

// Method to set maximum speed.
void Polynomial::setMaxSpeed(const double max_v, const double max_w) 
{
  max_v_ = max_v;
  max_ang_v_ = max_w;
}

// Set initial pose and velocity
void Polynomial::setStart(const Eigen::VectorXd& pos,
                             const Eigen::VectorXd& vel)
{
  start_position_ = pos;
  start_velocity_ = vel;
}

// Set goal pose and velocity
void Polynomial::setGoal(const Eigen::VectorXd& pos,
                            const Eigen::VectorXd& vel)
{
  goal_position_ = pos;
  goal_velocity_ = vel;
}

void Polynomial::zeroWaypointVelocity(
  const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  ROS_INFO("New zero waypoint velocity trajectory received!");
  
  int ndim = 4;

  std::vector<Eigen::VectorXd> waypoints;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory leg_trajectory;

  std::vector<mav_trajectory_generation::Trajectory> base_traj_vector;
  std::vector<mav_trajectory_generation::Trajectory> leg_traj_vector;

  // Iterate through all waypoints
  for (int i=0; i<msg->points.size()-1; i++)
  {
    printf("Segment: %d \n",i);
    // Resize for base
    goal_position_.resize(ndim);
    goal_velocity_.resize(ndim);
    start_position_.resize(ndim);
    start_velocity_.resize(ndim);

    start_velocity_.setZero();
    goal_velocity_.setZero();

    start_position_ << msg->points[i].positions[0], 
                       msg->points[i].positions[1],
                       msg->points[i].positions[2],
                       msg->points[i].positions[3];

    goal_position_ << msg->points[i+1].positions[0], 
                      msg->points[i+1].positions[1],
                      msg->points[i+1].positions[2],
                      msg->points[i+1].positions[3];
    
    // Generate Base Trajectory
    if (i==0)
    {
      printf("First one \n");
      if (!planTrajectory(waypoints, &trajectory))
      {
        ROS_WARN("Base trajectory generation failed!");
        return;
      }
    }
    else
    {
      mav_trajectory_generation::Trajectory base_seg;
      if (!planTrajectory(waypoints, &base_seg))
      {
        ROS_WARN("Base trajectory generation failed!");
        return;
      }
      base_traj_vector.push_back(base_seg);
    }
    // ----------------------- //
    // Leg Trajectory
    // ----------------------- //
    goal_position_.resize(3);
    goal_velocity_.resize(3);
    start_position_.resize(3);
    start_velocity_.resize(3);
    
    start_position_ << msg->points[i].positions[4], 
                       msg->points[i].positions[5], 0.; 
    goal_position_ << msg->points[i+1].positions[4], 
                      msg->points[i+1].positions[5], 0.05*(i+1); 
    start_velocity_.setZero();
    goal_velocity_.setZero();

    // Generate leg trajectory
    double leg_velocity_ = 0.36;
    if (i==0)
    {
      if (!planTrajectory(
            goal_position_, goal_velocity_, 
            start_position_, start_velocity_, waypoints, 
            leg_velocity_, leg_velocity_, &leg_trajectory))
      {
        ROS_WARN("Leg trajectory generation failed!");
        return;
      }
    }
    else
    {
      mav_trajectory_generation::Trajectory leg_seg;
      if (!planTrajectory(
            goal_position_, goal_velocity_, 
            start_position_, start_velocity_, waypoints, 
            leg_velocity_, leg_velocity_, &leg_seg))
      {
        ROS_WARN("Leg trajectory generation failed!");
        return;
      }
      leg_traj_vector.push_back(leg_seg);
    }
  }
  // Concatenate individual base and leg segments together
  trajectory.addTrajectories(base_traj_vector, &trajectory);
  leg_trajectory.addTrajectories(leg_traj_vector, &leg_trajectory);

  // Merge base and leg together
  trajectory_.clear();
  if (!trajectory.getTrajectoryWithAppendedDimension(
            leg_trajectory, &trajectory_))
  {
    ROS_WARN("Trajectory combining failed!");
    return;
  }
  ROS_INFO("Trajectory generation success!");
  std::cout << "Combined Trajectory time: " << trajectory_.getMaxTime() << std::endl;
  // std::cout << "D: " << trajectory_.D() << std::endl;
  publishTrajectory(trajectory);  // for visualization purpose

  // Trajectory Info
  // std::cout << "Base Trajectory time: " << trajectory.getMaxTime() << std::endl;
  // std::cout << "Leg Trajectory time: " << leg_trajectory.getMaxTime() << std::endl;

  // Start sampling trajectory
  publish_timer_.start();
  start_time_ = ros::Time::now();
  traj_sampler_.current_sample_time_ = 0.0;
  traj_sampler_.final_trigger_ = false;
}

void Polynomial::waypointCallback(
  const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  // zero_waypoint_velocity_ = false;
  if (zero_waypoint_velocity_)
  {
    zeroWaypointVelocity(msg);
    return;
  }

  ROS_INFO("New waypoint trajectory received!");
  
  int ndim = 4;

  // Check if waypoints have non-zero values 
  goal_position_.resize(ndim);
  goal_velocity_.resize(ndim);
  start_position_.resize(ndim);
  start_velocity_.resize(ndim);
  // std::cout << "here 0 "<< std::endl;
  // for (int i=0; i<3; i++)
  //   std::cout << msg->points.back().velocities[i] << std::endl;
  goal_position_ << msg->points.back().positions[0], 
                    msg->points.back().positions[1],
                    msg->points.back().positions[2],
                    msg->points.back().positions[3];
  goal_velocity_.setZero();

  start_position_ << msg->points[0].positions[0], 
                     msg->points[0].positions[1],
                     msg->points[0].positions[2],
                     msg->points[0].positions[3];
  start_velocity_.setZero();
  // std::cout << "here 1 "<< std::endl;
  // Define waypoints
  std::vector<Eigen::VectorXd> waypoints;
  Eigen::VectorXd waypoint(ndim);
  // Iterate through all waypoints
  for (int i=1; i<msg->points.size()-1; i++)
  {
    // std::cout << "here 2 " << i << std::endl;
    // std::cout << "waypoint: " << msg->points[i].positions[0] << std::endl;
    waypoint << msg->points[i].positions[0], 
                msg->points[i].positions[1],
                msg->points[i].positions[2],
                msg->points[i].positions[3];
    waypoints.push_back(waypoint);
  }
  std::cout << "Initial pose: " << start_position_.transpose() << std::endl;
  std::cout << "Goal pose   : " << goal_position_.transpose() << std::endl;
  // for (int i=0; i<msg->points[0].positions.size();i++)
  //   std::cout << "x: " << msg->points[0].positions[i] << std::endl;
  
  // Generate trajectory
  mav_trajectory_generation::Trajectory trajectory;
  if (!planTrajectory(waypoints, &trajectory))
  {
    ROS_WARN("Trajectory generation failed!");
    return;
  }
  
  // Check trajectory feasibility
  if (!checkFeasibility(trajectory))
  {
    ROS_WARN("Trajectory generation failed!");
    return;
  }

  ROS_INFO("Trajectory generation success!");
  publishTrajectory(trajectory);  // for visualization purpose

  // checkRange(trajectory);

  // Trajectory Info
  std::cout << "Base Trajectory time: " << trajectory.getMaxTime() << std::endl;

  // ========================= //
  //    Trajectory for legs    //
  // ========================= //
  goal_position_.resize(3);
  goal_velocity_.resize(3);
  start_position_.resize(3);
  start_velocity_.resize(3);
  
  start_position_ << msg->points[0].positions[4], 
                     msg->points[0].positions[5], 0.; 
  start_velocity_.setZero();
  goal_position_ << msg->points.back().positions[4], 
                    msg->points.back().positions[5], 0.05; 
  goal_velocity_.setZero();

  waypoints.clear();
  waypoint.resize(3);
  float jtemp = 0.05;
  // Iterate through all waypoints
  for (int i=1; i<msg->points.size()-1; i++)
  {
    // Force some values into 3D trajectory to generate fictitious time interval
    jtemp += 0.05;
    waypoint << msg->points[i].positions[4], 
                msg->points[i].positions[5], jtemp;
    waypoints.push_back(waypoint);
  }
  mav_trajectory_generation::Trajectory trajectory_leg;
  bool success = false;
  double leg_velocity_ = 0.36;

  success = planTrajectory(
        goal_position_, goal_velocity_, start_position_, start_velocity_,
        waypoints, leg_velocity_, leg_velocity_, &trajectory_leg);
  if (!success)
  {
    ROS_WARN("Leg trajectory generation failed!");
    return;
  }
  std::cout << "Leg Trajectory time: " << trajectory_leg.getMaxTime() << std::endl;
  
  // Combine base and leg trajectories
  if (!trajectory.getTrajectoryWithAppendedDimension(
            trajectory_leg, &trajectory_))
  {
    ROS_WARN("Trajectory combining failed!");
    return;
  }
  std::cout << "Combined Trajectory time: " << trajectory_.getMaxTime() << std::endl;
  std::cout << "D: " << trajectory_.D() << std::endl;
  // Start sampling trajectory
  publish_timer_.start();
  start_time_ = ros::Time::now();
  traj_sampler_.current_sample_time_ = 0.0;
  traj_sampler_.final_trigger_ = false;
}

// Plans a trajectory from the current position to the a goal position and velocity
bool Polynomial::planTrajectory(
    const std::vector<Eigen::VectorXd>& waypoints,
    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  trajectory->clear();

  // 3 Dimensional trajectory => 3D position
  // 4 Dimensional trajectory => 3D position + yaw
  // 5 Dimensional trajectory => 2D position + yaw + legs position
  // 6 Dimensional trajectory => through SE(3) space, position and orientation
  const int dimension = goal_position_.size();
  bool success = false;

  if (dimension == 6) 
  {
    // printf("Generating 6D Trajectory ... \n");
    // success = planTrajectory(
    //     goal_position_, goal_velocity_, start_position_, start_velocity_,
    //     waypoints, max_v_, max_a_, &(*trajectory));
    mav_trajectory_generation::Trajectory trajectory_trans, trajectory_rot;

    // Split waypoints to translation and rotation components
    std::vector<Eigen::VectorXd> t_points, r_points;
    for(auto const& i: waypoints)
    {
      // std::cout <<"wp: " << i.head(3).transpose() << std::endl;
      t_points.push_back(i.head(3));
      r_points.push_back(i.tail(3));
    }

    // Translation trajectory.
    success = planTrajectory(
        goal_position_.head(3), goal_velocity_.head(3),
        start_position_.head(3), start_velocity_.head(3),
        t_points, max_v_, max_a_, &trajectory_trans);

    // Rotation trajectory.
    success &= planTrajectory(
        goal_position_.tail(3), goal_velocity_.tail(3),
        start_position_.tail(3), start_velocity_.tail(3),
        r_points, max_ang_v_, max_ang_a_, &trajectory_rot);

    // Combine trajectories.
    success &= trajectory_trans.getTrajectoryWithAppendedDimension(
            trajectory_rot, &(*trajectory));
    return success;
  } 
  else if (dimension == 3) 
  {
    printf("Generating 3D Trajectory ... \n");
    success = planTrajectory(
        goal_position_, goal_velocity_, start_position_, start_velocity_,
        waypoints, max_v_, max_a_, &(*trajectory));
    return success;
  } 
  else if (dimension == 4) 
  {
    printf("Generating 4D Trajectory ... \n");
    success = planTrajectory(
        goal_position_, goal_velocity_, start_position_, start_velocity_, 
        waypoints, max_v_, max_a_, &(*trajectory));
    return success;
  }
  else if (dimension == 5) 
  {
    printf("Generating 5D Trajectory ... \n");
    success = planTrajectory(
        goal_position_, goal_velocity_, start_position_, start_velocity_,
        waypoints, max_v_, max_a_, &(*trajectory));
    return success;
  } 
  else 
  {
    LOG(WARNING) << "Dimension must be 3, 4, 5 or 6 to be valid.";
    return false;
  }
}

// Plans a trajectory from a start position and velocity to a goal position and velocity
bool Polynomial::planTrajectory(const Eigen::VectorXd& goal_pos,
                                const Eigen::VectorXd& goal_vel,
                                const Eigen::VectorXd& start_pos,
                                const Eigen::VectorXd& start_vel,
                                const std::vector<Eigen::VectorXd>& waypoints,
                                double v_max, double a_max,
                                mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  const int dimension = goal_pos.size();
  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Initialize vertices
  mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

  /******* Configure start point *******/
  // std::cout << "Start: " << start_pos.transpose() << std::endl;
  start.makeStartOrEnd(start_pos, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      start_vel);
  vertices.push_back(start);

  /******* Configure Mid points, if any *******/
  for(auto const& i: waypoints)
  {
    // std::cout << "Appending waypoints: " << i.transpose() << std::endl;
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, i);
    vertices.push_back(middle);
  }

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  // std::cout << "Goal: " << goal_pos.transpose() << std::endl;
  end.makeStartOrEnd(goal_pos, derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    goal_vel);
  vertices.push_back(end);

  // Estimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 6;

  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  // scale trajectory timing to respect constraint
  trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);
  // std::cout << "bf:" << trajectory->getMaxTime() << std::endl;
  // std::cout << "af:" << trajectory->getMaxTime() << std::endl;
  return true;
}

bool Polynomial::checkFeasibility(const mav_trajectory_generation::Trajectory& trajectory)
{
  ROS_INFO("Checking trajectory feasibility ...");
  // Update robot kinematics i.e. jacobian
  // kin_.updateKinematics(0,0);

  double t_current = 0.0;
  Eigen::MatrixXd v_cmd(5,1);
  Eigen::MatrixXd rot2D(2,2);

  while (t_current < trajectory.getMaxTime())
  {
    Eigen::VectorXd p_setpoint = trajectory.evaluate(t_current, mav_trajectory_generation::derivative_order::POSITION);
    Eigen::VectorXd v_setpoint = trajectory.evaluate(t_current, mav_trajectory_generation::derivative_order::VELOCITY);

    // Transform velocity to robot frame
    double yaw = p_setpoint(5);
    rot2D << cos(yaw), sin(yaw), -sin(yaw), cos(yaw);
    v_cmd << rot2D*v_setpoint.head(2), v_setpoint(5), 0, 0;

    auto w_cmd = kin_.jac_wheel_*v_cmd;
    auto w_abs = w_cmd.cwiseAbs();
    // // std::cout << v_setpoint.head(2).transpose() << std::endl;
    // // std::cout << vout.transpose() << std::endl;
    // std::cout << v_cmd.transpose() << std::endl;
    // std::cout << w_cmd.transpose() << std::endl;
    // // std::cout << w_abs.transpose() << std::endl;
    // std::cout << "===========================" << std::endl;
    // std::cout << abs_w.maxCoeff() << std::endl;
    auto w_max = w_abs.maxCoeff();
    if (w_max > max_wheel_velocity_)
    {
      ROS_WARN("Wheel velocity exceeded %.2f\n", w_max);
      return false;
    }

    t_current+= 0.1;
  }
  ROS_INFO("Trajectory feasibile!");
  
  return true;
}

bool Polynomial::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);

  return true;
}

// TEMP FUNCTION FOR EXPERIMENTS
void Polynomial::checkRange(
  const mav_trajectory_generation::Trajectory& trajectory)
{
  // List of joint angles to assess
  Eigen::MatrixXd joint_config(18,2);
  joint_config << 0, 0,
                  20, -20,
                  64, -64,
                  90, -90,
                  107, -107,
                  126, -127,
                  180, -180,
                  45, -63,
                  5, -40,
                  40, -5,
                  0, -179,
                  179, 0,
                  65, -120,
                  120, -65,
                  115, -160,
                  160, -115,
                  161, -171,
                  171, -161;

  for (int i=0; i<joint_config.rows(); i++)
  {
    std::cout << "q: " << joint_config.row(i) << std::endl;
    double j5 = joint_config(i,0) * M_PI/180.0;
    double j6 = joint_config(i,1) * M_PI/180.0;
    kin_.updateKinematics(j5,j6);
    checkFeasibility(trajectory);
  }  
}

void Polynomial::commandTimerCallback(const ros::TimerEvent&)
{
  static int empty_count = 0;
  double sample_time = 0;

  // std::cout << traj_sampler_.current_sample_time_ << "  " << trajectory_.getMaxTime() << std::endl;
  
  if (traj_sampler_.final_trigger_ == false)
  {
    // Publish first and last point - dragging it for a bit
    if (empty_count==0)
      ROS_INFO("Trajectory Sampler: Dragging setpoint ...");
    // Setpoint is either first or last time for fixed amount of time
    if (traj_sampler_.current_sample_time_ < 1)
      sample_time = 0;
    else if (traj_sampler_.current_sample_time_ > trajectory_.getMaxTime())
      sample_time = trajectory_.getMaxTime();

    trajectory_msgs::MultiDOFJointTrajectory msg;
    trajectoryToMsg(sample_time, msg);
    pub_command_.publish(msg);
    // Increment counter & reset counter and flags when finished
    empty_count++;
    // std::cout << empty_count << std::endl;
    if (empty_count > int(1/traj_sampler_.dt_))
    {
      empty_count = 0;
      traj_sampler_.final_trigger_ = true;
      if (traj_sampler_.current_sample_time_ < trajectory_.getMaxTime())
        ROS_INFO("Trajectory Sampler: Publishing trajectory setpoints ...");
    }
  }
  else if (traj_sampler_.current_sample_time_ <= trajectory_.getMaxTime())
  {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    trajectoryToMsg(traj_sampler_.current_sample_time_, msg);
    pub_command_.publish(msg);

    traj_sampler_.current_sample_time_ += traj_sampler_.dt_;
    // Trigger end setpoint  
    if (traj_sampler_.current_sample_time_ > trajectory_.getMaxTime())
      traj_sampler_.final_trigger_ = false;
  }
  else
  {
    ROS_INFO("Trajectory Sampler: Finished!");
    publish_timer_.stop();
  }
  
}

void Polynomial::trajectoryToMsg(double sample_time, trajectory_msgs::MultiDOFJointTrajectory& msg)
{
  msg.header.stamp = ros::Time::now();
  msg.joint_names.push_back("base_link");
  msg.joint_names.push_back("j5");
  msg.joint_names.push_back("j6");
  
  Eigen::VectorXd pos = trajectory_.evaluate(sample_time, traj_sampler_.pos_dorder);
  Eigen::VectorXd vel = trajectory_.evaluate(sample_time, traj_sampler_.vel_dorder);
  Eigen::VectorXd acc = trajectory_.evaluate(sample_time, traj_sampler_.acc_dorder);
  // std::cout << pos.size() << "\t" << pos.transpose() << std::endl;
  trajectory_msgs::MultiDOFJointTrajectoryPoint point;

  geometry_msgs::Transform position;
  geometry_msgs::Twist velocity;
  geometry_msgs::Twist acceleration;

  // ============ Base Link ============ //
  position.translation.x = pos(0);
  position.translation.y = pos(1);
  position.translation.z = pos(2);
  tf2::Quaternion quat;
  quat.setRPY( 0, 0, pos(3) );
  quat.normalize();
  tf2::convert(quat, position.rotation);

  velocity.linear.x = vel(0);
  velocity.linear.y = vel(1);
  velocity.linear.z = vel(2);
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = vel(3);

  acceleration.linear.x = acc(0);
  acceleration.linear.y = acc(1);
  acceleration.linear.z = acc(2);
  acceleration.angular.x = 0.0;
  acceleration.angular.y = 0.0;
  acceleration.angular.z = acc(3);

  point.transforms.push_back(position);
  point.velocities.push_back(velocity);
  point.accelerations.push_back(acceleration);

  // ============ J5 joint ============ //
  position.translation.x = pos(4);
  position.translation.y = 0.0;
  position.translation.z = 0.0;
  quat.setRPY( 0, 0, 0 );
  quat.normalize();
  tf2::convert(quat, position.rotation);

  velocity.linear.x = vel(4);
  velocity.linear.y = 0.0;
  velocity.linear.z = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;

  acceleration.linear.x = acc(4);
  acceleration.linear.y = 0.0;
  acceleration.linear.z = 0.0;
  acceleration.angular.x = 0.0;
  acceleration.angular.y = 0.0;
  acceleration.angular.z = 0.0;

  point.transforms.push_back(position);
  point.velocities.push_back(velocity);
  point.accelerations.push_back(acceleration);

  // ============ J6 joint ============ //
  position.translation.x = pos(5);
  position.translation.y = 0.0;
  position.translation.z = 0.0;
  quat.setRPY( 0, 0, 0 );
  quat.normalize();
  tf2::convert(quat, position.rotation);

  velocity.linear.x = vel(5);
  velocity.linear.y = 0.0;
  velocity.linear.z = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;

  acceleration.linear.x = acc(5);
  acceleration.linear.y = 0.0;
  acceleration.linear.z = 0.0;
  acceleration.angular.x = 0.0;
  acceleration.angular.y = 0.0;
  acceleration.angular.z = 0.0;

  point.transforms.push_back(position);
  point.velocities.push_back(velocity);
  point.accelerations.push_back(acceleration);

  msg.points.push_back(point);
  msg.points[0].time_from_start = ros::Duration(sample_time);
}