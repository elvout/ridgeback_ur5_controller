#include "AdmittanceController.h"

using std::placeholders::_1;

AdmittanceController::AdmittanceController(double frequency)
    : Node("admittance_control"), loop_rate_(frequency) {
  // Parameters
  world_frame_id_ = this->declare_parameter<std::string>("world_frame_id");
  const std::string topic_platform_command =
      this->declare_parameter<std::string>("topic_platform_command");
  const std::string topic_platform_state =
      this->declare_parameter<std::string>("topic_platform_state");
  const std::string topic_arm_command = this->declare_parameter<std::string>("topic_arm_command");
  // const std::string topic_arm_state = this->declare_parameter<std::string>("topic_arm_state");
  const std::string topic_arm_pose_world =
      this->declare_parameter<std::string>("topic_arm_pose_world");
  const std::string topic_arm_twist_world =
      this->declare_parameter<std::string>("topic_arm_twist_world");
  const std::string topic_external_wrench_arm_frame =
      this->declare_parameter<std::string>("topic_external_wrench_arm_frame");
  const std::string topic_control_wrench_arm_frame =
      this->declare_parameter<std::string>("topic_control_wrench_arm_frame");
  const std::string topic_external_wrench =
      this->declare_parameter<std::string>("topic_external_wrench");
  const std::string topic_control_wrench =
      this->declare_parameter<std::string>("topic_control_wrench");
  const std::string topic_admittance_ratio =
      this->declare_parameter<std::string>("topic_admittance_ratio");
  const std::string topic_equilibrium_desired =
      this->declare_parameter<std::string>("topic_equilibrium_desired");
  const std::string topic_equilibrium_real =
      this->declare_parameter<std::string>("topic_equilibrium_real");
  const std::string topic_ds_velocity = this->declare_parameter<std::string>("topic_ds_velocity");
  const std::vector<double> M_p = this->declare_parameter<std::vector<double>>("mass_platform");
  const std::vector<double> M_a = this->declare_parameter<std::vector<double>>("mass_arm");
  const std::vector<double> D = this->declare_parameter<std::vector<double>>("damping_coupling");
  const std::vector<double> D_p = this->declare_parameter<std::vector<double>>("damping_platform");
  const std::vector<double> D_a = this->declare_parameter<std::vector<double>>("damping_arm");
  const std::vector<double> K = this->declare_parameter<std::vector<double>>("stiffness_coupling");
  const std::vector<double> d_e =
      this->declare_parameter<std::vector<double>>("equilibrium_point_spring");
  const std::vector<double> workspace_limits =
      this->declare_parameter<std::vector<double>>("workspace_limits");
  arm_max_vel_ = this->declare_parameter<double>("arm_max_vel");
  arm_max_acc_ = this->declare_parameter<double>("arm_max_acc");
  platform_max_vel_ = this->declare_parameter<double>("platform_max_vel");
  platform_max_acc_ = this->declare_parameter<double>("platform_max_acc");
  wrench_filter_factor_ = this->declare_parameter<double>("wrench_filter_factor");
  force_dead_zone_thres_ = this->declare_parameter<double>("force_dead_zone_thres");
  torque_dead_zone_thres_ = this->declare_parameter<double>("torque_dead_zone_thres");

  // Subscribers
  sub_platform_state_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_platform_state, 5, std::bind(&AdmittanceController::state_platform_callback, this, _1));
  // sub_arm_state_ = this->create_subscription<cartesian_state_msgs::msg::PoseTwist>(
  //     topic_arm_state, 10, std::bind(&AdmittanceController::state_arm_callback, this, _1));
  sub_wrench_external_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topic_external_wrench, 5, std::bind(&AdmittanceController::wrench_callback, this, _1));
  sub_wrench_control_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topic_control_wrench, 5, std::bind(&AdmittanceController::wrench_control_callback, this, _1));
  sub_equilibrium_desired_ = this->create_subscription<geometry_msgs::msg::Point>(
      topic_equilibrium_desired, 10,
      std::bind(&AdmittanceController::equilibrium_callback, this, _1));
  sub_ds_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      topic_ds_velocity, 10, std::bind(&AdmittanceController::ds_velocity_callback, this, _1));
  sub_admittance_ratio_ = this->create_subscription<std_msgs::msg::Float32>(
      topic_admittance_ratio, 10,
      std::bind(&AdmittanceController::admittance_ratio_callback, this, _1));

  // Publishers
  pub_platform_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_platform_command, 5);
  pub_arm_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_arm_command, 5);

  pub_ee_pose_world_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_arm_pose_world, 5);
  pub_ee_twist_world_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_arm_twist_world, 5);

  pub_wrench_external_ =
      this->create_publisher<geometry_msgs::msg::WrenchStamped>(topic_external_wrench_arm_frame, 5);
  pub_wrench_control_ =
      this->create_publisher<geometry_msgs::msg::WrenchStamped>(topic_control_wrench_arm_frame, 5);

  pub_equilibrium_real_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(topic_equilibrium_real, 5);

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Arm max vel:" << arm_max_vel_ << " max acc:" << arm_max_acc_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Platform max vel:" << platform_max_vel_ << " max acc:" << platform_max_acc_);

  // initializing the class variables
  wrench_external_.setZero();
  wrench_control_.setZero();

  M_p_ = Vector6d(M_p.data()).asDiagonal();
  M_a_ = Vector6d(M_a.data()).asDiagonal();
  D_ = Vector6d(D.data()).asDiagonal();
  D_p_ = Vector6d(D_p.data()).asDiagonal();
  D_a_ = Vector6d(D_a.data()).asDiagonal();
  K_ = Vector6d(K.data()).asDiagonal();

  workspace_limits_ = Vector6d(workspace_limits.data());

  ee_pose_world_.setZero();
  ee_twist_world_.setZero();

  // setting the equilibrium position and orientation
  Vector7d equilibrium_full(d_e.data());
  equilibrium_position_ << equilibrium_full.topRows(3);

  // This does not change
  equilibrium_position_seen_by_platform << equilibrium_full.topRows(3);
  // Make sure the orientation goal is normalized
  equilibrium_orientation_.coeffs()
      << equilibrium_full.bottomRows(4) / equilibrium_full.bottomRows(4).norm();

  equilibrium_new_.setZero();

  // starting from a state that does not create movements on the robot
  arm_real_orientation_ = equilibrium_orientation_;

  // setting the robot state to zero and wait for data
  arm_real_position_.setZero();
  platform_real_position_.setZero();

  // Init integrator
  arm_desired_twist_adm_.setZero();
  platform_desired_twist_.setZero();

  arm_desired_twist_ds_.setZero();
  arm_desired_twist_final_.setZero();

  // Kinematic constraints between base and arm at the equilibrium
  // the base only effected by arm in x,y and rz
  kin_constraints_.setZero();
  kin_constraints_.topLeftCorner(2, 2).setIdentity();
  kin_constraints_.bottomRightCorner(1, 1).setIdentity();

  // kin_constraints_.setZero();
  // kin_constraints_.topLeftCorner(3, 3).setIdentity();
  // kin_constraints_.bottomRightCorner(3, 3).setIdentity();
  // Screw on the z torque axis
  // kin_constraints_.topRightCorner(3, 3) <<
  //                                       0, 0, equilibrium_position_(1),
  //                                       0, 0, -equilibrium_position_(0),
  //                                       0, 0, 0;

  ft_arm_ready_ = false;
  arm_world_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  admittance_ratio_ = 1;
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::run() {
  RCLCPP_INFO(this->get_logger(), "Running the admittance control loop .................");

  while (rclcpp::ok()) {
    // Admittance Dynamics computation
    compute_admittance();

    // sum the vel from admittance to DS in this function
    // limit the the movement of the arm to the permitted workspace
    limit_to_workspace();

    // Copy commands to messages
    send_commands_to_robot();

    // Arm pose/twist in the world frame
    publish_arm_state_in_world();

    // publishing visualization/debugging info
    publish_debuggings_signals();

    rclcpp::spin_some(this->shared_from_this());
    loop_rate_.sleep();
  }
}

///////////////////////////////////////////////////////////////
///////////////////// Admittance Dynamics /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::compute_admittance() {
  Vector6d platform_desired_acceleration;
  Vector6d arm_desired_accelaration;

  Vector6d error;

  this->update_arm_state();

  // Orientation error w.r.t. desired equilibriums
  if (equilibrium_orientation_.coeffs().dot(arm_real_orientation_.coeffs()) < 0.0) {
    arm_real_orientation_.coeffs() << -arm_real_orientation_.coeffs();
  }

  Eigen::Quaterniond quat_rot_err(arm_real_orientation_ * equilibrium_orientation_.inverse());
  if (quat_rot_err.coeffs().norm() > 1e-3) {
    // Normalize error quaternion
    quat_rot_err.coeffs() << quat_rot_err.coeffs() / quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();

  // Translation error w.r.t. desired equilibrium
  error.topRows(3) = arm_real_position_ - equilibrium_position_seen_by_platform;
  Vector6d coupling_wrench_platform = D_ * (arm_desired_twist_adm_) + K_ * error;

  error.topRows(3) = arm_real_position_ - equilibrium_position_;
  Vector6d coupling_wrench_arm = D_ * (arm_desired_twist_adm_) + K_ * error;

  platform_desired_acceleration =
      M_p_.inverse() * (-D_p_ * platform_desired_twist_ +
                        rotation_base_ * kin_constraints_ * coupling_wrench_platform);
  arm_desired_accelaration =
      M_a_.inverse() * (-coupling_wrench_arm - D_a_ * arm_desired_twist_adm_ +
                        admittance_ratio_ * wrench_external_ + wrench_control_);

  // limiting the accelaration for better stability and safety
  // x and y for  platform and x,y,z for the arm
  double p_acc_norm = (platform_desired_acceleration.segment(0, 2)).norm();
  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();

  if (p_acc_norm > platform_max_acc_) {
    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Admittance generates high platform accelaration !" << " norm : " << p_acc_norm);
    platform_desired_acceleration.segment(0, 2) *= (platform_max_acc_ / p_acc_norm);
  }

  if (a_acc_norm > arm_max_acc_) {
    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Admittance generates high arm accelaration!" << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  // Integrate for velocity based interface
  // Reassignment is necessary to cast to seconds.
  const std::chrono::duration<double> loop_cycle_period_sec = loop_rate_.period();

  platform_desired_twist_ += platform_desired_acceleration * loop_cycle_period_sec.count();
  arm_desired_twist_adm_ += arm_desired_accelaration * loop_cycle_period_sec.count();
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::state_platform_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  platform_real_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z;
  platform_real_orientation_.coeffs() << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;

  platform_real_twist_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
      msg->twist.twist.linear.z, msg->twist.twist.angular.x, msg->twist.twist.angular.y,
      msg->twist.twist.angular.z;
}

/*
void AdmittanceController::state_arm_callback(
  const cartesian_state_msgs::msg::PoseTwist::SharedPtr msg) {
    arm_real_position_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    arm_real_orientation_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w;

    arm_real_twist_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
    msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
  }
*/

bool AdmittanceController::update_arm_state() {
  const moveit::core::RobotStatePtr robot_state =
      planning_scene_monitor_->getStateMonitor()->getCurrentState();

  const Eigen::Isometry3d& T_baselink_ur10ebaselink =
      robot_state->getGlobalLinkTransform("ur10ebase_link");
  const Eigen::Isometry3d& T_baselink_vg10graspcenter =
      robot_state->getGlobalLinkTransform("vg10_grasp_center");
  const Eigen::Isometry3d T_ur10ebaselink_vg10graspcenter =
      T_baselink_ur10ebaselink.inverse() * T_baselink_vg10graspcenter;

  arm_real_position_ = T_ur10ebaselink_vg10graspcenter.translation();
  arm_real_orientation_ = Eigen::Quaterniond(T_ur10ebaselink_vg10graspcenter.rotation());

  const moveit::core::JointModelGroup* joint_group =
      robot_state->getJointModelGroup("ur_manipulator_with_vg10");
  const moveit::core::LinkModel* ee_link =
      robot_state->getRobotModel()->getLinkModel("vg10_grasp_center");

  Eigen::VectorXd joint_velocities(joint_group->getVariableCount());
  robot_state->copyJointGroupVelocities(joint_group, joint_velocities);

  Eigen::MatrixXd jacobian;
  const bool jacobian_valid =
      robot_state->getJacobian(joint_group, ee_link, Eigen::Vector3d::Zero(), jacobian);

  if (!jacobian_valid) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "update_arm_state: invalid jacobian");
    arm_real_twist_.setZero();
    return false;
  }

  const Eigen::VectorXd twist = jacobian * joint_velocities;
  if (twist.size() != 6) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "update_arm_state: twist.size != 6");
    return false;
  }

  arm_real_twist_ = twist;
  return true;
}

void AdmittanceController::wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (ft_arm_ready_) {
    // Reading the FT-sensor in its own frame (robotiq_force_torque_frame_id)
    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
        msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

    // Dead zone for the FT sensor
    // if (wrench_ft_frame.topRows(3).norm() < force_dead_zone_thres_) {
    //   wrench_ft_frame.topRows(3).setZero();
    // }
    // if (wrench_ft_frame.bottomRows(3).norm() < torque_dead_zone_thres_) {
    //   wrench_ft_frame.bottomRows(3).setZero();
    // }

    for (int i = 0; i < 3; i++) {
      if (abs(wrench_ft_frame(i)) < force_dead_zone_thres_) {
        wrench_ft_frame(i) = 0;
      }
      if (abs(wrench_ft_frame(i + 3)) < torque_dead_zone_thres_) {
        wrench_ft_frame(i + 3) = 0;
      }
    }

    // Get transform from arm base link to platform base link
    get_rotation_matrix(rotation_ft_base, "ur10ebase_link", "ur10etool0");

    // Filter and update
    wrench_external_ << (1 - wrench_filter_factor_) * wrench_external_ +
                            wrench_filter_factor_ * rotation_ft_base * wrench_ft_frame;
  }
}

void AdmittanceController::wrench_control_callback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  if (msg->header.frame_id == "ur10ebase_link") {
    wrench_control_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
        msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  } else {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "wrench_control_callback: The frame_id is not specified as ur10ebase_link");
  }
}

void AdmittanceController::ds_velocity_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  arm_desired_twist_ds_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
  // ROS_INFO_STREAM_THROTTLE(1,"received velocity, z:" << arm_desired_twist_ds_(2));
}
void AdmittanceController::equilibrium_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
  equilibrium_new_ << msg->x, msg->y, msg->z;

  // bool equ_update = true;
  // if (equilibrium_new_(0) < workspace_limits_(0) || equilibrium_new_(0) > workspace_limits_(1)) {
  //   ROS_WARN_STREAM_THROTTLE (1, "Desired equilibrium is out of workspace.  x = "
  //                             << equilibrium_new_(0) << " not in [" << workspace_limits_(0) << "
  //                             , "
  //                             << workspace_limits_(1) << "]");
  //   equ_update = false;
  // }

  // if (equilibrium_new_(1) < workspace_limits_(2) || equilibrium_new_(1) > workspace_limits_(3)) {
  //   ROS_WARN_STREAM_THROTTLE (1, "Desired equilibrium is out of workspace.  y = "
  //                             << equilibrium_new_(1) << " not in [" << workspace_limits_(2) << "
  //                             , "
  //                             << workspace_limits_(3) << "]");
  //   equ_update = false;
  // }

  // if (equilibrium_new_(2) < workspace_limits_(4) || equilibrium_new_(2) > workspace_limits_(5)) {
  //   ROS_WARN_STREAM_THROTTLE (1, "Desired equilibrium is out of workspace.  x = "
  //                             << equilibrium_new_(2) << " not in [" << workspace_limits_(4) << "
  //                             , "
  //                             << workspace_limits_(5) << "]");
  //   equ_update = false;
  // }

  // if (equ_update) {
  //   equilibrium_position_ = equilibrium_new_;
  //   // ROS_INFO_STREAM_THROTTLE(2, "New eauiibrium at : " <<
  //   //                          equilibrium_position_(0) << " " <<
  //   //                          equilibrium_position_(1) << " " <<
  //   //                          equilibrium_position_(2)   );
  // }

  if (equilibrium_new_(0) > workspace_limits_(0) && equilibrium_new_(0) < workspace_limits_(1)) {
    equilibrium_position_(0) = equilibrium_new_(0);
  }

  if (equilibrium_new_(1) > workspace_limits_(2) && equilibrium_new_(1) < workspace_limits_(3)) {
    equilibrium_position_(1) = equilibrium_new_(1);
  }

  if (equilibrium_new_(2) > workspace_limits_(4) && equilibrium_new_(2) < workspace_limits_(5)) {
    equilibrium_position_(2) = equilibrium_new_(2);
  }
}

void AdmittanceController::admittance_ratio_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  double h = msg->data;

  if (h > 1) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Admittance ration higher than one is recieved " << h);
    h = 1;
  } else if (h < 0) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Admittance ratio lower than zero is recieved " << h);
    h = 0;
  } else {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Admittance ratio between 0 and 1 recieved " << h);
  }

  admittance_ratio_ = h;
}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::send_commands_to_robot() {
  // for the platform
  geometry_msgs::msg::Twist platform_twist_cmd;

  platform_twist_cmd.linear.x = platform_desired_twist_(0);
  platform_twist_cmd.linear.y = platform_desired_twist_(1);
  platform_twist_cmd.linear.z = platform_desired_twist_(2);
  platform_twist_cmd.angular.x = platform_desired_twist_(3);
  platform_twist_cmd.angular.y = platform_desired_twist_(4);
  platform_twist_cmd.angular.z = platform_desired_twist_(5);

  pub_platform_cmd_->publish(platform_twist_cmd);

  // for the arm
  const moveit::core::RobotStatePtr robot_state =
      planning_scene_monitor_->getStateMonitor()->getCurrentState();
  const moveit_servo::TwistCommand command{"vg10_grasp_center", arm_desired_twist_final_};
  const moveit_servo::KinematicState next_state = servo_->getNextJointState(robot_state, command);

  std_msgs::msg::Float64MultiArray arm_vel_cmd =
      moveit_servo::composeMultiArrayMessage(servo_->getParams(), next_state);
  pub_arm_cmd_->publish(arm_vel_cmd);
}

void AdmittanceController::limit_to_workspace() {
  if (arm_real_position_(0) < workspace_limits_(0) ||
      arm_real_position_(0) > workspace_limits_(1)) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Out of permitted workspace.  x = "
                                    << arm_real_position_(0) << " not in [" << workspace_limits_(0)
                                    << " , " << workspace_limits_(1) << "]");
  }

  if (arm_real_position_(1) < workspace_limits_(2) ||
      arm_real_position_(1) > workspace_limits_(3)) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Out of permitted workspace.  y = "
                                    << arm_real_position_(1) << " not in [" << workspace_limits_(2)
                                    << " , " << workspace_limits_(3) << "]");
  }

  if (arm_real_position_(2) < workspace_limits_(4) ||
      arm_real_position_(2) > workspace_limits_(5)) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Out of permitted workspace.  z = "
                                    << arm_real_position_(2) << " not in [" << workspace_limits_(4)
                                    << " , " << workspace_limits_(5) << "]");
  }

  arm_desired_twist_final_ = arm_desired_twist_adm_;
  // arm_desired_twist_final_.segment(0,3) += (1- admittance_ratio_) * arm_desired_twist_ds_;
  arm_desired_twist_final_.segment(0, 3) += arm_desired_twist_ds_;

  if (arm_desired_twist_final_(0) < 0 && arm_real_position_(0) < workspace_limits_(0)) {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(0) > 0 && arm_real_position_(0) > workspace_limits_(1)) {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(1) < 0 && arm_real_position_(1) < workspace_limits_(2)) {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(1) > 0 && arm_real_position_(1) > workspace_limits_(3)) {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(2) < 0 && arm_real_position_(2) < workspace_limits_(4)) {
    arm_desired_twist_final_(2) = 0;
  }

  if (arm_desired_twist_final_(2) > 0 && arm_real_position_(2) > workspace_limits_(5)) {
    arm_desired_twist_final_(2) = 0;
  }

  // velocity of the arm along x, y, and z axis
  double norm_vel_des = (arm_desired_twist_final_.segment(0, 3)).norm();

  if (norm_vel_des > arm_max_vel_) {
    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);

    arm_desired_twist_final_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);
  }

  // velocity of the platfrom only along x and y axis
  double norm_vel_platform = (platform_desired_twist_.segment(0, 2)).norm();

  if (norm_vel_platform > platform_max_vel_) {
    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Admittance generate fast platform movements! velocity norm: " << norm_vel_platform);

    platform_desired_twist_.segment(0, 2) *= (platform_max_vel_ / norm_vel_platform);
  }
}

//////////////////////
/// INITIALIZATION ///
//////////////////////
void AdmittanceController::wait_for_transformations() {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this->shared_from_this());

  Matrix6d rot_matrix;
  rotation_base_.setZero();

  while (!update_arm_state()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Waiting for the state of the arm...");
    sleep(1);
  }

  // Makes sure all TFs exists before enabling all transformations in the callbacks
  while (!get_rotation_matrix(rotation_base_, "base_link", "ur10ebase_link")) {
    sleep(1);
  }

  while (!get_rotation_matrix(rot_matrix, world_frame_id_, "base_link")) {
    sleep(1);
  }
  base_world_ready_ = true;

  while (!get_rotation_matrix(rot_matrix, world_frame_id_, "ur10ebase_link")) {
    sleep(1);
  }
  arm_world_ready_ = true;
  while (!get_rotation_matrix(rot_matrix, "ur10ebase_link", world_frame_id_)) {
    sleep(1);
  }
  world_arm_ready_ = true;

  while (!get_rotation_matrix(rot_matrix, "ur10ebase_link", "ur10etool0")) {
    sleep(1);
  }

  ft_arm_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "The Force/Torque sensor is ready to use.");
}

void AdmittanceController::setup_moveit_servo() {
  const std::string servo_param_namespace = "moveit_servo";
  auto servo_param_listener =
      std::make_shared<const servo::ParamListener>(this->shared_from_this(), servo_param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  planning_scene_monitor_ =
      moveit_servo::createPlanningSceneMonitor(this->shared_from_this(), servo_params);
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startStateMonitor();

  servo_ = std::make_shared<moveit_servo::Servo>(this->shared_from_this(), servo_param_listener,
                                                 planning_scene_monitor_);
  servo_->setCommandType(moveit_servo::CommandType::TWIST);
}

////////////
/// UTIL ///
////////////

bool AdmittanceController::get_rotation_matrix(Matrix6d& rotation_matrix,
                                               std::string from_frame,
                                               std::string to_frame) {
  try {
    const geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
    const Eigen::Matrix3d rotation_from_to =
        Eigen::Quaterniond(transform.transform.rotation.w, transform.transform.rotation.x,
                           transform.transform.rotation.y, transform.transform.rotation.z)
            .toRotationMatrix();

    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  } catch (tf2::TransformException& ex) {
    rotation_matrix.setZero();
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for TF from: " << from_frame << " to: " << to_frame);
    return false;
  }

  return true;
}

void AdmittanceController::publish_arm_state_in_world() {
  // publishing the cartesian velocity of the EE in the world-frame
  Matrix6d rotation_a_base_world;
  Matrix6d rotation_p_base_world;

  if (arm_world_ready_ && base_world_ready_) {
    get_rotation_matrix(rotation_a_base_world, world_frame_id_, "ur10ebase_link");
    get_rotation_matrix(rotation_p_base_world, world_frame_id_, "base_link");

    ee_twist_world_ =
        rotation_a_base_world * arm_real_twist_ + rotation_p_base_world * platform_real_twist_;
    // ee_twist_world_ = arm_real_twist_ + platform_real_twist_;
  }

  geometry_msgs::msg::TwistStamped msg_twist;
  msg_twist.header.stamp = this->get_clock()->now();
  msg_twist.header.frame_id = world_frame_id_;
  msg_twist.twist.linear.x = ee_twist_world_(0);
  msg_twist.twist.linear.y = ee_twist_world_(1);
  msg_twist.twist.linear.z = ee_twist_world_(2);
  msg_twist.twist.angular.x = ee_twist_world_(3);
  msg_twist.twist.angular.y = ee_twist_world_(4);
  msg_twist.twist.angular.z = ee_twist_world_(5);
  pub_ee_twist_world_->publish(msg_twist);

  // publishing the cartesian position of the EE in the world-frame
  if (arm_world_ready_ && base_world_ready_) {
    try {
      // listener.lookupTransform("ur5_arm_base_link", "robotiq_force_torque_frame_id",
      const geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform(world_frame_id_, "ur10etool0", tf2::TimePointZero);

      ee_pose_world_(0) = transform.transform.translation.x;
      ee_pose_world_(1) = transform.transform.translation.y;
      ee_pose_world_(2) = transform.transform.translation.z;
      ee_pose_world_(3) = transform.transform.rotation.x;
      ee_pose_world_(4) = transform.transform.rotation.y;
      ee_pose_world_(5) = transform.transform.rotation.z;
      ee_pose_world_(6) = transform.transform.rotation.w;
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Couldn't lookup for ee to world transform...");
      ee_pose_world_.setZero();
      ee_pose_world_(6) = 1;  // quat.w = 1
    }
  }

  geometry_msgs::msg::PoseStamped msg_pose;
  msg_pose.header.stamp = this->get_clock()->now();
  msg_pose.header.frame_id = world_frame_id_;
  msg_pose.pose.position.x = ee_pose_world_(0);
  msg_pose.pose.position.y = ee_pose_world_(1);
  msg_pose.pose.position.z = ee_pose_world_(2);
  msg_pose.pose.orientation.x = ee_pose_world_(3);
  msg_pose.pose.orientation.y = ee_pose_world_(4);
  msg_pose.pose.orientation.z = ee_pose_world_(5);
  msg_pose.pose.orientation.w = ee_pose_world_(6);
  pub_ee_pose_world_->publish(msg_pose);
}

void AdmittanceController::publish_debuggings_signals() {
  geometry_msgs::msg::WrenchStamped msg_wrench;

  msg_wrench.header.stamp = this->get_clock()->now();
  msg_wrench.header.frame_id = "ur10ebase_link";
  msg_wrench.wrench.force.x = wrench_external_(0);
  msg_wrench.wrench.force.y = wrench_external_(1);
  msg_wrench.wrench.force.z = wrench_external_(2);
  msg_wrench.wrench.torque.x = wrench_external_(3);
  msg_wrench.wrench.torque.y = wrench_external_(4);
  msg_wrench.wrench.torque.z = wrench_external_(5);
  pub_wrench_external_->publish(msg_wrench);

  msg_wrench.header.stamp = this->get_clock()->now();
  msg_wrench.header.frame_id = "ur10ebase_link";
  msg_wrench.wrench.force.x = wrench_control_(0);
  msg_wrench.wrench.force.y = wrench_control_(1);
  msg_wrench.wrench.force.z = wrench_control_(2);
  msg_wrench.wrench.torque.x = wrench_control_(3);
  msg_wrench.wrench.torque.y = wrench_control_(4);
  msg_wrench.wrench.torque.z = wrench_control_(5);
  pub_wrench_control_->publish(msg_wrench);

  geometry_msgs::msg::PointStamped msg_point;

  msg_point.header.stamp = this->get_clock()->now();
  msg_point.header.frame_id = "ur10ebase_link";
  msg_point.point.x = equilibrium_position_(0);
  msg_point.point.y = equilibrium_position_(1);
  msg_point.point.z = equilibrium_position_(2);
  pub_equilibrium_real_->publish(msg_point);
}
