#ifndef ADMITTANCECONTROLLER_H
#define ADMITTANCECONTROLLER_H

#include <Eigen/Dense>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <moveit_servo/servo.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "cartesian_state_msgs/msg/pose_twist.hpp"

// AdmittanceController class //
// A simple class implementing an admittance
// controller for the ridgeback + UR5 platform at the LASA lab. An
// AdmittanceController is a while loop that:
// 1) Reads the robot state (arm+platform)
// 1) Sends a desired twist to the robot platform
// 2) Sends a desired twist to the robot arm
//
// Communication interfaces:
// - The desired twists are both sent through ROS topics.
// The low level controllers of both the arm and the platform are
// implemented in ROS control.
// NOTE: This node should be ideally a ROS
// controller, but the current implementation of ROS control in
// indigo requires writing a specific interface for this matter.
// The kinetic version of ROS control enables controllers with
// multiple interfaces (FT sensor, platform and arm), but a proper
// ROS control implementation of this controller remains future work.
//
// USAGE EXAMPLE;
// ros::NodeHandle nh;
// double frequency = 1000.0;
// std::string state_topic_arm, cmd_topic_arm, topic_arm_twist_world,
//   topic_wrench_u_e, topic_wrench_u_c, cmd_topic_platform,
//   state_topic_platform, wrench_topic, wrench_control_topic,
//   laser_front_topic, laser_rear_topic;
// std::vector<double> M_p, M_a, D, D_p, D_a, K, d_e;
// double wrench_filter_factor, force_dead_zone_thres,
//    torque_dead_zone_thres, obs_distance_thres, self_detect_thres;
//
//
// // Fill in values
// ...
//
//
// AdmittanceController admittance_controller(nh, frequency,
//                                           cmd_topic_platform,
//                                           state_topic_platform,
//                                           cmd_topic_arm,
//                                           topic_arm_twist_world,
//                                           topic_wrench_u_e,
//                                           topic_wrench_u_c,
//                                           state_topic_arm,
//                                           wrench_topic,
//                                           wrench_control_topic,
//                                           laser_front_topic,
//                                           laser_rear_topic,
//                                           M_p, M_a, D, D_p, D_a, K, d_e,
//                                           wrench_filter_factor,
//                                           force_dead_zone_thres,
//                                           torque_dead_zone_thres,
//                                           obs_distance_thres,
//                                           self_detect_thres);
// admittance_controller.run();

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class AdmittanceController : public rclcpp::Node {
 protected:
  // ROS VARIABLES:
  // Rate of the run loop
  rclcpp::Rate loop_rate_;

  // Subscribers:

  // Subscriber for the platform state
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_platform_state_;
  // Subscriber for the arm state
  // rclcpp::Subscription<cartesian_state_msgs::msg::PoseTwist>::SharedPtr sub_arm_state_;
  // Subscriber for the ft sensor at the endeffector
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_external_;
  // Subscriber for the ft sensor at the endeffector
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_control_;
  // Subscriber for the offset of the attractor
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_equilibrium_desired_;
  // Subscriber for the admittance ratio
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_admittance_ratio_;
  // Subscriber for the DS desired velocity
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_ds_velocity_;

  // Publishers:

  // Publisher for the twist of the platform
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_platform_cmd_;
  // Publisher for the twist of arm endeffector
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_arm_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_arm_cmd_;
  // Publisher for the pose of arm endeffector in the world frame
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_ee_pose_world_;
  // Publisher for the twist of arm endeffector in the world frame
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_ee_twist_world_;
  // Publisher for the external wrench specified in the world frame
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_external_;
  // Publisher for the control wrench specified in the world frame
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_control_;
  // Publisher to visualize the real equilibrium used by admittance.
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_equilibrium_real_;

  // MoveIt Servo (cartesian -> configuration space)
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::shared_ptr<moveit_servo::Servo> servo_;

  // INPUT SIGNAL
  // external wrench (force/torque sensor) in "robotiq_force_torque_frame_id" frame
  Vector6d wrench_external_;
  // control wrench (from any controller) expected to be in "ur5_arm_base_link" frame
  Vector6d wrench_control_;

  // FORCE/TORQUE-SENSOR FILTER:
  // Parameters for the noisy wrench
  double wrench_filter_factor_;
  double force_dead_zone_thres_;
  double torque_dead_zone_thres_;
  double admittance_ratio_;

  // ADMITTANCE PARAMETERS:
  // M_p_, M_a_ -> Desired mass of platform/arm
  // D_ -> Desired damping of the coupling
  // D_p_, D_a_ -> Desired damping of platform/arm
  // K_ -> Desired Stiffness of the coupling
  Matrix6d M_p_, M_a_, D_, D_p_, D_a_, K_;
  // equilibrium position of the coupling spring
  Vector3d equilibrium_position_;
  Vector3d equilibrium_position_seen_by_platform;
  // equilibrium orientation of the coupling spring
  Quaterniond equilibrium_orientation_;

  // receiving a new equilibrium from a topic
  Vector3d equilibrium_new_;

  // arm desired velocity based on DS (or any other velocity input)
  Vector3d arm_desired_twist_ds_;

  // desired velocity for arm based on admittance
  Vector6d arm_desired_twist_adm_;

  // OUTPUT COMMANDS
  // final arm desired velocity
  Vector6d arm_desired_twist_final_;
  // the desired velcoities computed by the admittance control
  Vector6d platform_desired_twist_;

  // limiting the workspace of the arm
  Vector6d workspace_limits_;
  double arm_max_vel_;
  double arm_max_acc_;
  double platform_max_vel_;
  double platform_max_acc_;

  // STATE VARIABLES:
  // Platform state: position, orientation, and twist (in "platform base_link")
  Vector3d platform_real_position_;
  Quaterniond platform_real_orientation_;
  Vector6d platform_real_twist_;

  // Arm state: position, orientation, and twist (in "ur5_arm_base_link")
  Vector3d arm_real_position_;
  Quaterniond arm_real_orientation_;
  Vector6d arm_real_twist_;

  // End-effector state: pose and twist (in "world" frame)
  Vector7d ee_pose_world_;
  Vector6d ee_twist_world_;

  // Transform from base_link to world
  Matrix6d rotation_base_;
  // Derivative of kinematic constraints between the arm and the platform
  Matrix6d kin_constraints_;

  // TF:
  // Listeners
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Guards
  bool ft_arm_ready_;
  bool arm_world_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;

  // Initialization
public:
  void wait_for_transformations();
  // moveit_servo setup requires calls to this->shared_from_this, which are not
  // available in the constructor.
  void setup_moveit_servo();

protected:
  // Control
  void compute_admittance();

  // Callbacks
  void state_platform_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // void state_arm_callback(const cartesian_state_msgs::msg::PoseTwist::SharedPtr msg);
  bool update_arm_state();
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void wrench_control_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  // Util
  bool get_rotation_matrix(Matrix6d& rotation_matrix, std::string from_frame, std::string to_frame);

  void publish_arm_state_in_world();

  // void get_ee_pose_world(geometry_msgs::Pose & ee_pose_world,
  //                        tf::TransformListener & listener);

  void limit_to_workspace();

  void publish_debuggings_signals();

  void send_commands_to_robot();

  void equilibrium_callback(const geometry_msgs::msg::Point::SharedPtr msg);

  void admittance_ratio_callback(const std_msgs::msg::Float32::SharedPtr msg);

  void ds_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

 public:
  AdmittanceController(double frequency);
  void run();
};

#endif  // ADMITTANCECONTROLLER_H
