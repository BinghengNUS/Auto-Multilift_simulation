// offboard_fsm_node.cpp  – modified version (2025-05-24)
// Finite-state machine for PX4 offboard control (ROS 2)
// Author: Yichao Gao

#include "offboard_fsm_node.hpp"
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>

using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
using VehicleCommand      = px4_msgs::msg::VehicleCommand;
using VehicleStatus       = px4_msgs::msg::VehicleStatus;
using TrajectorySetpoint  = px4_msgs::msg::TrajectorySetpoint;
using VehicleAttitudeSetpoint = px4_msgs::msg::VehicleAttitudeSetpoint;
using VehicleOdometry     = px4_msgs::msg::VehicleOdometry;

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */
// ENU → NED yaw: +90° rotation.
static float enu_to_ned_yaw(float yaw_enu)
{
  float y = yaw_enu + static_cast<float>(M_PI_2);
  if (y >  M_PI) y -= 2.0f * static_cast<float>(M_PI);
  if (y < -M_PI) y += 2.0f * static_cast<float>(M_PI);
  return y;
}

/* ------------------------------------------------------------------ */
/*  Constructor                                                       */
/* ------------------------------------------------------------------ */
OffboardFSM::OffboardFSM(int drone_id)
: Node("offboard_fsm_node_" + std::to_string(drone_id))
, drone_id_(drone_id)
, takeoff_alt_     (declare_parameter("takeoff_alt", 3.0))            // [m]
, takeoff_time_s_  (declare_parameter("takeoff_time", 5.0))           // [s]
, landing_time_s_  (declare_parameter("landing_time", 5.0))           // [s]
, circle_radius_   (declare_parameter("circle_radius", 1.4))   
, inward_offset_   (declare_parameter("inward_offset", 1.2))   
, num_drones_      (declare_parameter("num_drones",    6))
, takeoff_pos_x_   (declare_parameter("takeoff_pos_x", 0.0))          // [m]
, takeoff_pos_y_   (declare_parameter("takeoff_pos_y", 0.0))          // [m]
, trajectory_type_ (declare_parameter("trajectory_type", std::string("hover")))
, timer_period_s_  (declare_parameter("timer_period", 0.02))          // [s]
, alt_tol_         (declare_parameter("alt_tol", 0.05))               // [m]
, current_state_(FsmState::INIT)
, offb_counter_(0)
, nav_state_(VehicleStatus::NAVIGATION_STATE_MAX)
, arming_state_(VehicleStatus::ARMING_STATE_DISARMED)
, ext_state_cmd_(false)
, current_x_(0.0), current_y_(0.0), current_z_(0.0)
, last_z_(0.0)
, takeoff_start_count_(0)
, takeoff_complete_count_(-1)
, landing_start_z_(0.0)
, landing_x_(0.0), landing_y_(0.0)
, landing_start_count_(0)
// trajectory params
, radius_(declare_parameter("circle_radius_traj", 3.0))
, period_s_(declare_parameter("circle_period", 20.0))
// go to


{
  double theta    = 2.0 * M_PI * drone_id_ / static_cast<double>(num_drones_);
  double r_target =  - inward_offset_;
  takeoff_pos_x_  =  r_target * std::sin(theta);
  takeoff_pos_y_  =  r_target * std::cos(theta);


  RCLCPP_INFO(get_logger(), "Init FSM for drone %d: takeoff %.1fm in %.1fs", 
              drone_id_, takeoff_alt_, takeoff_time_s_);

  px4_ns_ = (drone_id_ == 0) ? "/fmu/" : "/px4_" + std::to_string(drone_id_) + "/fmu/";

  /* ---------------- publishers ---------------- */
  pub_offb_mode_ = create_publisher<OffboardControlMode>(
                     px4_ns_ + "in/offboard_control_mode", rclcpp::QoS(1));
  pub_cmd_       = create_publisher<VehicleCommand>(
                     px4_ns_ + "in/vehicle_command", rclcpp::QoS(1));
  pub_state_     = create_publisher<std_msgs::msg::Int32>(
                     "/state/state_drone_" + std::to_string(drone_id_), rclcpp::QoS(1));
  pub_traj_sp_   = create_publisher<TrajectorySetpoint>(
                     px4_ns_ + "in/trajectory_setpoint", rclcpp::QoS(1));
  pub_att_sp_    = create_publisher<VehicleAttitudeSetpoint>(
                     px4_ns_ + "in/vehicle_attitude_setpoint", rclcpp::QoS(1));

  /* ---------------- subscriptions ------------- */
  sub_status_ = create_subscription<VehicleStatus>(
                  px4_ns_ + "out/vehicle_status_v1", rclcpp::SensorDataQoS(),
                  std::bind(&OffboardFSM::status_cb, this, std::placeholders::_1));

  sub_state_cmd_ = create_subscription<std_msgs::msg::Int32>(
                     "/state/command_drone_" + std::to_string(drone_id_), rclcpp::QoS(1),
                     std::bind(&OffboardFSM::state_cmd_cb, this, std::placeholders::_1));

  sub_odom_ = create_subscription<VehicleOdometry>(
                px4_ns_ + "out/vehicle_odometry", rclcpp::SensorDataQoS(),
                std::bind(&OffboardFSM::odom_cb, this, std::placeholders::_1));

  /* ---------------- timer --------------------- */
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period_s_),
                             std::bind(&OffboardFSM::timer_cb, this));
}

/* ------------------------------------------------------------------ */
/*  Callbacks                                                         */
/* ------------------------------------------------------------------ */
void OffboardFSM::status_cb(const VehicleStatus::SharedPtr msg)
{
  nav_state_    = msg->nav_state;
  arming_state_ = msg->arming_state;
}

void OffboardFSM::odom_cb(const VehicleOdometry::SharedPtr msg)
{
  current_x_ = msg->position[0];
  current_y_ = msg->position[1];
  current_z_ = msg->position[2];
}

void OffboardFSM::state_cmd_cb(const std_msgs::msg::Int32::SharedPtr msg)
{
  int s = msg->data;
  if (s < (int)FsmState::INIT || s > (int)FsmState::DONE) return;

  current_state_ = static_cast<FsmState>(s);
  ext_state_cmd_ = true;
  RCLCPP_WARN(get_logger(), "External state override → %d", s);

  if (current_state_ == FsmState::LAND) {
    landing_start_count_ = offb_counter_;
    landing_start_z_     = -current_z_;   // positive up
    landing_x_           = current_x_;
    landing_y_           = current_y_;
  }
}

/* ------------------------------------------------------------------ */
/*  Main timer                                                        */
/* ------------------------------------------------------------------ */
void OffboardFSM::timer_cb()
{
  publish_offboard_mode();

  switch (current_state_) {
  case FsmState::INIT:
    if (offb_counter_ >= 10) {
      try_set_offboard_and_arm();
      current_state_ = FsmState::ARMING;
    }
    break;

  /* ---- wait until PX4 reports ARMED & OFFBOARD ------------------ */
  case FsmState::ARMING:
    if (nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
        arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
      RCLCPP_INFO(get_logger(), "Drone %d armed + offboard", drone_id_);
      current_state_       = FsmState::TAKEOFF;
      takeoff_start_count_ = offb_counter_;
      last_z_              = current_z_;
      offb_counter_        = 0;
    } else {
      try_set_offboard_and_arm();
    }
    break;

  /* ---- climb to take-off altitude -------------------------------- */
  case FsmState::TAKEOFF: {
    double elapsed   = offb_counter_ * timer_period_s_;
    double climb_rate= takeoff_alt_ / takeoff_time_s_;     // m/s
    double alt_sp    = std::min(takeoff_alt_, climb_rate * elapsed);
    
    TrajectorySetpoint sp{};
    // sp.position[0] = takeoff_pos_x_;
    // sp.position[1] = takeoff_pos_y_;
    sp.position[0] = 0.0;
    sp.position[1] = 0.0;
    sp.position[2] = -alt_sp;                              // NED (down = +)
    sp.velocity[2] = 0.0;
    // sp.yaw         = enu_to_ned_yaw(0.0f);
    sp.timestamp   = now().nanoseconds() / 1000;
    pub_traj_sp_->publish(sp);

    /* ----- reached-altitude detection --------------------------- */
    double actual_alt = -current_z_;                       // convert to +up
    bool   alt_ok     = actual_alt >= takeoff_alt_ - alt_tol_;
    bool   vel_ok     = std::abs(current_z_ - last_z_) < 0.02; // ~2 cm/s

    if (alt_ok && vel_ok && takeoff_complete_count_ < 0) {
      takeoff_complete_count_ = offb_counter_;
      RCLCPP_INFO(get_logger(), "Altitude reached (%.2f m), holding", actual_alt);
    }

    /* ----- after 5 s hover switch to next state ------------------ */
    bool hold_done = takeoff_complete_count_ >= 0 &&
                     (offb_counter_ - takeoff_complete_count_) * timer_period_s_ >= 5.0;
    // bool hold_done = 0;

    if (hold_done) {
      // if (trajectory_type_ != "hover") {
      //   selfdefine_traj_timer_ = create_wall_timer(
      //       std::chrono::duration<double>(timer_period_s_),
      //       std::bind(&OffboardFSM::generate_trajectory, this));
      // }
      current_state_ = FsmState::TRAJ;
      state_start_time_ = now();
      offb_counter_   = 0;
    }

    last_z_ = current_z_;
  } break;

  /* ---- user trajectory ------------------------------------------ */
  case FsmState::TRAJ:
    // set-points published by generate_trajectory() timer.
    // RCLCPP_INFO(get_logger(), "TRAJ");
    if (!use_attitude_control_) {
      use_attitude_control_ = true;  // switch to attitude control
      RCLCPP_INFO(get_logger(), "Switching to attitude control.");
    }
    // generate_trajectory();
    break;

  case FsmState::HOVER:{
    // Hover at current position, no trajectory setpoint.
    if (!use_attitude_control_) {
      RCLCPP_INFO(get_logger(), "Switching to HOVER.");
    }

    TrajectorySetpoint sp{};
    sp.position[0] = current_x_;
    sp.position[1] = current_y_;
    sp.position[2] = current_z_;  // NED
    sp.velocity[0] = 0.0f;
    sp.velocity[1] = 0.0f;
    sp.velocity[2] = 0.0f;
    sp.timestamp   = now().nanoseconds() / 1000;
    pub_traj_sp_->publish(sp);
  } break;

  /* ---- descend + disarm ----------------------------------------- */
  case FsmState::LAND: {
    double elapsed = (offb_counter_ - landing_start_count_) * timer_period_s_;
    double rate    = landing_start_z_ / landing_time_s_;
    double alt     = std::max(0.0, landing_start_z_ - rate * elapsed);

    TrajectorySetpoint sp{};
    sp.position[0] = landing_x_;
    sp.position[1] = landing_y_;
    sp.position[2] = -alt;                     // NED
    // sp.velocity[2] = static_cast<float>(rate);
    // sp.yaw         = enu_to_ned_yaw(0.0f);
    sp.timestamp   = now().nanoseconds() / 1000;
    pub_traj_sp_->publish(sp);

    if (alt <= 0.01 || elapsed >= landing_time_s_) {
      RCLCPP_INFO(get_logger(), "Drone %d landed", drone_id_);
      current_state_ = FsmState::DONE;
      offb_counter_  = 0;
    }
  } break;

  case FsmState::DONE:
    // nothing more to do.
    break;
  }

  // broadcast FSM state for monitoring.
  std_msgs::msg::Int32 st; st.data = static_cast<int>(current_state_);
  pub_state_->publish(st);
  ++offb_counter_;
}

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */
void OffboardFSM::publish_offboard_mode()
{
  OffboardControlMode m{};
  m.position  = !use_attitude_control_;
  m.velocity  = !use_attitude_control_;
  m.attitude  = use_attitude_control_;
  m.timestamp = now().nanoseconds() / 1000;
  pub_offb_mode_->publish(m);
}

void OffboardFSM::send_vehicle_cmd(uint16_t cmd, float p1, float p2)
{
  VehicleCommand m{};
  m.command       = cmd;
  m.param1        = p1;
  m.param2        = p2;
  m.target_system = drone_id_ + 1;           // PX4-uORB indexing starts at 1.
  m.from_external = true;
  m.timestamp     = now().nanoseconds() / 1000;
  pub_cmd_->publish(m);
}

void OffboardFSM::try_set_offboard_and_arm()
{
  send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,           1.0f, 6.0f);
  send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,  1.0f, 0.0f);
}

/* user trajectory – override as needed */
void OffboardFSM::generate_trajectory()
{
  if (!current_z_) {
    // If no odometry yet, do nothing
    return;
  }

  // Extract current altitude from odometry (NED frame: position[2] is down)
  float current_altitude = -current_z_;

  VehicleAttitudeSetpoint sp{};
  sp.timestamp = now().nanoseconds() / 1000ULL;

  if (current_altitude < static_cast<float>(takeoff_alt_)) {
    // Climb vertically: keep level attitude (no roll/pitch yaw), climb thrust > hover
    // Quaternion for level orientation (roll=0, pitch=0, yaw=0)
    double cr = std::cos(0.0 / 2.0);
    double sr = std::sin(0.0 / 2.0);
    double cp = std::cos(0.0 / 2.0);
    double sp2 = std::sin(0.0 / 2.0);
    double cy = std::cos(0.0 / 2.0);
    double sy = std::sin(0.0 / 2.0);

    // Following ZYX (yaw-pitch-roll) to quaternion conversion:
    double qw = cr*cp*cy + sr*sp2*sy;
    double qx = sr*cp*cy - cr*sp2*sy;
    double qy = cr*sp2*cy + sr*cp*sy;
    double qz = cr*cp*sy - sr*sp2*cy;
    sp.q_d[0] = static_cast<float>(qw);
    sp.q_d[1] = static_cast<float>(qx);
    sp.q_d[2] = static_cast<float>(qy);
    sp.q_d[3] = static_cast<float>(qz);

    // Set thrust in body frame to climb (negative downward)
    float thrust_norm = 0.3f;  // hover 0.28
    sp.thrust_body[0] = 0.0f;
    sp.thrust_body[1] = 0.0f;
    sp.thrust_body[2] = -thrust_norm;
  }
  else {
    // We are at or above takeoff altitude: compute attitude for circle motion

    // Time since entering CIRCLE state
    double t     = (now() - state_start_time_).seconds();
    double omega = 2.0 * M_PI / period_s_;  // angular speed [rad/s]
    double theta = omega * t;               // angle around circle, from north

    // Desired horizontal speed = omega * radius
    double v_horiz = omega * radius_;
    // Centripetal horizontal acceleration = v^2 / r
    double a_horiz = (v_horiz * v_horiz) / radius_;

    // Compute acceleration components in NED: north and east
    // In a circle parameterized by angle theta (measured from north toward east):
    //   a_north = -a_horiz * sin(theta)
    //   a_east  =  a_horiz * cos(theta)
    double a_north = -a_horiz * std::sin(theta);
    double a_east  =  a_horiz * std::cos(theta);

    // Convert horizontal acceleration into roll and pitch angles:
    //   roll  = asin(a_east / g)
    //   pitch = asin(a_north / g)
    const double g = 9.81;
    double roll_cmd  = std::asin(a_east  / g);
    double pitch_cmd = std::asin(a_north / g);

    // Yaw pointing tangent to circle: yaw = theta + 90 degrees (pi/2)
    double yaw_cmd = theta + M_PI / 2.0;
    float yaw_body = wrap_pi(static_cast<float>(yaw_cmd));

    // Convert desired (roll, pitch, yaw) into a quaternion (ZYX order)
    double cr = std::cos(roll_cmd  / 2.0);
    double sr = std::sin(roll_cmd  / 2.0);
    double cp = std::cos(pitch_cmd / 2.0);
    double sp2 = std::sin(pitch_cmd / 2.0);
    double cy = std::cos(yaw_body  / 2.0);
    double sy = std::sin(yaw_body  / 2.0);

    double qw = cr*cp*cy + sr*sp2*sy;
    double qx = sr*cp*cy - cr*sp2*sy;
    double qy = cr*sp2*cy + sr*cp*sy;
    double qz = cr*cp*sy - sr*sp2*cy;
    sp.q_d[0] = static_cast<float>(qw);
    sp.q_d[1] = static_cast<float>(qx);
    sp.q_d[2] = static_cast<float>(qy);
    sp.q_d[3] = static_cast<float>(qz);

    // Set thrust to maintain altitude (hover thrust ~0.5)
    float thrust_norm = 0.5f;  // approximate hover thrust (tune as needed)
    sp.thrust_body[0] = 0.0f;
    sp.thrust_body[1] = 0.0f;
    sp.thrust_body[2] = -thrust_norm;
  }

  pub_att_sp_->publish(sp);
}

float OffboardFSM::wrap_pi(float x)
{
  while (x >  M_PI) x -= 2.0f * M_PI;
  while (x < -M_PI) x += 2.0f * M_PI;
  return x;
}
