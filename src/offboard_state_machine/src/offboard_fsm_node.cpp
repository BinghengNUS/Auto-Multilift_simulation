// offboard_fsm_node.cpp – revised 2025-06-07
// Finite-state machine for PX4 offboard control (ROS 2)
// Author: Yichao Gao 

#include "offboard_fsm_node.hpp"

#include <Eigen/Core> 

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
// using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */
static float wrap_pi(float x)
{
  while (x >  M_PI) x -= 2.f * M_PI;
  while (x < -M_PI) x += 2.f * M_PI;
  return x;
}

// ENU → NED yaw: +90 deg rotation
static float enu_to_ned_yaw(float yaw_enu)
{
  float y = yaw_enu + static_cast<float>(M_PI_2);
  return wrap_pi(y);
}


/* ------------------------------------------------------------------ */
/*  Constructor                                                       */
/* ------------------------------------------------------------------ */
OffboardFSM::OffboardFSM(int drone_id)
: Node("offboard_fsm_node_" + std::to_string(drone_id))
, drone_id_(drone_id)
, takeoff_alt_    (declare_parameter("takeoff_alt",     1.51))
, takeoff_time_s_ (declare_parameter("takeoff_time",    1.0))
, climb_rate_     (declare_parameter("climb_rate",      1.0))  
, landing_time_s_ (declare_parameter("landing_time",    5.0))
, circle_radius_  (declare_parameter("circle_radius",   1.4))
// , inward_offset_  (declare_parameter("inward_offset",   0.90))
, inward_offset_  (declare_parameter("inward_offset",   0.80))
, num_drones_     (declare_parameter("num_drones",      6))
, timer_period_s_ (declare_parameter("timer_period",    0.02))
, alt_tol_        (declare_parameter("alt_tol",         0.03))
, radius_         (declare_parameter("circle_radius_traj", 3.0))
, period_s_       (declare_parameter("circle_period",   20.0))
// goto
, goto_x_       (declare_parameter<double>("goto_x", std::numeric_limits<double>::quiet_NaN()))
, goto_y_       (declare_parameter<double>("goto_y", std::numeric_limits<double>::quiet_NaN()))
, goto_z_       (declare_parameter<double>("goto_z", std::numeric_limits<double>::quiet_NaN()))
// initial state
, current_state_(FsmState::INIT)
, offb_counter_(0)
, takeoff_complete_count_(-1)
, use_attitude_control_(false)         
, odom_ready_(false)                   
{
  /*  take-off xy position: equally spaced on a circle  */
  double theta    = 2.0 * M_PI * drone_id_ / static_cast<double>(num_drones_);
  double r_target = - inward_offset_;
  takeoff_pos_x_  =  r_target * std::sin(theta);
  takeoff_pos_y_  =  r_target * std::cos(theta);
  // takeoff_pos_x_ = goto_x_;
  // takeoff_pos_y_ = goto_y_;
  // RCLCPP_INFO(get_logger(),
  //         "Drone %d: take-off-x %.2f, take-off-y %.2f",
  //         drone_id_, takeoff_pos_x_, takeoff_pos_y_);
  
  climb_rate_ = takeoff_alt_ / takeoff_time_s_;  // [m/s]

  RCLCPP_INFO(get_logger(),
              "Init FSM for drone %d: take-off to %.2f m",
              drone_id_, takeoff_alt_);

  /* PX4 namespace */
  px4_ns_ = (drone_id_ == 0) ? "/fmu/" :
            "/px4_" + std::to_string(drone_id_) + "/fmu/";

  /* ---------------- publishers ---------------- */
  pub_offb_mode_ = create_publisher<OffboardControlMode>(
      px4_ns_ + "in/offboard_control_mode", rclcpp::QoS{1});
  pub_cmd_       = create_publisher<VehicleCommand>(
      px4_ns_ + "in/vehicle_command",       rclcpp::QoS{1});
  pub_state_     = create_publisher<std_msgs::msg::Int32>(
      "/state/state_drone_" + std::to_string(drone_id_), rclcpp::QoS{1});
  pub_traj_sp_   = create_publisher<TrajectorySetpoint>(
      px4_ns_ + "in/trajectory_setpoint",   rclcpp::QoS{1});
  // pub_att_sp_    = create_publisher<VehicleAttitudeSetpoint>(
  //     px4_ns_ + "in/vehicle_attitude_setpoint", rclcpp::QoS{1});

  /* ---------------- subscriptions ------------- */
  sub_status_ = create_subscription<VehicleStatus>(
      px4_ns_ + "out/vehicle_status_v1", rclcpp::SensorDataQoS(),
      std::bind(&OffboardFSM::status_cb, this, std::placeholders::_1));

  sub_state_cmd_ = create_subscription<std_msgs::msg::Int32>(
      "/state/command_drone_" + std::to_string(drone_id_), rclcpp::QoS{1},
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
  odom_ready_ = true;
}

void OffboardFSM::state_cmd_cb(const std_msgs::msg::Int32::SharedPtr msg)
{
  int s = msg->data;
  if (s < static_cast<int>(FsmState::INIT) ||
      s > static_cast<int>(FsmState::DONE))
    return;

  /* external override */
  current_state_ = static_cast<FsmState>(s);

  /* bookkeeping for LAND → DONE */
  if (current_state_ == FsmState::LAND) {
    landing_start_count_ = offb_counter_;
    landing_start_z_     = -current_z_;          // +up
    landing_x_           = current_x_;
    landing_y_           = current_y_;
  }

  /* entering TRAJ: store start time & enable attitude control */
  if (current_state_ == FsmState::TRAJ) {
    state_start_time_     = now();
    use_attitude_control_ = true;
  }

  RCLCPP_WARN(get_logger(), "External state override → %d", s);
}

bool OffboardFSM::has_goto_target() const
{
  return std::isfinite(goto_x_) &&
         std::isfinite(goto_y_) &&
         std::isfinite(goto_z_);
}

/* ------------------------------------------------------------------ */
/*  Main timer                                                        */
/* ------------------------------------------------------------------ */
void OffboardFSM::timer_cb()
{
  publish_offboard_mode();

  switch (current_state_) {
  /* ---------- initial offboard / arm dance ---------------------- */
  case FsmState::INIT:
    if (offb_counter_ >= 10) {
      try_set_offboard_and_arm();
      current_state_ = FsmState::ARMING;
    }
    break;

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

  case FsmState::TAKEOFF: {
    double elapsed   = offb_counter_ * timer_period_s_;
    double climb_rate= takeoff_alt_ / takeoff_time_s_;     // m/s
    double alt_sp    = std::min(takeoff_alt_, climb_rate * elapsed);
    
    TrajectorySetpoint sp{};
    sp.position[0] = takeoff_pos_x_;
    sp.position[1] = takeoff_pos_y_;
    // sp.position[0] = 0.0;
    // sp.position[1] = 0.0;
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
      RCLCPP_INFO(get_logger(), "Altitude reached (%.2f m)", actual_alt);
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
      current_state_ = FsmState::GOTO;
      state_start_time_ = now();
      offb_counter_   = 0;
    }

    last_z_ = current_z_;
  } break;

  case FsmState::GOTO: {
    if (!has_goto_target()) {
        RCLCPP_WARN(get_logger(), "No GOTO target, entering HOVER.");
        current_state_   = FsmState::HOVER;
        state_start_time_= now();
        offb_counter_    = 0;
        break;
      }

    TrajectorySetpoint sp{};
    sp.position[0] = goto_x_;
    sp.position[1] = goto_y_;
    sp.position[2] = goto_z_;         // NED
    sp.timestamp   = now().nanoseconds() / 1000;
    pub_traj_sp_->publish(sp);

    double err = std::sqrt(std::pow(current_x_-goto_x_,2)+
                            std::pow(current_y_-goto_y_,2)+
                            std::pow(current_z_-goto_z_,2));
    if (err < goto_tol_ && has_goto_target()) {
        RCLCPP_INFO(get_logger(), "Reached GOTO target.");
        current_state_ = FsmState::HOVER;
        hover_x_ = goto_x_;          // lock reference
        hover_y_ = goto_y_;
        hover_z_ = goto_z_;
        state_start_time_ = now();
        offb_counter_ = 0;
    }
  } break;


  /* ---------- HOVER (default after goto) -------------------- */
  case FsmState::HOVER: {
    if (use_attitude_control_) {          // coming back from TRAJ or others
      use_attitude_control_ = false;
      RCLCPP_INFO(get_logger(), "Switching to position control (HOVER)");
    }

    TrajectorySetpoint sp{};
    sp.position[0] = hover_x_;          // hold current position
    sp.position[1] = hover_y_;
    sp.position[2] = hover_z_;          // keep current NED pos
    sp.velocity[0] = 0.0;
    sp.velocity[1] = 0.0;
    sp.velocity[2] = 0.0;                 // no vertical velocity
    sp.timestamp   = now().nanoseconds() / 1000ULL;
    pub_traj_sp_->publish(sp);
  } break;

  /* ---------- Trajectory (external command only) ----------- */
  case FsmState::TRAJ:
    if (!use_attitude_control_) {
      use_attitude_control_ = true;
      state_start_time_     = now();      // reset timer in case of re-entry
      RCLCPP_INFO(get_logger(), "Switching to attitude control (TRAJ)");
    }
    // generate_trajectory();
    break;

  case FsmState::END_TRAJ:
    if (use_attitude_control_) {
      use_attitude_control_ = false;
      state_start_time_     = now();      // reset timer in case of re-entry
      RCLCPP_INFO(get_logger(), "Trajectory done");
    //   if (hold_done) {
    //   current_state_ = FsmState::GOTO;
    //   state_start_time_ = now();
    //   offb_counter_   = 0;
    // }
    }break;

  /* ---------- LAND ---------------------------------------------- */
  case FsmState::LAND: {
    double elapsed = (offb_counter_ - landing_start_count_) * timer_period_s_;
    double rate    = landing_start_z_ / landing_time_s_;
    double alt     = std::max(0.0, landing_start_z_ - rate * elapsed);

    TrajectorySetpoint sp{};
    sp.position[0] = landing_x_;
    sp.position[1] = landing_y_;
    sp.position[2] = -alt;
    sp.timestamp   = now().nanoseconds() / 1000ULL;
    pub_traj_sp_->publish(sp);

    if (alt <= 0.01 || elapsed >= landing_time_s_) {
      RCLCPP_INFO(get_logger(), "Drone %d landed", drone_id_);
      current_state_ = FsmState::DONE;
      use_attitude_control_ = false;
    }
  } break;

  case FsmState::DONE:
    break;
  }

  /* broadcast FSM state */
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
  // m.position  = !use_attitude_control_;
  // m.velocity  = !use_attitude_control_;
  m.position = true;
  m.velocity  = true;
  m.attitude  =  use_attitude_control_;
  // m.attitude  = false; 
  // m.body_rate = false;
  // m.thrust    = true;
  m.timestamp = now().nanoseconds() / 1000ULL;
  pub_offb_mode_->publish(m);
}

void OffboardFSM::send_vehicle_cmd(uint16_t cmd, float p1, float p2)
{
  VehicleCommand m{};
  m.command       = cmd;
  m.param1        = p1;
  m.param2        = p2;
  m.target_system = drone_id_ + 1;          // PX4 index starts at 1
  m.from_external = true;
  m.timestamp     = now().nanoseconds() / 1000ULL;
  pub_cmd_->publish(m);
}

void OffboardFSM::try_set_offboard_and_arm()
{
  send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,          1.f, 6.f);
  send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
}

/* ------------------------------------------------------------------ */
/*  Trajectory generator (circle)                                     */
/* ------------------------------------------------------------------ */
void OffboardFSM::generate_trajectory()
{
}
