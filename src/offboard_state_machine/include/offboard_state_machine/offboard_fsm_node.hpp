// offboard_fsm_node.hpp
#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/int32.hpp>

class OffboardFSM : public rclcpp::Node
{
public:
  explicit OffboardFSM(int drone_id);

private:
  // FSM states (added LAND before DONE)
  enum class FsmState { INIT = 0, ARMING, TAKEOFF,GOTO, HOVER, TRAJ, END_TRAJ, LAND, DONE };

  // callbacks
  void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void state_cmd_cb(const std_msgs::msg::Int32::SharedPtr msg);
  void timer_cb();

  // helpers
  void publish_offboard_mode();
  void send_vehicle_cmd(uint16_t cmd, float p1, float p2);
  void try_set_offboard_and_arm();
  void generate_trajectory();
  bool has_goto_target() const;
  // static float wrap_pi(float x);
  struct Poly5 { double a[6]; };
  Poly5 poly_[3];            // 0:x, 1:y, 2:z
  rclcpp::Time traj_start_;
  double traj_T_ = 0.0;

  static inline double eval_poly(const Poly5 &p, double t)
  {
    return ((p.a[5]*t + p.a[4])*t + p.a[3])*t*t*t + p.a[0];
  }
  static inline double eval_d1(const Poly5 &p, double t)
  {
    return (5*p.a[5]*t + 4*p.a[4])*t*t*t + 3*p.a[3]*t*t;
  }
  static inline double eval_d2(const Poly5 &p, double t)
  {
    return (20*p.a[5]*t + 12*p.a[4])*t*t + 6*p.a[3]*t;
  }

  // parameters
  int    drone_id_;
  double takeoff_alt_;
  double takeoff_time_s_;
  double climb_rate_;                 
  double landing_time_s_;               
  double takeoff_pos_x_;
  double takeoff_pos_y_;
  double timer_period_s_;
  double alt_tol_;
  std::string trajectory_type_;
  // for circle trajectory
  double radius_;
  double period_s_;

  // state variables
  FsmState current_state_;
  int      offb_counter_;
  int      nav_state_;
  int      arming_state_;
  bool     ext_state_cmd_;
  bool     use_attitude_control_ = false;
  bool     odom_ready_ = false;         // new
  double   current_x_;                  // new
  double   current_y_;                  // new
  double   current_z_;
  double   last_z_;
  int      takeoff_start_count_;
  int      takeoff_complete_count_;
  double   circle_radius_;     
  double   inward_offset_;     
  int      num_drones_; 
  double   landing_start_z_;            
  double   landing_x_, landing_y_;      
  int      landing_start_count_;        
  double   hover_x_ = 0.0;
  double   hover_y_ = 0.0;
  double   hover_z_ = 0.0;  // NED down
  // double   goto_x_{std::nanf("")}, goto_y_{std::nanf("")}, goto_z_{std::nanf("")};
  // offboard_fsm_node.hpp  (add after the other member variables)
  double goto_x_{std::numeric_limits<double>::quiet_NaN()};
  double goto_y_{std::numeric_limits<double>::quiet_NaN()};
  double goto_z_{std::numeric_limits<double>::quiet_NaN()};

  double   goto_tol_{0.03};

  std::string px4_ns_;

  // publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offb_mode_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr               pub_state_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  pub_traj_sp_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr pub_att_sp_;

  // subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr    sub_status_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            sub_state_cmd_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr  sub_odom_;

  // timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr selfdefine_traj_timer_;
  rclcpp::Time state_start_time_;
};
