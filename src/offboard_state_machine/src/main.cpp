// main.cpp  (FULL REPLACEMENT)
#include "offboard_fsm_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Read the parameter that the launch file supplies:  --ros-args -p drone_id:=N
  auto param_reader = std::make_shared<rclcpp::Node>("drone_id_reader");
  const int drone_id = param_reader->declare_parameter<int>("drone_id", 0);

  RCLCPP_INFO(param_reader->get_logger(),
              "Starting Offboard FSM process for drone %d", drone_id);

  // Create *one* FSM node for this drone
  auto fsm = std::make_shared<OffboardFSM>(drone_id);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(fsm);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
