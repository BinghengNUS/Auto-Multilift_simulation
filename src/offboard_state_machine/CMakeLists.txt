cmake_minimum_required(VERSION 3.8)
project(offboard_state_machine)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------- build target ----------
add_executable(offboard_fsm_node
  src/offboard_fsm_node.cpp
  src/main.cpp
)

target_compile_features(offboard_fsm_node PUBLIC cxx_std_17)

target_include_directories(offboard_fsm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/offboard_state_machine>
  $<INSTALL_INTERFACE:include/offboard_state_machine>
  ${EIGEN3_INCLUDE_DIRS}
)


ament_target_dependencies(offboard_fsm_node
  rclcpp
  std_msgs
  px4_msgs
  Eigen3
)

# ---------- install ----------
install(TARGETS offboard_fsm_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
