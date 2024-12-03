# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;geometry_msgs;nav_msgs;mavros_msgs;trajectory_msgs;controller_msgs;std_srvs;visualization_msgs;gestelt_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltrajectory_server".split(';') if "-ltrajectory_server" != "" else []
PROJECT_NAME = "trajectory_server"
PROJECT_SPACE_DIR = "/home/carlson/ros/multi_cable/install"
PROJECT_VERSION = "0.0.1"
