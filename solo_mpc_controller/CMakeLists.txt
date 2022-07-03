cmake_minimum_required(VERSION 3.0.2)
project(solo_mpc_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_manager
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES solo_mpc_controller
#  CATKIN_DEPENDS control_msgs controller_manager geometry_msgs roscpp rospy std_msgs
  DEPENDS Eigen3
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
)

# Include OSQP Header and Library
include_directories(include/${PROJECT_NAME}/osqp)
LINK_DIRECTORIES(src/osqp/lib)

# Include EIGEN interface with OSQP
add_subdirectory(src/osqp-eigen)
include_directories(src/osqp-eigen/include)

add_library(${PROJECT_NAME} 
	src/solo8_centroid_mpc.cpp 
	src/solo8_jacobians.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} osqp OsqpEigen)

add_executable(solo8_centroid_mpc 
  src/solo8_centroid_mpc_node.cpp
)
target_link_libraries(solo8_centroid_mpc ${catkin_LIBRARIES} ${PROJECT_NAME} ${EIGEN3_LIBRARIES} osqp OsqpEigen)
target_include_directories(solo8_centroid_mpc PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
