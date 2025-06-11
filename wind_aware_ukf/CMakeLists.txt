cmake_minimum_required(VERSION 3.0.2)
project(wind_aware_ukf)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  kr_mav_msgs
  tf2_ros
  message_generation
  message_runtime
  tf2_eigen
  tf2
  crazyflie_driver
  kalman_filter
)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
add_message_files(
  FILES
  MotorSpeeds.msg
  WindEstimate.msg
  WindEstimateStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)

## Declare a C++ executable
add_executable(wind_aware_ukf_node src/wind_ukf_node.cpp src/wind_ukf)

## Specify libraries to link against
target_link_libraries(wind_aware_ukf_node
  ${catkin_LIBRARIES}  # ROS libraries
  ${EIGEN3_LIBRARIES}  # Eigen
)

#############
## Install ##
#############

# Mark executables for installation
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

## Mark other files for installation (e.g., launch and bag files)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
