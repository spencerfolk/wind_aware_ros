cmake_minimum_required(VERSION 3.0.2)
project(wind_aware_mppi)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  message_generation
  message_runtime
  tf2_eigen
  tf2
)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

# If you need to define custom messages or services, uncomment and define them here
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # or any other dependencies you might have
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# If dynamic reconfigure is needed, you can declare .cfg files here
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
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
add_executable(wind_aware_mppi_node src/mppi_controller_node.cpp)

## Specify libraries to link against
target_link_libraries(wind_aware_mppi_node
  ${catkin_LIBRARIES}  # ROS libraries
  ${EIGEN3_LIBRARIES}  # Eigen
)

#############
## Install ##
#############

## Mark executables for installation
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

#############
## Testing ##
#############

# If you want to add tests
# catkin_add_gtest(${PROJECT_NAME}-test test/test_wind_aware_mppi.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Add folders to be run by Python nosetests
# catkin_add_nosetests(test)