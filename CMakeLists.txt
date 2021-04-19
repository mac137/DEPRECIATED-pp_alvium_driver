cmake_minimum_required(VERSION 3.2)
project(pp_alvium_driver)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  #message_generation
  #message_runtime
  #roscpp
  rospy
  #std_msgs
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

#add_service_files(
#  FILES
#        PPLaunchSensor.srv
#)
#
#generate_messages(
#  DEPENDENCIES
#  #std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  #message_runtime
  #roscpp
  rospy
  #std_msgs
  sensor_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(
#  ${PROJECT_NAME}
#  src/${PROJECT_NAME}.cpp
#)
#target_link_libraries(
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)
#add_dependencies(
#  ${PROJECT_NAME}
#  ${PROJECT_NAME}_generate_messages_cpp
#  ${catkin_EXPORTED_TARGETS}
#)

catkin_python_setup()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
        scripts/ros_alvium_rgb.py
        scripts/ros_alvium_nir.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark executables and/or libraries for installation
#install(
#  TARGETS
#  ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#############
## Testing ##
#############
