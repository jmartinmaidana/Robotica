cmake_minimum_required(VERSION 3.5)
project(ej2)

# Enable C++11 support
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   tf2_ros
#   lazo_abierto
#   robmovil_msgs
#   cmake_modules
#   angles
# )
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(trajectory_msgs REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robmovil_ekf
#   CATKIN_DEPENDS roscpp tf2_ros ej2
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  src
)
add_library(trajectory_controller src/TrajectoryFollower.cpp)

## Declare a C++ executable
add_executable(trajectory_follower_cl 
  src/kinematic_position_controller_node.cpp 
  src/KinematicPositionController.cpp
)
add_executable(trajectory_generator src/trajectory_generator_node.cpp)

ament_target_dependencies(trajectory_follower_cl rclcpp ament_cmake geometry_msgs robmovil_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs angles lazo_abierto)
ament_target_dependencies(trajectory_controller rclcpp geometry_msgs robmovil_msgs nav_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(trajectory_generator rclcpp geometry_msgs robmovil_msgs nav_msgs tf2 tf2_geometry_msgs)

add_library(ej2::trajectory_controller ALIAS trajectory_controller)




add_executable(logger src/logger_node.cpp)
ament_target_dependencies(logger rclcpp ament_cmake geometry_msgs robmovil_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs angles) 
# target_link_libraries(logger ej2::trajectory_controller)

## Specify libraries to link a library or executable target against
#target_link_libraries(trajectory_follower ${catkin_LIBRARIES})
#target_link_libraries(logger ${catkin_LIBRARIES})

target_include_directories(trajectory_follower_cl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  trajectory_follower_cl
  logger
  DESTINATION lib/${PROJECT_NAME}
)
# Install targets
install(TARGETS
  trajectory_controller
  #trajectory_generator
  EXPORT export_trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  #trajectory_controller
  trajectory_generator
  #EXPORT export_trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
# Install include files
#install(DIRECTORY include/
#  DESTINATION include
#)

# Install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )
ament_export_targets(export_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs robmovil_msgs nav_msgs tf2 tf2_geometry_msgs)

ament_export_include_directories(include)

ament_package()
