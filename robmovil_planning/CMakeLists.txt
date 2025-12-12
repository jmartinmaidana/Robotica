cmake_minimum_required(VERSION 3.5)
project(robmovil_planning)

# Enable C++11 support
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   tf
#   geometry_msgs
#   nav_msgs
#   robmovil_msgs
#   cmake_modules
# )
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES grid_planner
#    CATKIN_DEPENDS roscpp
# #  DEPENDS system_lib
# )
include_directories(
  include
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )

add_library(grid_planner src/GridPlanner.cpp src/GridPlannerNode.cpp)
ament_target_dependencies(grid_planner rclcpp geometry_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs angles)
target_include_directories(grid_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

## Declare a C++ executable
#add_executable(grid_planner src/GridPlannerNode.cpp)
#target_link_libraries(grid_planner grid_planner_lib)
#ament_target_dependencies(grid_planner rclcpp)

add_library(robmovil_planning::grid_planner ALIAS grid_planner)

install(TARGETS
  grid_planner
  #grid_planner
  EXPORT export_grid_planner
  #ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  #RUNTIME DESTINATION lib/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS grid_planner_lib
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_grid_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs angles)
#ament_export_libraries(grid_planner_lib)

ament_package()