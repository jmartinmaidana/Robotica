cmake_minimum_required(VERSION 3.8)
project(ej2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

# 2. Directorios de Include
include_directories(include)

# 3. Librería central para los controladores (Base + Derivada)
add_library(controllers_lib SHARED
  src/TrajectoryFollower.cpp
  src/KinematicPositionController.cpp
)
ament_target_dependencies(controllers_lib
  rclcpp geometry_msgs nav_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs angles
)
target_include_directories(controllers_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 4. Nodos Ejecutables

# Controlador Cinemático
add_executable(trajectory_follower_cl src/kinematic_position_controller_node.cpp)
target_link_libraries(trajectory_follower_cl controllers_lib)
ament_target_dependencies(trajectory_follower_cl rclcpp geometry_msgs nav_msgs robmovil_msgs)

# Generador de Trayectorias
add_executable(trajectory_generator src/trajectory_generator_node.cpp)
ament_target_dependencies(trajectory_generator rclcpp geometry_msgs nav_msgs robmovil_msgs tf2 tf2_geometry_msgs)

# Waypoints
add_executable(trajectory_waypoints src/trajectory_waypoints_node.cpp)
ament_target_dependencies(trajectory_waypoints rclcpp trajectory_msgs nav_msgs geometry_msgs tf2)

# Logger
add_executable(logger src/logger_node.cpp)
ament_target_dependencies(logger rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

# 5. Instalación (para que ROS encuentre los ejecutables y archivos al hacer source)

# La librería debe ir en la raíz "lib" para que LD_LIBRARY_PATH la encuentre
install(TARGETS controllers_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Los nodos ejecutables sí van dentro de lib/ej2/
install(TARGETS
  trajectory_follower_cl
  trajectory_generator
  trajectory_waypoints
  logger
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()