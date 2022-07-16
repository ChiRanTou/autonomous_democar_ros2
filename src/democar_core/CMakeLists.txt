cmake_minimum_required(VERSION 3.5)
project(democar_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
include_directories(${QPOASES_INLUDE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(democar_purepursuit_node
  src/purePursuit.cpp
  src/loadPath.cpp
)

add_executable(democar_lqr_dynamics_node
  src/lqr_control_dynamics.cpp
  src/loadPath.cpp
)

add_executable(democar_lqr_kinematics_node
  src/lqr_control_kinematics.cpp
  src/loadPath.cpp
)

add_executable(democar_mpc_node
  src/mpc_control.cpp
  src/loadPath.cpp
)

include_directories(include)
# include_directories(/usr/local/include/qpOASES)
target_link_libraries(democar_mpc_node /usr/local/lib/libqpOASES.a)

ament_target_dependencies(
  democar_purepursuit_node
  geometry_msgs
  nav_msgs
  rclcpp
  ament_index_cpp
)

ament_target_dependencies(
  democar_lqr_dynamics_node
  geometry_msgs
  nav_msgs
  rclcpp
  ament_index_cpp
)

ament_target_dependencies(
  democar_lqr_kinematics_node
  geometry_msgs
  nav_msgs
  rclcpp
  ament_index_cpp
)

ament_target_dependencies(
  democar_mpc_node
  geometry_msgs
  nav_msgs
  rclcpp
  ament_index_cpp
)

install(TARGETS
  democar_purepursuit_node
  democar_lqr_dynamics_node
  democar_lqr_kinematics_node
  democar_mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  waypoints
  DESTINATION share/${PROJECT_NAME}
)

