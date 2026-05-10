cmake_minimum_required(VERSION 3.5)
project(laica_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_ros REQUIRED)
find_package(enc_lc REQUIRED)

add_executable(laica_velocity_predictor
  src/laica_velocity_predictor.cpp
)

ament_target_dependencies(laica_velocity_predictor
  rclcpp
  geometry_msgs
  unitree_ros
  enc_lc
)

install(TARGETS laica_velocity_predictor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
