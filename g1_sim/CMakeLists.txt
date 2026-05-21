cmake_minimum_required(VERSION 3.5)
project(g1_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(stand_g1
  src/stand_g1.cpp
  src/motor_crc_hg.cpp
)

ament_target_dependencies(stand_g1
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
)

add_executable(wave_g1
  src/wave_g1.cpp
  src/motor_crc_hg.cpp
)

ament_target_dependencies(wave_g1
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
)

install(TARGETS stand_g1 wave_g1
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/joint_state_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
