cmake_minimum_required(VERSION 3.8)
project(ackermann_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


install(
  PROGRAMS
    scripts/cmd_vel_to_ackermann.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
