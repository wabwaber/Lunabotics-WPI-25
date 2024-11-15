cmake_minimum_required(VERSION 3.5)
project(luna_gz)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})
