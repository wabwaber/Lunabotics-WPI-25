cmake_minimum_required(VERSION 3.8)
project(jetson_comm)

set(jetson_comm_MAJOR_VERSION 0)
set(jetson_comm_MINOR_VERSION 0)
set(jetson_comm_PATCH_VERSION 0)
set(jetson_comm_VERSION
  ${jetson_comm_MAJOR_VERSION}.${jetson_comm_MINOR_VERSION}.${jetson_comm_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
