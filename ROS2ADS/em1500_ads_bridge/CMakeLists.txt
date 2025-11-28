cmake_minimum_required(VERSION 3.8)
project(em1500_ads_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# POSIX threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ADS library (assumed pre-built in ../ADS/build)
find_library(
  LIBADS_LIBRARY
  NAMES AdsLib
  HINTS "../ADS/build"
)

# Your updated executable name
add_executable(em1500_node src/em1500_ads_node.cpp)

# Ament dependencies
ament_target_dependencies(em1500_node
  rclcpp
  std_msgs
)

# C++ standard
target_compile_features(em1500_node PUBLIC c_std_99 cxx_std_17)

# Include directories
target_include_directories(em1500_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    "../ADS/AdsLib"
)

# Link libraries (must use plain form)
target_link_libraries(em1500_node
  ${LIBADS_LIBRARY}
  Threads::Threads
)

# Install
install(TARGETS em1500_node
  DESTINATION lib/${PROJECT_NAME}
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

