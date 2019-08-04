cmake_minimum_required(VERSION 2.8.3)
project(imu_madgwick_filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/imu_madgwick_filter_node.cpp src/madgwick_filter.cpp src/imu_madgwick_filter.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
