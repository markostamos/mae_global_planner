cmake_minimum_required(VERSION 3.0.2)
project(mae_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)


###################################



include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(global_planner src/GlobalPlanner.cpp)

add_executable(main test.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(main
   ${catkin_LIBRARIES}
    global_planner
 )

