cmake_minimum_required(VERSION 3.0.2)
project(mae_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
    FILES
    PointArray.msg
)

## Generate services in the 'srv' folder
  add_service_files(
   FILES
   PlanService.srv
   GlobalPlanService.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mae_global_planner
   CATKIN_DEPENDS roscpp message_runtime
 #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(global_planner src/GlobalPlanner.cpp)

add_executable(global_planner_node src/global_planner_node.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(global_planner_node
   ${catkin_LIBRARIES}
    global_planner
 )

