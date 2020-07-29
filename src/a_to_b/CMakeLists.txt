cmake_minimum_required(VERSION 3.0.2)
project(a_to_b)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  roscpp
)

# find_package(Boost REQUIRED COMPONENTS system)

## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES a_to_b
#  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs roscpp
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${PROJECT_NAME}/include/${PROJECT_NAME}
 ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/a_to_b.cpp)

target_link_libraries(
   ${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

