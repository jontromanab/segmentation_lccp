cmake_minimum_required(VERSION 2.8.3)
project(segmentation_lccp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)

catkin_package(
  INCLUDE_DIRS
      include
  LIBRARIES
      segmentation_lccp
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

IF (PCL_FOUND)
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
  ADD_DEFINITIONS(${PCL_DEFINITIONS})
ENDIF (PCL_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIR} )


add_library(segmentation_lccp src/segmentation/segmentation_lccp.cpp)
target_link_libraries(segmentation_lccp ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS} )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_segmentation_lccp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(test_lccp_algorithm src/test/test_lccp_algorithm.cpp)
add_dependencies(test_lccp_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_lccp_algorithm segmentation_lccp ${catkin_LIBRARIES})
