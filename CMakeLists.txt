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
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)

add_service_files(
   FILES
   segmentation.srv
)

generate_messages(
   DEPENDENCIES
   sensor_msgs
)


catkin_package(
  INCLUDE_DIRS
      include
  LIBRARIES
      segmentation_lccp
      segmentation_lccp_ros
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
add_library(segmentation_lccp_ros src/segmentation/segmentation_lccp_ros.cpp)

target_link_libraries(segmentation_lccp ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})
target_link_libraries(segmentation_lccp_ros segmentation_lccp ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

install(TARGETS segmentation_lccp segmentation_lccp_ros
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(segmentation_lccp_server src/nodes/segmentation_lccp_server.cpp)
add_dependencies(segmentation_lccp_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(segmentation_lccp_server segmentation_lccp_ros ${catkin_LIBRARIES})

add_executable(segmentation_lccp_client src/nodes/segmentation_lccp_client.cpp)
add_dependencies(segmentation_lccp_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(segmentation_lccp_client segmentation_lccp_ros ${catkin_LIBRARIES})

#############
## Testing ##
#############

catkin_add_gtest(gtest_lccp_algo src/test/gtest_lccp_algo.cpp)
target_link_libraries(gtest_lccp_algo segmentation_lccp ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(test_lccp_algorithm src/test/test_lccp_algorithm.cpp)
add_dependencies(test_lccp_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_lccp_algorithm segmentation_lccp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
