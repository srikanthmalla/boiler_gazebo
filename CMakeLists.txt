cmake_minimum_required(VERSION 2.8.3)
project(boiler_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  rospy
  visualization_msgs
  message_generation
  #ca_nav_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  cmake_modules
  mavros_msgs
  tf
  tf_conversions
  # tsp
)

##CGAL COMPONENTS
#find_package(CGAL QUIET COMPONENTS Core )
#if ( CGAL_FOUND )
#  include( ${CGAL_USE_FILE} )
#  include( CGAL_CreateSingleSourceCGALProgram )
#  include_directories (BEFORE ../../include)
#else()
#    message(STATUS "This program requires the CGAL library, and will not be compiled.")
#endif()
## CGAL ends


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(Eigen_INCLUDE_DIRS "/usr/include/eigen3/Eigen/")

add_message_files(
  FILES
  Coorxyz.msg
  trianglewithnormal.msg
  triangularmesh.msg
  djicommand.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Marker_publisher
  CATKIN_DEPENDS roscpp message_runtime
  #DEPENDS system_lib CGAL
)

##pcl stuff started
find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
##pcl stuff ended

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  #${CGAL_INCLUDE_DIRS}
  include
)

add_executable(stlreader src/stlreader.cpp)
target_link_libraries(stlreader ${catkin_LIBRARIES})

# add_executable(planner src/planner.cpp) #backup is tsp_lemon
# target_link_libraries(planner ${catkin_LIBRARIES} emon)

#add_executable(path_gen src/generate_path.cpp)
#target_link_libraries(path_gen ${catkin_LIBRARIES} )

add_executable(orderingwp src/orderingwp.cpp)
target_link_libraries(orderingwp ${catkin_LIBRARIES} )

add_executable(coverage src/coverage.cpp)
target_link_libraries(coverage ${catkin_LIBRARIES} )

add_executable(offboard src/offboard.cpp)
target_link_libraries(offboard ${catkin_LIBRARIES} )

#add_executable(cgal_test src/cgal_test.cpp)
#target_link_libraries(cgal_test ${catkin_LIBRARIES})

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES})
# add_executable(uncovered_points_eth src/uncovered_points_eth.cpp)
# target_link_libraries(uncovered_points_eth ${catkin_LIBRARIES} )
