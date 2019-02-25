cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_util)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  Eigen3 REQUIRED
  hector_uav_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
    INCLUDE_DIRS 
    LIBRARIES
    CATKIN_DEPENDS roscpp hector_uav_msgs rosbag
    DEPENDS  
)


set(SRC 
	${PROJECT_SOURCE_DIR}/src/main.cpp
	${PROJECT_SOURCE_DIR}/src/joy_sim.cpp	
  ${PROJECT_SOURCE_DIR}/src/navigator.cpp	
  ${PROJECT_SOURCE_DIR}/src/PIDcontroller.cpp	
  ${PROJECT_SOURCE_DIR}/src/ctrl_interface.cpp	
  
)
add_executable(hector_joy_sim ${SRC}) 
target_link_libraries(hector_joy_sim ${catkin_LIBRARIES})
add_dependencies(hector_joy_sim ${catkin_EXPORTED_TARGETS})


install(TARGETS hector_joy_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 
#find_package(Eigen3 REQUIRED)

