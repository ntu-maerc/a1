cmake_minimum_required(VERSION 2.8.3)
project(robotic_arm_kinematics)

find_package(catkin REQUIRED COMPONENTS     
             moveit_core
    	     moveit_ros_planning
    	     moveit_ros_planning_interface
             moveit_ros_perception
             rviz_visual_tools
             moveit_visual_tools       
             cmake_modules
             geometry_msgs
             tf2_ros
             geometry_msgs
             tf2_geometry_msgs
)

find_package(Eigen REQUIRED)
find_package(catkin REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotic_arm_kinematics
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
)


add_library(robotic_arm_kinematics
  src/inverse_kinematics.cpp
)

target_link_libraries(robotic_arm_kinematics
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

add_executable(ik_exec src/ik_test.cpp)
target_link_libraries(ik_exec robotic_arm_kinematics ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ik_exec robotic_arm_kinematics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
