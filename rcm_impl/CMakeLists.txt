cmake_minimum_required(VERSION 3.0.2)
project(rcm_impl)

# Catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    moveit_ros_planning_interface
    eigen_conversions
    tf_conversions
    tf
    trajectory_msgs
    geometry_msgs
    control_msgs
    rcm_msgs
)

# Other packages
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp actionlib moveit_ros_planning_interface eigen_conversions tf_conversions tf trajectory_msgs geometry_msgs control_msgs rcm_msgs
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
    DEPENDS EIGEN3
)

# Includes
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Executable
add_executable(three_dof_rcm_node src/three_dof_rcm_node.cpp)
target_link_libraries(three_dof_rcm_node ${catkin_LIBRARIES})

add_executable(four_dof_rcm_node src/four_dof_rcm_node.cpp)
target_link_libraries(four_dof_rcm_node ${catkin_LIBRARIES})

# Install
install(
    DIRECTORY include/rcm_impl/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
    TARGETS three_dof_rcm_node four_dof_rcm_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
