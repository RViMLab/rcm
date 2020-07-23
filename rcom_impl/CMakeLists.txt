cmake_minimum_required(VERSION 3.0.2)
project(rcom_impl)

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
    rcom_msgs
)

# Other packages
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp actionlib moveit_ros_planning_interface eigen_conversions tf_conversions tf trajectory_msgs geometry_msgs control_msgs rcom_msgs
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
add_executable(rcom_node src/rcom_node.cpp)
target_link_libraries(rcom_node ${catkin_LIBRARIES})

# Install
install(
    DIRECTORY include/rcom_impl/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
    TARGETS rcom_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
