cmake_minimum_required(VERSION 2.8.3)
project(rcom_msgs)

find_package(catkin REQUIRED COMPONENTS 
    actionlib
    message_generation 
    geometry_msgs
)

# Generate custom messages
add_message_files(
    FILES rcom.msg
)

add_message_files(
    FILES values.msg
)

add_action_files(
    FILES rcom.action
)

generate_messages(
    DEPENDENCIES geometry_msgs actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs
)
