cmake_minimum_required(VERSION 2.8.3)
project(rcm_msgs)

find_package(catkin REQUIRED COMPONENTS 
    actionlib
    message_generation 
    geometry_msgs
)

# Generate custom messages
add_message_files(
    FILES p_trocar.msg rcm.msg task.msg
)

add_action_files(
    FILES rcm.action
)

generate_messages(
    DEPENDENCIES geometry_msgs actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs
)
