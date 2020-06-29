#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <rcom_impl/rcom_action_server.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "rcom_node");
    auto nh = ros::NodeHandle();
    auto spinner = ros::AsyncSpinner(2);
    spinner.start();

    // Initialize position
    auto move_group = moveit::planning_interface::MoveGroupInterface("arm_endoscope");
    move_group.setMaxVelocityScalingFactor(1.0);

    // Go home
    move_group.setNamedTarget("home");
    move_group.move();
    move_group.stop();

    // Set an initial pose, corresponding to p_trocar
    auto joint_values = move_group.getCurrentJointValues();

    joint_values[1] -= 1.0*M_PI/4.;
    joint_values[3] -= 2.0*M_PI/4.;
    joint_values[5] += 1.0*M_PI/4.;
    

    move_group.setJointValueTarget(joint_values);
    move_group.move();
    move_group.stop();

    // RCOMNode rcom(nh, 0.1, 1e0, 1e-1);  // k_t and k_rcm as in paper
    rcom::RCoMActionServer rcom_as(
        nh, "RCoM_ActionServer", "PositionJointInterface_trajectory_controller/follow_joint_trajectory",
        1e0, 1e-1, 0.7, 0.1,
        "arm_endoscope", 0.1, "lbr_link_ee", "endoscope_link_cm",
        1., 1., 10    
    );

    ros::waitForShutdown();

    return 0;
}

