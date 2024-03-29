#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <rcm_impl/three_dof_rcm_action_server.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "three_dof_rcm_node");
    auto nh = ros::NodeHandle();
    auto spinner = ros::AsyncSpinner(2);
    spinner.start();

    // Parameters
    std::string action_server, control_client;
    std::vector<double> kpt, kit, kdt, kprcm, kircm, kdrcm;
    double lambda0, dt;
    std::string planning_group;
    double alpha;
    std::string link_pi, link_pip1;
    double t1_td, t1_p_trocar, t2_td, t2_p_trocar;
    std::vector<double> t_td_scale;
    int max_iter;
    double exp_smooth, dumping;
    bool rcm_priority;

    nh.getParam("action_server", action_server);
    nh.getParam("control_client", control_client);
    nh.getParam("kpt", kpt);
    nh.getParam("kit", kit);
    nh.getParam("kdt", kdt);
    nh.getParam("kprcm", kprcm);
    nh.getParam("kircm", kircm);
    nh.getParam("kdrcm", kdrcm);
    nh.getParam("lambda0", lambda0);
    nh.getParam("dt", dt);
    nh.getParam("planning_group", planning_group);
    nh.getParam("alpha", alpha);
    nh.getParam("link_pi", link_pi);
    nh.getParam("link_pip1", link_pip1);
    nh.getParam("t1_td", t1_td);
    nh.getParam("t_td_scale", t_td_scale);
    nh.getParam("t1_p_trocar", t1_p_trocar);
    nh.getParam("t2_td", t2_td);
    nh.getParam("t2_p_trocar", t2_p_trocar);
    nh.getParam("max_iter", max_iter);
    nh.getParam("exp_smooth", exp_smooth);
    nh.getParam("dumping", dumping);
    nh.getParam("rcm_priority", rcm_priority);

    // Initialize position
    auto move_group = moveit::planning_interface::MoveGroupInterface(planning_group);
    move_group.setMaxVelocityScalingFactor(0.01);

    // Go home
    move_group.setNamedTarget("home");
    move_group.move();
    move_group.stop();

    // Set an initial pose, corresponding to p_trocar
    auto joint_values = move_group.getCurrentJointValues();

    // joint_values[1] = -1.0*M_PI/4.;
    // joint_values[3] = -2.0*M_PI/2.;
    // joint_values[5] = +1.0*M_PI/4.;

    // joint_values[1] = 1.0*M_PI/4.;
    // joint_values[2] = 1.0*M_PI/4.;
    // joint_values[3] = 2.0*M_PI/4.;
    // joint_values[4] = 1.0*M_PI/4.;
    // joint_values[5] = 1.0*M_PI/4.;
    // joint_values[6] = 1.0*M_PI/4.;

    // joint_values[0]  = - 85.15*M_PI/180.;
    // joint_values[1]  = - 41.71*M_PI/180.;
    // joint_values[2]  =    0.79*M_PI/180.;
    // joint_values[3]  =   78.44*M_PI/180.;
    // joint_values[4]  = - 50.62*M_PI/180.;
    // joint_values[5]  = - 20.25*M_PI/180.;
    // joint_values[6]  = - 15.22*M_PI/180.;

    // joint_values[0]  = - 1.1784;
    // joint_values[1]  =   1.0322;
    // joint_values[2]  = - 0.0233;
    // joint_values[3]  = - 1.3586;
    // joint_values[4]  = - 1.6409;
    // joint_values[5]  = - 1.6744;
    // joint_values[6]  =   1.4576;

    // joint_values[0] = -0.0225921;
    // joint_values[1] = -0.0948019;
    // joint_values[2] =  0.0266029;
    // joint_values[3] =  0.9266743;
    // joint_values[4] = -0.0062513;
    // joint_values[5] = -1.2412048;
    // joint_values[6] =  0.0036204;

    // joint_values[0]  =  0.4422;
    // joint_values[1]  = -1.6297;
    // joint_values[2]  = -1.5751;
    // joint_values[3]  = -0.6265;
    // joint_values[4]  =  0.0230;
    // joint_values[5]  =  0.4897;
    // joint_values[6]  = -0.8020;

    joint_values[0]  = -2.2984826451234;
    joint_values[1]  = 1.69281634903135;
    joint_values[2]  = -0.5179222776075;
    joint_values[3]  = 1.98368018817924;
    joint_values[4]  = 2.17922032575187;
    joint_values[5]  = 1.05067073761087;
    joint_values[6]  = 0.41726981677228;

    move_group.setJointValueTarget(joint_values);
    move_group.move();
    move_group.stop();

    move_group.setMaxVelocityScalingFactor(1.0);
    // Action server
    rcm::ThreeDoFRCMActionServer rcm_as(
        nh, action_server, control_client,
        kpt, kit, kdt, kprcm, kircm, kdrcm, lambda0, dt,
        planning_group, alpha, link_pi, link_pip1,
        t1_td, t1_p_trocar, t2_td, t2_p_trocar, t_td_scale, max_iter,
        exp_smooth, dumping, rcm_priority
    );

    ros::waitForShutdown();

    return 0;
}

