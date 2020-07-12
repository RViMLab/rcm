#pragma once

#include <rcom_impl/base_rcom_action_server.h>


namespace rcom
{

class RCoMActionServer : BaseRCoMActionServer {

    public:

        RCoMActionServer(      
            ros::NodeHandle nh, std::string action_server, std::string control_client, 
            std::vector<double> kt, double krcm, double lambda0, double dt, 
            std::string planning_group, double alpha, std::string link_pi, std::string link_pip1,
            double t1_td, double t1_p_trocar, double t2_td, double t2_p_trocar, int max_iter
        );

    private:

        // Update remote center of motion with task Jacobian
        virtual std::vector<double> _computeUpdate(Eigen::VectorXd& td, Eigen::Vector3d p_trocar) override;

        // Compute error between current and desired values via forward kinematics
        virtual Eigen::VectorXd _computeTaskForwardKinematics(std::vector<double>& q) override;
};


RCoMActionServer::RCoMActionServer(
    ros::NodeHandle nh, std::string action_server, std::string control_client, 
    std::vector<double> kt, double krcm, double lambda0, double dt, 
    std::string planning_group, double alpha, std::string link_pi, std::string link_pip1,
    double t1_td, double t1_p_trocar, double t2_td, double t2_p_trocar, int max_iter
) : BaseRCoMActionServer(
    nh, action_server, control_client, 
    kt, krcm, lambda0, dt, 
    planning_group, alpha, link_pi, link_pip1, 
    t1_td, t1_p_trocar, t2_td, t2_p_trocar, max_iter) {   };


std::vector<double> RCoMActionServer::_computeUpdate(Eigen::VectorXd& td, Eigen::Vector3d p_trocar) {
    
    // Compute Jacobians and positions of robot model at current pose
    auto robot_state = _move_group.getCurrentState();
    auto q = _move_group.getCurrentJointValues();

    auto pi = robot_state->getGlobalLinkTransform(_link_pi).translation();
    auto pip1 = robot_state->getGlobalLinkTransform(_link_pip1).translation();

    Eigen::MatrixXd Ji;
    Eigen::MatrixXd Jip1;

    robot_state->getJacobian(
        robot_state->getJointModelGroup(_planning_group),
        robot_state->getLinkModel(_link_pi),
        Eigen::Vector3d::Zero(),
        Ji
    );

    robot_state->getJacobian(
        robot_state->getJointModelGroup(_planning_group),
        robot_state->getLinkModel(_link_pip1),
        Eigen::Vector3d::Zero(),
        Jip1
    );

    Ji   = Ji.topRows(3);  // get translational part of Jacobian
    Jip1 = Jip1.topRows(3);

    auto Jt = Jip1;  // task Jacobian here is translation of end-effector

    auto dq = _rcom.computeFeedback(
        td, p_trocar,
        pi, pip1,
        Ji, Jip1, Jt
    );

    for (int i = 0; i < q.size(); i++) {
        q[i] += dq[i];
    }

    return q;
};


Eigen::VectorXd RCoMActionServer::_computeTaskForwardKinematics(std::vector<double>& q) {

    // Compute forward kinematics
    auto robot_model = _move_group.getRobotModel();
    auto robot_state = moveit::core::RobotState(robot_model);

    for (int i = 0; i < q.size(); i++) {
        robot_state.setJointPositions(_move_group.getJointNames()[i], &q[i]);
    }

    auto pip1 = robot_state.getGlobalLinkTransform(_link_pip1).translation();

    return pip1;
};

} // namespace rcom
