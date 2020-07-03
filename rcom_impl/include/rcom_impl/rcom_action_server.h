#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <rcom_msgs/rcom.h>
#include <rcom_msgs/rcomAction.h>

#include <rcom_impl/rcom_impl.h>


namespace rcom
{

class RCoMActionServer {
    public:
        RCoMActionServer(
            ros::NodeHandle nh, std::string action_server, std::string control_client, 
            double kt, double krcm, double lambda0, double dt, 
            std::string planning_group, double alpha, std::string link_pi, std::string link_pip1,
            double t1_td, double t1_p_trocar, double t2_td, double t2_p_trocar, int max_iter
        );

    private:
        ros::NodeHandle _nh;

        // Server to handle goals via _computeUpdateCB callback
        std::string _action_server;
        actionlib::SimpleActionServer<rcom_msgs::rcomAction> _as;

        // Client to request joint angle goals on actual robot
        std::string _control_client;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> _ac;

        // Actual RCoM implementation
        rcom::RCoMImpl _rcom;

        // Robot model
        std::string _planning_group;
        double _alpha;
        moveit::planning_interface::MoveGroupInterface _move_group;
        std::string _link_pi, _link_pip1;

        // Error margin and max iterations
        double _t1_td, _t1_p_trocar, _t2_td, _t2_p_trocar;
        int _max_iter;


        // Goal callback
        void _goalCB(const rcom_msgs::rcomGoalConstPtr& goal);

        // State machine
        std::vector<double> _computeUpdate(Eigen::VectorXd& td, Eigen::Vector3d p_trocar);

        // Compute error between current and desired values via forward kinematics
        std::tuple<Eigen::Vector3d, Eigen::Vector3d> _computeForwardKinematics(std::vector<double>& q);

        std::tuple<Eigen::VectorXd, Eigen::Vector3d> _computeError(
            Eigen::VectorXd& td,
            Eigen::Vector3d& pip1,
            Eigen::Vector3d& p_trocar,
            Eigen::Vector3d& prcm
        );

        // Execute goal on client robot
        actionlib::SimpleClientGoalState _executeGoal(std::vector<double> q);

        // Compute goal feedback and result
        template<typename T>
        T _computeFeedback(std::tuple<Eigen::VectorXd, Eigen::Vector3d>& e, Eigen::Vector3d& td, Eigen::Vector3d& p_trocar);
};


RCoMActionServer::RCoMActionServer(
    ros::NodeHandle nh, std::string action_server, std::string control_client, 
    double kt, double krcm, double lambda0, double dt, 
    std::string planning_group, double alpha, std::string link_pi, std::string link_pip1,
    double t1_td, double t1_p_trocar, double t2_td, double t2_p_trocar, int max_iter
) : _action_server(action_server), _as(nh, action_server, boost::bind(&RCoMActionServer::_goalCB, this, _1), false),
    _control_client(control_client), _ac(nh, control_client, false),
    _rcom(kt, krcm, lambda0, dt),
    _planning_group(planning_group),
    _alpha(alpha),
    _move_group(planning_group),
    _link_pi(link_pi),
    _link_pip1(link_pip1),
    _t1_td(t1_td), _t1_p_trocar(t1_p_trocar), _t2_td(t2_td), _t2_p_trocar(t2_p_trocar), _max_iter(max_iter) {    
    
    _as.start();
    _move_group.setMaxVelocityScalingFactor(alpha);
}


void RCoMActionServer::_goalCB(const rcom_msgs::rcomGoalConstPtr& goal) {

    bool update = true;
    
    // Get desired positions from goal
    Eigen::Vector3d td_3d;     // task
    Eigen::Vector3d p_trocar;  // trocar

    tf::vectorMsgToEigen(goal->positions.td, td_3d);
    tf::vectorMsgToEigen(goal->positions.p_trocar, p_trocar);

    Eigen::VectorXd td = td_3d;  // sadly eigen_conversions does not support matrices, of which VectorXd is a special case

    if (_as.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", _action_server.c_str());
        _as.setPreempted();
        update = false;
    }

    int iter = 0;

    // State machine
    while (update) {

        // Compute joint angles that satisfy desired positions
        auto q = _computeUpdate(td, p_trocar);
        auto p = _computeForwardKinematics(q);
        auto prcm = _rcom.computePRCoM(std::get<0>(p), std::get<1>(p));
        auto e = _computeError(td, std::get<1>(p), p_trocar, prcm);

        if (std::get<0>(e).norm() > _t1_td || std::get<1>(e).norm() > _t1_p_trocar ) {
            ROS_INFO("%s: Aborted due to divergent RCoM", _action_server.c_str());
            _as.setAborted();
            update = false;
        }
        else {
            auto status = _executeGoal(q);

            if (status == actionlib::SimpleClientGoalState::SUCCEEDED) {
                q = _move_group.getCurrentJointValues();
                p = _computeForwardKinematics(q);
                prcm = _rcom.computePRCoM(std::get<0>(p), std::get<1>(p));
                e = _computeError(td, std::get<1>(p), p_trocar, prcm);

                // Update lambda to remove drift
                _rcom.feedbackLambda(std::get<0>(p), std::get<1>(p), prcm);

                if (std::get<0>(e).norm() <= _t2_td && std::get<1>(e).norm() <= _t2_p_trocar ) {
                    ROS_INFO("%s: Suceeded", _action_server.c_str());
                    auto rs = _computeFeedback<rcom_msgs::rcomResult>(e, std::get<1>(p), prcm);
                    _as.setSucceeded(rs);
                    update = false;
                }
                else {
                    ROS_INFO("%s: Iterating on joint angles", _action_server.c_str());
                    auto fb = _computeFeedback<rcom_msgs::rcomFeedback>(e, std::get<1>(p), prcm);
                    _as.publishFeedback(fb);

                    if (iter > _max_iter) {
                        ROS_INFO("%s: Aborted due to max_iter", _action_server.c_str());
                        _as.setAborted();
                        update = false;
                    }

                    iter++;
                }
            }
            else {
                ROS_INFO("%s: Aborted due to client %s failure", _action_server.c_str(), _control_client.c_str());
                _as.setAborted();
                update = false;
            }
        }
    }
}


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


std::tuple<Eigen::Vector3d, Eigen::Vector3d> RCoMActionServer::_computeForwardKinematics(std::vector<double>& q) {

    // Compute forward kinematics
    auto robot_model = _move_group.getRobotModel();
    auto robot_state = moveit::core::RobotState(robot_model);

    for (int i = 0; i < q.size(); i++) {
        robot_state.setJointPositions(_move_group.getJointNames()[i], &q[i]);
    }

    auto pi = robot_state.getGlobalLinkTransform(_link_pi).translation();
    auto pip1 = robot_state.getGlobalLinkTransform(_link_pip1).translation();

    return std::make_tuple(pi, pip1);
};


std::tuple<Eigen::VectorXd, Eigen::Vector3d> RCoMActionServer::_computeError(
    Eigen::VectorXd& td,
    Eigen::Vector3d& pip1,
    Eigen::Vector3d& p_trocar,
    Eigen::Vector3d& prcm
) {

    // Compute error
    auto t1_td = td - pip1;
    auto t1_p_trocar = p_trocar - prcm;

    return std::make_tuple(t1_td, t1_p_trocar);
};


actionlib::SimpleClientGoalState RCoMActionServer::_executeGoal(std::vector<double> q) {
    
    // Execute motion on client
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(_rcom.getdt()/_alpha);
    point.positions = q;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = _move_group.getJointNames();
    goal.trajectory.points.push_back(point);

    _ac.sendGoal(goal);
    _ac.waitForResult();

    return _ac.getState();
};


template<typename T>
T RCoMActionServer::_computeFeedback(std::tuple<Eigen::VectorXd, Eigen::Vector3d>& e, Eigen::Vector3d& td, Eigen::Vector3d& p_trocar) {
    T fb;

    // Feedback errors
    tf::vectorEigenToMsg(std::get<0>(e), fb.errors.td);
    tf::vectorEigenToMsg(std::get<1>(e), fb.errors.p_trocar);

    // Feedback positions
    tf::vectorEigenToMsg(td, fb.positions.td);
    tf::vectorEigenToMsg(p_trocar, fb.positions.p_trocar);

    return fb;
};

} // namespace rcom

