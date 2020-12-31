#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
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

class BaseRCoMActionServer {
    public:
        BaseRCoMActionServer(
            ros::NodeHandle nh, std::string action_server, std::string control_client, 
            std::vector<double> kpt, std::vector<double> kit, std::vector<double> kdt,
            std::vector<double> kprcm, std::vector<double> kircm, std::vector<double> kdrcm, double lambda0, double dt, 
            std::string planning_group, double alpha, std::string link_pi, std::string link_pip1,
            double t1_td, double t1_p_trocar, double t2_td, double t2_p_trocar, std::vector<double> t_td_scale, int max_iter
        );

        ~BaseRCoMActionServer();

    protected:
        ros::NodeHandle _nh;

        // Server to handle goals via _computeUpdateCB callback
        std::string _action_server;
        actionlib::SimpleActionServer<rcom_msgs::rcomAction> _as;

        // Client to request joint angle goals on actual robot
        std::string _control_client;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> _ac;

        // Publisher to publish current state via timer callback this->_timerCB
        ros::Timer _timer;
        ros::Publisher _state_pub;

        // Actual RCoM implementation
        rcom::RCoMImpl _rcom;

        // Robot model
        std::string _planning_group;
        double _alpha;
        moveit::planning_interface::MoveGroupInterface _move_group;
        std::string _link_pi, _link_pip1;

        // Error margin and max iterations
        double _t1_td, _t1_p_trocar, _t2_td, _t2_p_trocar;
        Eigen::VectorXd _t_td_scale;
        int _max_iter;

        // Goal callback, state machine
        void _goalCB(const rcom_msgs::rcomGoalConstPtr& goal);

        // Timer callback, publish current state
        void _timerCB(const ros::TimerEvent&);

        // Compute task Jacobian
        virtual Eigen::MatrixXd _computeTaskJacobian(moveit::core::RobotStatePtr robot_state) = 0;

        // Compute error between current and desired values via forward kinematics
        virtual Eigen::VectorXd _computeTaskForwardKinematics(std::vector<double>& q) = 0;

        // Transform task
        virtual Eigen::VectorXd _transformTask(Eigen::VectorXd& td);

        // Update joint angles
        virtual std::vector<double> _computeUpdate(Eigen::VectorXd& td, Eigen::Vector3d p_trocar);

        std::tuple<Eigen::Vector3d, Eigen::Vector3d> _computeRCoMForwardKinematics(std::vector<double>& q);

        std::tuple<Eigen::VectorXd, Eigen::Vector3d> _computeError(
            const Eigen::VectorXd& td,
            const Eigen::VectorXd& t,
            const Eigen::Vector3d& p_trocar,
            const Eigen::Vector3d& prcm
        );

        // Compute joint velocities
        Eigen::VectorXd _computeJointVelocities(moveit::core::RobotStatePtr robot_state);

        // Execute goal on client robot
        actionlib::SimpleClientGoalState _executeGoal(std::vector<double> q, bool wait_for_result=true);

        // Compute goal feedback and result
        template<typename T>
        T _computeFeedback(std::tuple<Eigen::VectorXd, Eigen::Vector3d>& e, Eigen::VectorXd& t, Eigen::Vector3d& p_trocar, bool is_velocity=false);

        // State machines
        actionlib::SimpleClientGoalState _positionControlStateMachine(Eigen::VectorXd& td, Eigen::Vector3d p_trocar);
        actionlib::SimpleClientGoalState _velocityControlStateMachine(Eigen::VectorXd& td, Eigen::Vector3d p_trocar);
};


BaseRCoMActionServer::BaseRCoMActionServer(
    ros::NodeHandle nh, std::string action_server, std::string control_client, 
    std::vector<double> kpt, std::vector<double> kit, std::vector<double> kdt,
    std::vector<double> kprcm, std::vector<double> kircm, std::vector<double> kdrcm, double lambda0, double dt, 
    std::string planning_group, double alpha, std::string link_pi, std::string link_pip1,
    double t1_td, double t1_p_trocar, double t2_td, double t2_p_trocar, std::vector<double> t_td_scale, int max_iter
) : _action_server(action_server), _as(nh, action_server, boost::bind(&BaseRCoMActionServer::_goalCB, this, _1), false),
    _control_client(control_client), _ac(nh, control_client, false),
    _rcom(
        Eigen::Map<Eigen::VectorXd>(kpt.data(), kpt.size()), 
        Eigen::Map<Eigen::VectorXd>(kit.data(), kit.size()), 
        Eigen::Map<Eigen::VectorXd>(kdt.data(), kdt.size()), 
        Eigen::Map<Eigen::VectorXd>(kprcm.data(), kprcm.size()), 
        Eigen::Map<Eigen::VectorXd>(kircm.data(), kircm.size()), 
        Eigen::Map<Eigen::VectorXd>(kdrcm.data(), kdrcm.size()), 
        lambda0, dt
    ),
    _planning_group(planning_group),
    _alpha(alpha),
    _move_group(planning_group),
    _link_pi(link_pi),
    _link_pip1(link_pip1),
    _t1_td(t1_td), _t1_p_trocar(t1_p_trocar), _t2_td(t2_td), _t2_p_trocar(t2_p_trocar), _t_td_scale(Eigen::Map<Eigen::VectorXd>(t_td_scale.data(), t_td_scale.size())), _max_iter(max_iter) {    
    
    _as.start();
    _move_group.setMaxVelocityScalingFactor(alpha);

    // enable dynamics copying for velocity extraction from move_group https://github.com/ros-planning/moveit_ros/pull/622
    auto current_state_monitor = moveit::planning_interface::getSharedStateMonitor(_move_group.getRobotModel(), moveit::planning_interface::getSharedTF());
    current_state_monitor->enableCopyDynamics(true);

    _timer = nh.createTimer(ros::Duration(dt), &BaseRCoMActionServer::_timerCB, this);
    _state_pub = nh.advertise<rcom_msgs::rcom>(action_server + "/state", 1);
}


BaseRCoMActionServer::~BaseRCoMActionServer() {
    _nh.shutdown();
    _as.shutdown();
    _ac.cancelAllGoals();
    _timer.stop();

    _state_pub.shutdown();
}


void BaseRCoMActionServer::_goalCB(const rcom_msgs::rcomGoalConstPtr& goal) {
    
    // Get desired positions from goal
    Eigen::VectorXd td;        // task
    Eigen::Vector3d p_trocar;  // trocar

    td = Eigen::VectorXd::Map(goal->states.task.values.data(), goal->states.task.values.size());  // sadly eigen_conversions does not support matrices, of which VectorXd is a special case
    td = _transformTask(td);

    // Read trocar position
    tf::vectorMsgToEigen(goal->states.p_trocar.position, p_trocar);

    // State machines
    if (goal->states.task.is_velocity) {  // Handle task velocity goal
        _velocityControlStateMachine(td, p_trocar);
        return;
    }
    if (!goal->states.task.is_velocity){  // Handle task position goal
        _positionControlStateMachine(td, p_trocar);
        return;
    }
}


void BaseRCoMActionServer::_timerCB(const ros::TimerEvent&) {

    // Compute current state
    auto q = _move_group.getCurrentJointValues();
    auto p = _computeRCoMForwardKinematics(q);
    auto prcm = _rcom.computePRCoM(std::get<0>(p), std::get<1>(p));
    auto t = _computeTaskForwardKinematics(q);

    // Publish current state
    rcom_msgs::rcom msg;
    msg.task.values.resize(t.size());

    tf::vectorEigenToMsg(prcm, msg.p_trocar.position);
    Eigen::VectorXd::Map(msg.task.values.data(), msg.task.values.size()) = t;

    _state_pub.publish(msg);
}


Eigen::VectorXd BaseRCoMActionServer::_transformTask(Eigen::VectorXd& td) {
    // Can be used to transform the task
    return td;
}


std::vector<double> BaseRCoMActionServer::_computeUpdate(Eigen::VectorXd& td, Eigen::Vector3d p_trocar) {
    
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

    Eigen::MatrixXd Jt = _computeTaskJacobian(robot_state);
    Eigen::VectorXd t = _computeTaskForwardKinematics(q);

    Ji   = Ji.topRows(3);  // get translational part of Jacobian
    Jip1 = Jip1.topRows(3);

    auto dq = _rcom.computeFeedback(
        td, t, 
        p_trocar, pi, pip1,
        Ji, Jip1, Jt
    );

    for (int i = 0; i < q.size(); i++) {
        q[i] += dq[i];
    }

    return q;
};


std::tuple<Eigen::Vector3d, Eigen::Vector3d> BaseRCoMActionServer::_computeRCoMForwardKinematics(std::vector<double>& q) {

    // Compute forward kinematics
    auto robot_model = _move_group.getRobotModel();
    auto robot_state = moveit::core::RobotState(robot_model);

    robot_state.setJointGroupPositions(robot_state.getJointModelGroup(_move_group.getName()), q);

    auto pi = robot_state.getGlobalLinkTransform(_link_pi).translation();
    auto pip1 = robot_state.getGlobalLinkTransform(_link_pip1).translation();

    return std::make_tuple(pi, pip1);
};


std::tuple<Eigen::VectorXd, Eigen::Vector3d> BaseRCoMActionServer::_computeError(
    const Eigen::VectorXd& td,
    const Eigen::VectorXd& t,
    const Eigen::Vector3d& p_trocar,
    const Eigen::Vector3d& prcm
) {

    // Compute error
    if (td.size() != _t_td_scale.size()) throw std::runtime_error("Size of t_td_scale must equal task dimension!");
    auto t_td = _t_td_scale.asDiagonal()*(td - t);
    auto t_p_trocar = p_trocar - prcm;

    return std::make_tuple(t_td, t_p_trocar);
};


Eigen::VectorXd BaseRCoMActionServer::_computeJointVelocities(moveit::core::RobotStatePtr robot_state) { 
    Eigen::VectorXd dq(_move_group.getJointNames().size());

    if (robot_state->hasVelocities()) {  // not coppied by default https://github.com/ros-planning/moveit_ros/pull/622
        robot_state->copyJointGroupVelocities(robot_state->getJointModelGroup(_move_group.getName()), dq);
    }

    return dq;
}


actionlib::SimpleClientGoalState BaseRCoMActionServer::_executeGoal(std::vector<double> q, bool wait_for_result) {
    
    // Execute motion on client
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(_rcom.getdt()/_alpha);
    point.positions = q;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = _move_group.getJointNames();
    goal.trajectory.points.push_back(point);

    _ac.sendGoal(goal);
    if (wait_for_result) _ac.waitForResult();

    return _ac.getState();
};


template<typename T>
T BaseRCoMActionServer::_computeFeedback(std::tuple<Eigen::VectorXd, Eigen::Vector3d>& e, Eigen::VectorXd& t, Eigen::Vector3d& p_trocar, bool is_velocity) {
    T fb;

    fb.errors.task.is_velocity = is_velocity;
    fb.states.task.is_velocity = is_velocity;

    // Allocate space for mapping
    fb.errors.task.values.resize(std::get<0>(e).size());
    fb.states.task.values.resize(t.size());

    // Feedback errors
    Eigen::VectorXd::Map(fb.errors.task.values.data(), std::get<0>(e).size()) = std::get<0>(e);
    tf::vectorEigenToMsg(std::get<1>(e), fb.errors.p_trocar.position);

    // Feedback task and trocar position
    Eigen::VectorXd::Map(fb.states.task.values.data(), t.size()) = t;
    tf::vectorEigenToMsg(p_trocar, fb.states.p_trocar.position);

    return fb;
};


actionlib::SimpleClientGoalState BaseRCoMActionServer::_positionControlStateMachine(Eigen::VectorXd& td, Eigen::Vector3d p_trocar) {

    for (int i = 0; i < _max_iter; i++) {

        if (_as.isPreemptRequested() || !ros::ok()) {
            ROS_INFO("%s: Preempted", _action_server.c_str());
            _as.setPreempted();
            return actionlib::SimpleClientGoalState::PREEMPTED;
        }

        // Compute joint angles that satisfy desired positions
        auto q = _computeUpdate(td, p_trocar);
        auto p = _computeRCoMForwardKinematics(q);
        auto prcm = _rcom.computePRCoM(std::get<0>(p), std::get<1>(p));
        auto t = _computeTaskForwardKinematics(q);
        auto e = _computeError(td, t, p_trocar, prcm);

        if (std::get<0>(e).norm() > _t1_td) {
            ROS_INFO("%s: Aborted due to divergent task\npi:   (%f, %f, ,%f)\nprcm: (%f, %f, ,%f)\npip1: (%f, %f, %f)", _action_server.c_str(), std::get<0>(p)[0], std::get<0>(p)[1], std::get<0>(p)[2], prcm[0], prcm[1], prcm[2], std::get<1>(p)[0], std::get<1>(p)[1], std::get<1>(p)[2]);
            _as.setAborted();
            return actionlib::SimpleClientGoalState::REJECTED;
        } 
        else if (std::get<1>(e).norm() > _t1_p_trocar) {
            ROS_INFO("%s: Aborted due to divergent RCoM\npi:   (%f, %f, ,%f)\nprcm: (%f, %f, ,%f)\npip1: (%f, %f, %f)", _action_server.c_str(), std::get<0>(p)[0], std::get<0>(p)[1], std::get<0>(p)[2], prcm[0], prcm[1], prcm[2], std::get<1>(p)[0], std::get<1>(p)[1], std::get<1>(p)[2]);
            _as.setAborted();
            return actionlib::SimpleClientGoalState::REJECTED;
        }
        else {
            auto status = _executeGoal(q, true);

            if (status == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    q = _move_group.getCurrentJointValues();
                    p = _computeRCoMForwardKinematics(q);
                    prcm = _rcom.computePRCoM(std::get<0>(p), std::get<1>(p));
                    t = _computeTaskForwardKinematics(q);
                    e = _computeError(td, t, p_trocar, prcm);

                    // Update lambda to remove drift
                    _rcom.feedbackLambda(std::get<0>(p), std::get<1>(p), prcm);

                if (std::get<0>(e).norm() <= _t2_td && std::get<1>(e).norm() <= _t2_p_trocar ) {
                    ROS_INFO("%s: Suceeded\npi:   (%f, %f, ,%f)\nprcm: (%f, %f, ,%f)\npip1: (%f, %f, %f)", _action_server.c_str(), std::get<0>(p)[0], std::get<0>(p)[1], std::get<0>(p)[2], prcm[0], prcm[1], prcm[2], std::get<1>(p)[0], std::get<1>(p)[1], std::get<1>(p)[2]);
                    auto fb = _computeFeedback<rcom_msgs::rcomFeedback>(e, t, prcm, false);
                    auto rs = _computeFeedback<rcom_msgs::rcomResult>(e, t, prcm, false);
                    _as.publishFeedback(fb);
                    _as.setSucceeded(rs);
                    return actionlib::SimpleClientGoalState::SUCCEEDED;
                }
                else {
                    ROS_INFO("%s: Iterating on joint angles\npi:   (%f, %f, ,%f)\nprcm: (%f, %f, ,%f)\npip1: (%f, %f, %f)", _action_server.c_str(), std::get<0>(p)[0], std::get<0>(p)[1], std::get<0>(p)[2], prcm[0], prcm[1], prcm[2], std::get<1>(p)[0], std::get<1>(p)[1], std::get<1>(p)[2]);
                    auto fb = _computeFeedback<rcom_msgs::rcomFeedback>(e, t, prcm, false);
                    _as.publishFeedback(fb);
                }
            }
            else {
                ROS_INFO("%s: Aborted due to client %s failure", _action_server.c_str(), _control_client.c_str());
                _as.setAborted();
                return actionlib::SimpleClientGoalState::ABORTED;
            }
        }
    }

    ROS_INFO("%s: Aborted due to max_iter", _action_server.c_str());
    _as.setAborted();
    return actionlib::SimpleClientGoalState::ABORTED;
};


actionlib::SimpleClientGoalState BaseRCoMActionServer::_velocityControlStateMachine(Eigen::VectorXd& td, Eigen::Vector3d p_trocar) {

    if (_as.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", _action_server.c_str());
        _as.setPreempted();
        return actionlib::SimpleClientGoalState::PREEMPTED;
    }

    auto dtd = td;  // safe velocity task

    // Convert velocity task to position task
    auto q = _move_group.getCurrentJointValues();
    auto t = _computeTaskForwardKinematics(q);
    td = t + _rcom.getdt()*dtd;

    // Compute joint angles that satisfy desired positions
    q = _computeUpdate(td, p_trocar);
    auto p = _computeRCoMForwardKinematics(q);
    auto prcm = _rcom.computePRCoM(std::get<0>(p), std::get<1>(p));
    auto robot_state = _move_group.getCurrentState();
    auto dq = _computeJointVelocities(robot_state);
    Eigen::VectorXd dt = _computeTaskJacobian(robot_state)*dq;  // compute task velocity via task Jacobian, Jdq = dt
    auto e = _computeError(dtd, dt, p_trocar, prcm);

    if (std::get<0>(e).norm() > _t1_td) {
        ROS_INFO("%s: Aborted due to divergent task\npi:   (%f, %f, ,%f)\nprcm: (%f, %f, ,%f)\npip1: (%f, %f, %f)", _action_server.c_str(), std::get<0>(p)[0], std::get<0>(p)[1], std::get<0>(p)[2], prcm[0], prcm[1], prcm[2], std::get<1>(p)[0], std::get<1>(p)[1], std::get<1>(p)[2]);
        _as.setAborted();
        return actionlib::SimpleClientGoalState::REJECTED;
    } 
    else if (std::get<1>(e).norm() > _t1_p_trocar) {
        ROS_INFO("%s: Aborted due to divergent RCoM\npi:   (%f, %f, ,%f)\nprcm: (%f, %f, ,%f)\npip1: (%f, %f, %f)", _action_server.c_str(), std::get<0>(p)[0], std::get<0>(p)[1], std::get<0>(p)[2], prcm[0], prcm[1], prcm[2], std::get<1>(p)[0], std::get<1>(p)[1], std::get<1>(p)[2]);
        _as.setAborted();
        return actionlib::SimpleClientGoalState::REJECTED;
    }
    else {
        auto status = _executeGoal(q, false);  // don't wait for execution to finish

        if (status == actionlib::SimpleClientGoalState::SUCCEEDED || actionlib::SimpleClientGoalState::ACTIVE || actionlib::SimpleClientGoalState::PENDING) {
            q = _move_group.getCurrentJointValues();
            p = _computeRCoMForwardKinematics(q);
            prcm = _rcom.computePRCoM(std::get<0>(p), std::get<1>(p));
            robot_state = _move_group.getCurrentState();
            dq = _computeJointVelocities(robot_state);
            dt = _computeTaskJacobian(robot_state)*dq;  // compute task velocity via task Jacobian, Jdq = dt
            e = _computeError(dtd, dt, p_trocar, prcm);

            // Update lambda to remove drift
            _rcom.feedbackLambda(std::get<0>(p), std::get<1>(p), prcm);

            ROS_INFO("%s: Suceeded\npi:   (%f, %f, ,%f)\nprcm: (%f, %f, ,%f)\npip1: (%f, %f, %f)", _action_server.c_str(), std::get<0>(p)[0], std::get<0>(p)[1], std::get<0>(p)[2], prcm[0], prcm[1], prcm[2], std::get<1>(p)[0], std::get<1>(p)[1], std::get<1>(p)[2]);
            auto fb = _computeFeedback<rcom_msgs::rcomFeedback>(e, dt, prcm, true);  // TODO: update feedback
            auto rs = _computeFeedback<rcom_msgs::rcomResult>(e, dt, prcm, true);
            _as.publishFeedback(fb);
            _as.setSucceeded(rs);
            return actionlib::SimpleClientGoalState::SUCCEEDED;
        }
        else {
            ROS_INFO("%s: Aborted due to client %s failure", _action_server.c_str(), _control_client.c_str());
            _as.setAborted();
            return actionlib::SimpleClientGoalState::ABORTED;
        }
    }
};


} // namespace rcom
