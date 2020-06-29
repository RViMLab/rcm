#include <mutex>
#include <math.h>
#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <Eigen/Dense>


#include <pseudo_inverse.h>




using namespace std;




class RCOM {
    public:
        RCOM(ros::NodeHandle& nh, double dt, double k_t, double k_rcm) :
                _planning_group("arm_endoscope"),
                _move_group(_planning_group) {

            _p_trocar.setZero();
            _t_d.setZero();

            _dt = dt;
            _lambda0 = 0.7; // TODO: externalize lambda init
            _lambda = _lambda0;

            int nt = 3;  // TODO: task dimension
            _K = Eigen::MatrixXd::Zero(3+nt, 3+nt);
            _K.topLeftCorner(nt, nt) = k_t*Eigen::MatrixXd::Identity(nt, nt);
            _K.bottomRightCorner(3, 3) = k_rcm*Eigen::MatrixXd::Identity(3, 3);

            // Publish trajectories
            _traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
                "PositionJointInterface_trajectory_controller/command",
                1
            );

            // Subscribe to p_trocar and t_d
            _p_trocar_sub = nh.subscribe<geometry_msgs::Vector3>(
                "p_trocar",
                1,
                &RCOM::_p_trocar_sub_cb, this
            );

            _t_d_sub = nh.subscribe<geometry_msgs::Vector3>(
                "t_d",
                1,
                &RCOM::_t_d_sub_cb, this
            );

            _move_group.setMaxVelocityScalingFactor(0.1);

            // Go home
            _move_group.setNamedTarget("home");
            _move_group.move();
            _move_group.stop();

            // Set an initial pose, corresponding to p_trocar
            auto joint_values = _move_group.getCurrentJointValues();

            joint_values[1] -= 1.0*M_PI/4.;
            joint_values[3] -= 2.0*M_PI/4.;
            joint_values[5] += 1.0*M_PI/4.;
            

            _move_group.setJointValueTarget(joint_values);
            _move_group.move();
            _move_group.stop();

            // TODO *************************************************
            auto robot_state = _move_group.getCurrentState();

            // Compute positions, includes orientations
            auto p_i = robot_state->getGlobalLinkTransform("lbr_link_ee").translation();
            auto p_ip1 = robot_state->getGlobalLinkTransform("endoscope_link_cm").translation();

            // Compute Jacobians and keep translational velocity
            Eigen::MatrixXd J_i;
            Eigen::MatrixXd J_ip1;
            auto reference = Eigen::Vector3d::Zero();

            robot_state->getJacobian(
                robot_state->getJointModelGroup(_planning_group),
                robot_state->getLinkModel("lbr_link_ee"),
                reference,
                J_i
            );

            robot_state->getJacobian(
                robot_state->getJointModelGroup(_planning_group),
                robot_state->getLinkModel("endoscope_link_cm"),
                reference,
                J_ip1
            );

            J_i   = J_i.topRows(3);
            J_ip1 = J_ip1.topRows(3);
            auto J_t = J_ip1;  // TODO: task Jacobian externally?

            auto p_rcm = _computePRCM(p_i, p_ip1, _lambda);

            _t_d_init = p_ip1;
            _t_d = _t_d_init; // TODO: remove this by actual desired value
            _p_trocar = p_rcm; // TODO: remove this by actual trocar pos
            // TODO *************************************************

            // Timer with callback to execute control
            _timer = nh.createTimer(ros::Duration(dt), &RCOM::_timer_cb, this);
        };

    private:
        int count = 0; // TODO: remove count
        Eigen::Vector3d _t_d_init; // TODO: remove init

        const string _planning_group;
        moveit::planning_interface::MoveGroupInterface _move_group;

        // Reference values, eq.6 and following
        Eigen::Vector3d _p_trocar;
        Eigen::Vector3d _t_d;

        // Control intervals
        double _dt;

        // eq. 1
        double _lambda0;
        double _lambda; 

        // Gain
        Eigen::MatrixXd _K;

        // Publishers
        ros::Publisher _traj_pub;

        // Subscribers
        ros::Subscriber _p_trocar_sub;
        ros::Subscriber _t_d_sub;

        // Subscriber callbacks
        void _p_trocar_sub_cb(const geometry_msgs::Vector3::ConstPtr& msg) {
            tf::vectorMsgToEigen(*msg, _p_trocar);
        };

        void _t_d_sub_cb(const geometry_msgs::Vector3::ConstPtr& msg) {
            tf::vectorMsgToEigen(*msg, _t_d);
        };

        // Timer and timer callback to execute control
        ros::Timer _timer;
        void _timer_cb(const ros::TimerEvent& msg) {

            count++;
            _t_d[0] = _t_d_init[0] + 0.1*sin(count*2*M_PI/500.); // TODO: remove desired
            _t_d[1] = _t_d_init[1] + 0.1*cos(count*2*M_PI/500.) - 0.1;
            // _t_d[2] = _t_d_init[2] + 0.1*sin(count*2*M_PI/500.); // TODO: remove desired

            auto robot_state = _move_group.getCurrentState();

            // Compute positions, includes orientations
            auto p_i = robot_state->getGlobalLinkTransform("lbr_link_ee").translation();
            auto p_ip1 = robot_state->getGlobalLinkTransform("endoscope_link_cm").translation();

            // Compute Jacobians and keep translational velocity
            Eigen::MatrixXd J_i;
            Eigen::MatrixXd J_ip1;
            auto reference = Eigen::Vector3d::Zero();

            robot_state->getJacobian(
                robot_state->getJointModelGroup(_planning_group),
                robot_state->getLinkModel("lbr_link_ee"),
                reference,
                J_i
            );

            robot_state->getJacobian(
                robot_state->getJointModelGroup(_planning_group),
                robot_state->getLinkModel("endoscope_link_cm"),
                reference,
                J_ip1
            );

            J_i   = J_i.topRows(3);
            J_ip1 = J_ip1.topRows(3);
            auto J_t = J_ip1;  // TODO: task Jacobian externally?

            auto p_rcm = _computePRCM(p_i, p_ip1, _lambda);
            auto J_rcm = _computeJacobianRCM(J_i, J_ip1, _lambda, p_i, p_ip1);
            auto J = _computeJacobian(J_t, J_rcm);

            // TODO: use mutex for _t_d, _p_trocar?
            // _t_d[2] -= 0.0001;
            auto e_t = _computeError(_t_d, p_ip1, _p_trocar, p_rcm);  // feel like error is wrongly computed

            
            auto J_pseudo_inverse = pseudoinverse(J);

            auto I = Eigen::MatrixXd::Identity(J_pseudo_inverse.rows(), J.cols());
            auto w = Eigen::VectorXd(J.cols());
            w[w.size() - 1] = _lambda0 - _lambda;
            auto dq = J_pseudo_inverse*_K*e_t; // + (I - J_pseudo_inverse*J)*w;

            // TODO: update control
            double alpha = dq.head(7).maxCoeff()/robot_state->getJointModel(_move_group.getJointNames()[0])->getVariableBounds()[0].max_velocity_;
            // _move_group.setMaxVelocityScalingFactor(alpha);
            _lambda += dq[dq.size() - 1]*_dt;

            cout << "desired: " << _t_d.transpose() << endl;
            cout << "current: " << p_ip1.transpose() << endl;
            
            auto joint_values = _move_group.getCurrentJointValues();
            
            for (int i = 0; i < joint_values.size(); i++) {
                joint_values[i] += dq[i]*_dt;
            }

            auto valid = true;
            for (int i = 0; i < joint_values.size(); i++) {
                auto max = robot_state->getJointModel(_move_group.getJointNames()[i])->getVariableBounds()[0].max_velocity_;
                auto min = robot_state->getJointModel(_move_group.getJointNames()[i])->getVariableBounds()[0].min_velocity_;
                cout << "min: " << min << " req: " << dq[i] << " max: " << max << endl;

                if (max < dq[i] < min) {
                    valid = false;
                    ROS_ERROR("Fatal error, quiting remote center of motion");
                }
            }



            auto trajectory = trajectory_msgs::JointTrajectory();
            auto trajectory_point = trajectory_msgs::JointTrajectoryPoint();

            trajectory_point.positions = joint_values;
            trajectory_point.time_from_start = ros::Duration(_dt);

            trajectory.joint_names = _move_group.getJointNames();
            trajectory.points.push_back(trajectory_point);

            _traj_pub.publish(trajectory);
        }

        // Equations
        // eq. 1
        template<typename derived>
        Eigen::Vector3d _computePRCM(Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1, double lambda) {
            return p_i + lambda*(p_ip1 - p_i);
        }

        // eq. 3
        template<typename derived>
        Eigen::MatrixXd _computeJacobianRCM(Eigen::MatrixXd& J_i, Eigen::MatrixXd& J_ip1, double lambda, Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1) {
            auto J_RCM_left  = J_i + lambda*(J_ip1 - J_i); 
            auto J_RCM_right = p_ip1 - p_i;

            Eigen::MatrixXd J_RCM(J_RCM_left.rows(), J_RCM_left.cols() + J_RCM_right.cols());
            J_RCM << J_RCM_left, J_RCM_right;
            return J_RCM;
        }

        // eq. 6
        Eigen::MatrixXd _computeJacobian(Eigen::MatrixXd& J_t, Eigen::MatrixXd& J_RCM) {
            Eigen::MatrixXd J(J_t.rows() + J_RCM.rows(), J_RCM.cols());

            J << J_t, Eigen::VectorXd::Zero(J_t.rows()), J_RCM;
            return J;
        }

        // eq. 7
        template<typename derived>
        Eigen::VectorXd _computeError(Eigen::Vector3d& td, Eigen::MatrixBase<derived>& t, Eigen::Vector3d& p_trocar, Eigen::Vector3d& p) {
            auto e_t_upper = td - t;
            auto e_t_lower = p_trocar - p;

            Eigen::VectorXd e_t(e_t_upper.rows() + e_t_lower.rows());
            e_t << e_t_upper, e_t_upper;
            return e_t;
        }

};



int main(int argc, char** argv) {

    ros::init(argc, argv, "rcom_node");
    auto nh = ros::NodeHandle();
    auto spinner = ros::AsyncSpinner(2);
    spinner.start();

    auto rcom = RCOM(nh, 0.1, 1e1, 1e0);  // k_t and k_rcm as in paper

    ros::waitForShutdown();

















    // static const string planning_group = "arm_endoscope";
    // auto move_group = moveit::planning_interface::MoveGroupInterface(planning_group);


    // move_group.setMaxVelocityScalingFactor(0.01);

    // // Go home
    // move_group.setNamedTarget("home");
    // move_group.move();
    // move_group.stop();

    // // Set an initial pose, corresponding to p_trocar
    // auto joint_values = move_group.getCurrentJointValues();

    // joint_values[1] += 0.5*M_PI/4.;
    // joint_values[3] -= 2.0*M_PI/4.;
    // joint_values[5] += 0.5*M_PI/4.;

    // move_group.setJointValueTarget(joint_values);
    // move_group.move();
    // move_group.stop();

    // while (ros::ok()) {

    //     auto robot_state = move_group.getCurrentState();

    //     // compute jacobians at different stages
    //     Eigen::MatrixXd jacobian_i;
    //     Eigen::MatrixXd jacobian_ip1;
    //     auto reference = Eigen::Vector3d::Zero();

    //     robot_state->getJacobian(
    //         move_group.getCurrentState()->getJointModelGroup(planning_group),
    //         move_group.getCurrentState()->getLinkModel("lbr_link_ee"),
    //         reference,
    //         jacobian_i
    //     );

    //     robot_state->getJacobian(
    //         move_group.getCurrentState()->getJointModelGroup(planning_group),
    //         move_group.getCurrentState()->getLinkModel("endoscope_link_cm"),
    //         reference,
    //         jacobian_ip1
    //     );

    //     // Compute pseudo inverse
    //     auto pi_jacobian_i = jacobian_i.completeOrthogonalDecomposition().pseudoInverse(); // pseudo inverse


    //     // compute forward kinematics at different stages
    //     auto p_i = robot_state->getGlobalLinkTransform("lbr_link_ee");
    //     auto p_ip1 = robot_state->getGlobalLinkTransform("endoscope_link_cm");

    //     cout << "ji\n" << jacobian_i << endl;
    //     cout << "pi_ji\n" << pi_jacobian_i << endl;
    //     cout << "jip1\n" << jacobian_ip1 << endl;
    //     cout << "pi\n" << p_i.translation() << "\n" << p_i.rotation() << endl;
    //     cout << "p_ip1\n" << p_ip1.translation() << "\n" << p_ip1.rotation() << endl;

    //     ros::Duration(0.5).sleep();
    // }

    // move_group.setNamedTarget("home");
    // move_group.move();
    // move_group.stop();

    // compute jacobian at different stages, that is ee lbr and ee endoscope

    // compute forward kinematics at different stages, that is ee lbr and ee endoscope

    // update q with dq and lambda with dlambda



    // ros::shutdown();
    return 0;
}

