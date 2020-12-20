#pragma once

#include <exception>
#include <eigen3/Eigen/Core>

#include <rcom_impl/pseudo_inverse.h>


namespace rcom {

/**
 * @brief Implements paper 'Task Control with Remote Center of Motion Constraint for Minimally Invasive Robotic Surgery'
 *        https://ieeexplore.ieee.org/document/6631412
 * 
 * @param kpt task proportional gain (Eigen::VectorXd)
 * @param kit task integral gain (Eigen::VectorXd)
 * @param kdt task differential gain (Eigen::VectorXd)
 * @param krcm remote center of motion gain (Eigen::VectorXd)
 * @param lambda0 initial relative remote center of motion position (double)
 * @param dt control interval in seconds (double)
**/
class RCoMImpl {
    public:
        RCoMImpl(Eigen::VectorXd kpt, Eigen::VectorXd kit, Eigen::VectorXd kdt, Eigen::VectorXd krcm, double lambda0=1.0, double dt=0.1);

        /**
         * @brief eq. 7, see paper
         * 
         * @param td desired task position (Eigen::VectorXd)
         * @param t current task position (Eigen::VectorXd)
         * @param p_trocar trocar position (Eigen::Vector3d)
         * @param p_i see fig. 1, paper (Eigen::Vector3d)
         * @param p_ip1 see fig. 1, paper (Eigen::Vector3d)
         * @param J_i Jacobian at p_i (Eigen::MatrixXd)
         * @param J_ip1 Jacobian at p_ip1 (Eigen::MatrixXd)
         * @param J_t task Jacobian (Eigen::MatrixXd)
        **/
        template<typename derived>
        Eigen::VectorXd computeFeedback(
                Eigen::VectorXd& td, 
                Eigen::VectorXd& t,
                Eigen::Vector3d& p_trocar,
                Eigen::MatrixBase<derived>& p_i,
                Eigen::MatrixBase<derived>& p_ip1,
                Eigen::MatrixXd& J_i,
                Eigen::MatrixXd& J_ip1,
                Eigen::MatrixXd& J_t
        );

        /**
         * @brief eq. 1, see paper
         * 
         * @param p_i see fig. 1, paper (Eigen::Vector3d)
         * @param p_ip1 see fig. 1, paper (Eigen::Vector3d)
        **/
        template<typename derived>
        Eigen::Vector3d computePRCoM(Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1);

        /**
         * @brief feedback lambda to remove drift
         * 
         * @param p_i see fig. 1, paper (Eigen::Vector3d)
         * @param p_ip1 see fig. 1, paper (Eigen::Vector3d)
         * @param p_trocar trocar position (Eigen::Vector3d)
        **/
        void feedbackLambda(Eigen::Vector3d& p_i, Eigen::Vector3d& p_ip1, Eigen::Vector3d& p_trocar);

        /**
         * @brief get _dt
        **/
        inline const double& getdt() const { return _dt; };

    private:

        // gains eq. 7, see paper
        Eigen::VectorXd _kpt;
        Eigen::VectorXd _kit;  // not in paper
        Eigen::VectorXd _kdt;  // not in paper
        Eigen::VectorXd _krcm;

        // eq. 1, see paper
        double _lambda0;
        double _lambda; 

        // Control interval
        double _dt;

        // eq. 3, see paper
        template<typename derived>
        Eigen::MatrixXd _computeRCoMJacobian(Eigen::MatrixXd& J_i, Eigen::MatrixXd& J_ip1, double lambda, Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1);

        // eq. 6 and following, see paper
        Eigen::MatrixXd _computeJacobian(Eigen::MatrixXd& J_t, Eigen::MatrixXd& J_RCM);

        Eigen::VectorXd _computeError(Eigen::VectorXd& td, Eigen::VectorXd& t, Eigen::Vector3d& p_trocar, Eigen::Vector3d& p);
};


// Define functions in header due to templates
RCoMImpl::RCoMImpl(Eigen::VectorXd kpt, Eigen::VectorXd kit, Eigen::VectorXd kdt, Eigen::VectorXd krcm, double lambda0, double dt) 
    : _kpt(kpt), _kit(kit), _kdt(kdt), _krcm(krcm), _lambda0(lambda0), _lambda(_lambda0), _dt(dt) {   }


// eq.7, see paper
template<typename derived>
Eigen::VectorXd RCoMImpl::computeFeedback(
        Eigen::VectorXd& td, 
        Eigen::VectorXd& t,
        Eigen::Vector3d& p_trocar,
        Eigen::MatrixBase<derived>& p_i,
        Eigen::MatrixBase<derived>& p_ip1,
        Eigen::MatrixXd& J_i,
        Eigen::MatrixXd& J_ip1,
        Eigen::MatrixXd& J_t
    ) {

        // Compute Jacobians
        auto J_rcm = _computeRCoMJacobian(J_i, J_ip1, _lambda, p_i, p_ip1);
        auto J = _computeJacobian(J_t, J_rcm);

        // Compute error
        auto p_rcm = computePRCoM(p_i, p_ip1);
        auto e_t = _computeError(td, t, p_trocar, p_rcm);

        // Compute feedback dq, eq. 7
        auto J_pseudo_inverse = pseudoinverse(J);

        int nt = J_t.rows();
        Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3+nt, 3+nt);
        if (nt != _kpt.size()) throw std::runtime_error("Size of _kpt must equal task dimension!");
        if (nt != _kit.size()) throw std::runtime_error("Size of _kit must equal task dimension!");
        if (nt != _kdt.size()) throw std::runtime_error("Size of _kdt must equal task dimension!");
        if (3 != _krcm.size()) throw std::runtime_error("Size of _krcm must equal 3!");
        K.topLeftCorner(nt, nt) = _kpt.asDiagonal();
        K.bottomRightCorner(3, 3) = _krcm.asDiagonal();

        auto dq = J_pseudo_inverse*K*e_t;  // TODO: add other gains here

        // Update lambda
        _lambda += _dt*dq[dq.size() - 1];

        // Return only joint velocities, not dlambda
        return _dt*dq.topRows(dq.size() - 1);
}


// eq. 1, see paper
template<typename derived>
Eigen::Vector3d RCoMImpl::computePRCoM(Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1) {
    return p_i + _lambda*(p_ip1 - p_i);
}


// feedback lambda to remove drift, not in paper
void RCoMImpl::feedbackLambda(Eigen::Vector3d& p_i, Eigen::Vector3d& p_ip1, Eigen::Vector3d& p_trocar) {
    _lambda = (p_ip1 - p_i).normalized().dot(p_trocar - p_i)/(p_ip1 - p_i).norm();
}


// eq. 3, see paper
template<typename derived>
Eigen::MatrixXd RCoMImpl::_computeRCoMJacobian(Eigen::MatrixXd& J_i, Eigen::MatrixXd& J_ip1, double lambda, Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1) {
    auto J_RCM_left  = J_i + lambda*(J_ip1 - J_i); 
    auto J_RCM_right = p_ip1 - p_i;

    Eigen::MatrixXd J_RCM(J_RCM_left.rows(), J_RCM_left.cols() + J_RCM_right.cols());
    J_RCM << J_RCM_left, J_RCM_right;
    return J_RCM;
}


// eq. 6 and following, see paper
Eigen::MatrixXd RCoMImpl::_computeJacobian(Eigen::MatrixXd& J_t, Eigen::MatrixXd& J_RCM) {
    Eigen::MatrixXd J(J_t.rows() + J_RCM.rows(), J_RCM.cols());

    J << J_t, Eigen::VectorXd::Zero(J_t.rows()), J_RCM;
    return J;
}


Eigen::VectorXd RCoMImpl::_computeError(Eigen::VectorXd& td, Eigen::VectorXd& t, Eigen::Vector3d& p_trocar, Eigen::Vector3d& p) {
    auto e_t_upper = td - t;
    auto e_t_lower = p_trocar - p;

    Eigen::VectorXd e_t(e_t_upper.rows() + e_t_lower.rows());
    e_t << e_t_upper, e_t_lower;
    return e_t;
}

} // end of namespace rcom
