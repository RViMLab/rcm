#pragma once

#include <exception>
#include <eigen3/Eigen/Core>
#include <tuple>

#include <rcm_impl/damped_least_squares.h>
#include <rcm_impl/pseudo_inverse.h>


namespace rcm {

/**
 * @brief Implements paper 'Task Control with Remote Center of Motion Constraint for Minimally Invasive Robotic Surgery'
 *        https://ieeexplore.ieee.org/document/6631412
 * 
 * @param kpt task proportional gain (Eigen::VectorXd)
 * @param kit task integral gain (Eigen::VectorXd)
 * @param kdt task differential gain (Eigen::VectorXd)
 * @param kprcm remote center of motion proportional gain (Eigen::VectorXd)
 * @param kircm remote center of motion integral gain (Eigen::VectorXd)
 * @param kdrcm remote center of motion differential gain (Eigen::VectorXd)
 * @param lambda0 initial relative remote center of motion position (double)
 * @param dt control interval in seconds (double)
**/
class RCMImpl {
    public:
        RCMImpl(
            Eigen::VectorXd kpt, Eigen::VectorXd kit, Eigen::VectorXd kdt,
            Eigen::VectorXd kprcm, Eigen::VectorXd kircm, Eigen::VectorXd kdrcm, 
            double lambda0=1.0, double dt=0.1
        );

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
         * @param dumping dumping factor for the pseudo inverse (double)
         * @param lambda_regularizer add lambda regularizer, see eq. 7 paper (bool)
         * @param rcm_priority consider RCM as higher priority and solve task in RCM nullspace (bool)
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
                Eigen::MatrixXd& J_t,
                double dumping=1.e-2,
                bool lambda_regularizer=false,
                bool rcm_priority=false
        );

        /**
         * @brief eq. 1, see paper
         * 
         * @param p_i see fig. 1, paper (Eigen::Vector3d)
         * @param p_ip1 see fig. 1, paper (Eigen::Vector3d)
        **/
        template<typename derived>
        Eigen::Vector3d computePRCM(Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1);

        /**
         * @brief eq. 3, see paper
         * 
         * @param J_i Jacobian at p_i (Eigen::MatrixXd)
         * @param J_ip1 Jacobian at p_ip1 (Eigen::MatrixXd)
         * @param lambda see fig. 1, paper (double)
         * @param p_i see fig. 1, paper (Eigen::Vector3d)
         * @param p_ip1 see fig. 1, paper (Eigen::Vector3d)
        **/
        template<typename derived>
        Eigen::MatrixXd computeRCMJacobian(Eigen::MatrixXd& J_i, Eigen::MatrixXd& J_ip1, double lambda, Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1);

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

        /**
         * @brief get _lambda
        **/
        inline const double& getLambda() const { return _lambda; };

    private:

        // gains eq. 7, see paper
        Eigen::VectorXd _kpt;
        Eigen::VectorXd _kit;  // not in paper
        Eigen::VectorXd _kdt;  // not in paper
        Eigen::VectorXd _kprcm;
        Eigen::VectorXd _kircm;  // not in paper
        Eigen::VectorXd _kdrcm;  // not in paper

        // eq. 1, see paper
        double _lambda0;
        double _lambda; 

        // integral and differential errors, not in paper
        Eigen::VectorXd _ei;
        Eigen::VectorXd _eprev;

        // Control interval
        double _dt;

        // eq. 6 and following, see paper
        Eigen::MatrixXd _computeJacobian(Eigen::MatrixXd& J_t, Eigen::MatrixXd& J_RCM);

        std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> _computeError(Eigen::VectorXd& td, Eigen::VectorXd& t, Eigen::Vector3d& p_trocar, Eigen::Vector3d& p);

        Eigen::VectorXd _computeLambdaRegularizer(Eigen::MatrixXd& J, Eigen::MatrixXd& J_pseudo_inverse, double lambda);
};


// Define functions in header due to templates
RCMImpl::RCMImpl(
    Eigen::VectorXd kpt, Eigen::VectorXd kit, Eigen::VectorXd kdt,
    Eigen::VectorXd kprcm, Eigen::VectorXd kircm, Eigen::VectorXd kdrcm, 
    double lambda0, double dt)
    : _kpt(kpt), _kit(kit), _kdt(kdt), _kprcm(kprcm), _kircm(kircm), _kdrcm(kdrcm), _lambda0(lambda0), _lambda(_lambda0), _dt(dt) {   }


// eq.7, see paper
template<typename derived>
Eigen::VectorXd RCMImpl::computeFeedback(
        Eigen::VectorXd& td, 
        Eigen::VectorXd& t,
        Eigen::Vector3d& p_trocar,
        Eigen::MatrixBase<derived>& p_i,
        Eigen::MatrixBase<derived>& p_ip1,
        Eigen::MatrixXd& J_i,
        Eigen::MatrixXd& J_ip1,
        Eigen::MatrixXd& J_t,
        double dumping,
        bool lambda_regularizer,
        bool rcm_priority
    ) {

        // Compute Jacobians
        auto J_rcm = computeRCMJacobian(J_i, J_ip1, _lambda, p_i, p_ip1);
        auto J = _computeJacobian(J_t, J_rcm);

        // Compute error
        auto p_rcm = computePRCM(p_i, p_ip1);
        auto e = _computeError(td, t, p_trocar, p_rcm);
        auto ep = std::get<0>(e);
        auto ei = std::get<1>(e);
        auto ed = std::get<2>(e);

        int nt = J_t.rows();
        Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(3+nt, 3+nt);
        Eigen::MatrixXd Ki = Eigen::MatrixXd::Zero(3+nt, 3+nt);
        Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(3+nt, 3+nt);
        if (nt != _kpt.size()) throw std::runtime_error("Size of _kpt must equal task dimension!");
        if (nt != _kit.size()) throw std::runtime_error("Size of _kit must equal task dimension!");
        if (nt != _kdt.size()) throw std::runtime_error("Size of _kdt must equal task dimension!");
        if (3 != _kprcm.size()) throw std::runtime_error("Size of _kprcm must equal 3!");
        if (3 != _kircm.size()) throw std::runtime_error("Size of _kircm must equal 3!");
        if (3 != _kdrcm.size()) throw std::runtime_error("Size of _kdrcm must equal 3!");
        Kp.topLeftCorner(nt, nt) = _kpt.asDiagonal();
        Ki.topLeftCorner(nt, nt) = _kit.asDiagonal();
        Kd.topLeftCorner(nt, nt) = _kdt.asDiagonal();
        Kp.bottomRightCorner(3, 3) = _kprcm.asDiagonal();
        Ki.bottomRightCorner(3, 3) = _kircm.asDiagonal();
        Kd.bottomRightCorner(3, 3) = _kdrcm.asDiagonal();

        // Compute feedback dq, eq. 7
        Eigen::VectorXd dq;
        if (rcm_priority) {
            auto J_task_inverse = pseudoinverse(J.topRows(J_t.rows()), 0.);
            auto J_rcm_inverse = pseudoinverse(J.bottomRows(J_rcm.rows()), 0.);

            auto N_rcm = (Eigen::MatrixXd::Identity(J_rcm_inverse.rows(), J_rcm_inverse.rows()) - J_rcm_inverse*J_rcm);

            auto dq_rcm = J_rcm_inverse*(Kp.bottomRightCorner(3, 3)*ep.tail(3) + Ki.bottomRightCorner(3, 3)*ei.tail(3) + Kd.bottomRightCorner(3, 3)*ed.tail(3));
            auto dq_task = N_rcm*J_task_inverse*(Kp.topLeftCorner(nt, nt)*ep.head(nt) + Kd.topLeftCorner(nt, nt)*ed.head(nt) + Ki.topLeftCorner(nt, nt)*ei.head(nt));

            dq = dq_rcm + dq_task;
        }
        else {
            // auto J_inverse = pseudoinverse(J);
            auto J_inverse = dampedLeastSquares(J, dumping);

            dq = J_inverse*(Kp*ep + Ki*ei + Kd*ed);
        
            if (lambda_regularizer) {
                dq += _computeLambdaRegularizer(J, J_inverse, _lambda);
            }
        }

        // Update lambda
        _lambda += _dt*dq[dq.size() - 1];

        // Return only joint velocities, not dlambda
        return _dt*dq.topRows(dq.size() - 1);
}


// eq. 1, see paper
template<typename derived>
Eigen::Vector3d RCMImpl::computePRCM(Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1) {
    return p_i + _lambda*(p_ip1 - p_i);
}


// eq. 3, see paper
template<typename derived>
Eigen::MatrixXd RCMImpl::computeRCMJacobian(Eigen::MatrixXd& J_i, Eigen::MatrixXd& J_ip1, double lambda, Eigen::MatrixBase<derived>& p_i, Eigen::MatrixBase<derived>& p_ip1) {
    auto J_RCM_left  = J_i + lambda*(J_ip1 - J_i); 
    auto J_RCM_right = p_ip1 - p_i;

    Eigen::MatrixXd J_RCM(J_RCM_left.rows(), J_RCM_left.cols() + J_RCM_right.cols());
    J_RCM << J_RCM_left, J_RCM_right;
    return J_RCM;
}


// feedback lambda to remove drift, not in paper
void RCMImpl::feedbackLambda(Eigen::Vector3d& p_i, Eigen::Vector3d& p_ip1, Eigen::Vector3d& p_trocar) {
    _lambda = (p_ip1 - p_i).normalized().dot(p_trocar - p_i)/(p_ip1 - p_i).norm();
}


// eq. 6 and following, see paper
Eigen::MatrixXd RCMImpl::_computeJacobian(Eigen::MatrixXd& J_t, Eigen::MatrixXd& J_RCM) {
    Eigen::MatrixXd J(J_t.rows() + J_RCM.rows(), J_RCM.cols());

    J << J_t, Eigen::VectorXd::Zero(J_t.rows()), J_RCM;
    return J;
}


std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> RCMImpl::_computeError(Eigen::VectorXd& td, Eigen::VectorXd& t, Eigen::Vector3d& p_trocar, Eigen::Vector3d& p) {
    auto e_upper = td - t;
    auto e_lower = p_trocar - p;

    Eigen::VectorXd ep(e_upper.rows() + e_lower.rows());
    Eigen::VectorXd ei(e_upper.rows() + e_lower.rows());
    Eigen::VectorXd ed(e_upper.rows() + e_lower.rows());
    ep << e_upper, e_lower;

    if (ep.size() != _ei.size()) {
        _ei.resize(ep.size());
        _ei.setZero();
    };
    if (ep.size() != _eprev.size()) {
        _eprev.resize(ep.size());
        _eprev.setZero();
    };

    // compute integral and differential errors
    _ei += ep;
    ei = _ei;
    ed = (ep - _eprev)/_dt;
    _eprev = ep;

    return std::make_tuple(ep, ei, ed);
}

Eigen::VectorXd RCMImpl::_computeLambdaRegularizer(Eigen::MatrixXd& J, Eigen::MatrixXd& J_pseudo_inverse, double lambda) {
    Eigen::VectorXd w(J.cols());
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(J.cols(), J.cols());
    w[w.size() - 1] = _lambda0 - lambda;
    return (identity - J_pseudo_inverse*J)*w;
};

} // end of namespace rcm
