#pragma once

#include <Eigen/Core>


// taken from https://gist.github.com/javidcf/25066cf85e71105d57b6
// with singularity avoidance e.g. https://studywolf.wordpress.com/2013/10/10/robot-control-part-6-handling-singularities/
//   - singular values s become small close to singularities 
//       -> the inverse 1/s explodes and leads to uncontrolled motion
//   - increasing the tolerance limits the value of 1/s
// damped least squares solutions: http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf
// trust region optimization https://en.wikipedia.org/wiki/Trust_region
// trust region https://people.maths.ox.ac.uk/nakatsukasa/preprints/TRSrev2.pdf
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-2}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
};
